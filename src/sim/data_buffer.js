'use strict';

const { secToTimestamp, timestampToSec } = require('./clock');
const dataBufferPb = require('../bosdyn/api/data_buffer_pb');
const dataIndexPb = require('../bosdyn/api/data_index_pb');
const { TimeRange } = require('../bosdyn/api/time_range_pb');
const { LoggerUtil } = require('../loggerUtil');

const logger = LoggerUtil.getLogger('DATA_BUFFER');

// Records kept in memory, per kind.
const MAX_RECORDS = 20000;
// The records are grouped in pages of one minute, like the BDDF pages of a real robot.
const PAGE_SEC = 60;

/**
 * @param {string} glob A glob with '*'.
 * @returns {RegExp}
 */
function globToRegExp(glob) {
  return new RegExp(
    `^${glob
      .split('*')
      .map(part => part.replace(/[.+?^${}()|[\]\\]/g, '\\$&'))
      .join('.*')}$`,
  );
}

/**
 * @param {?TimeRange} range
 * @returns {{start: number, end: number}}
 */
function rangeOf(range) {
  return {
    start: range?.hasStart() ? timestampToSec(range.getStart()) : -Infinity,
    end: range?.hasEnd() ? timestampToSec(range.getEnd()) : Infinity,
  };
}

/**
 * The data buffer of the robot (text messages, operator comments, events, blobs, signals), and the data service which
 * indexes it in pages.
 */
class DataBuffer {
  /**
   * @param {import('../robot')} robot
   */
  constructor(robot) {
    this.robot = robot;
    /** @type {{kind: string, time: number, source: string, channel: string, typeId: string, bytes: number,
     *   proto: any}[]} */
    this.records = [];
    /** @type {Map<string, dataBufferPb.SignalSchema>} */
    this.schemas = new Map();
    this.nextSchemaId = 1n + BigInt(Math.floor(Math.random() * 1000));
    this.deletedPages = new Set();
  }

  _add(kind, proto, { time, source = '', channel = '', typeId = '' }) {
    this.records.push({ kind, time, source, channel, typeId, bytes: proto.serializeBinary().length, proto });
    if (this.records.length > MAX_RECORDS) this.records.shift();
  }

  _time(timestamp) {
    return timestamp && (timestamp.getSeconds() || timestamp.getNanos())
      ? timestampToSec(timestamp)
      : this.robot.clock.now();
  }

  /**
   * @param {dataBufferPb.TextMessage[]} messages
   * @returns {dataBufferPb.RecordTextMessagesResponse.Error[]}
   */
  recordTextMessages(messages) {
    for (const message of messages) {
      this._add('text', message, { time: this._time(message.getTimestamp()), source: message.getSource() });
    }
    return [];
  }

  /**
   * @param {dataBufferPb.OperatorComment[]} comments
   * @returns {dataBufferPb.RecordOperatorCommentsResponse.Error[]}
   */
  recordOperatorComments(comments) {
    for (const comment of comments) {
      logger.info(`Operator comment: ${comment.getMessage()}`);
      this._add('comment', comment, { time: this._time(comment.getTimestamp()), source: 'operator' });
    }
    return [];
  }

  /**
   * @param {dataBufferPb.DataBlob[]} blobs
   * @param {string} source
   * @returns {dataBufferPb.RecordDataBlobsResponse.Error[]}
   */
  recordDataBlobs(blobs, source) {
    const errors = [];
    blobs.forEach((blob, index) => {
      if (!blob.getChannel() && !blob.getTypeId()) {
        errors.push(
          new dataBufferPb.RecordDataBlobsResponse.Error()
            .setType(dataBufferPb.RecordDataBlobsResponse.Error.Type.CLIENT_ERROR)
            .setMessage('The blob has neither a channel nor a type id.')
            .setIndex(index),
        );
        return;
      }
      this._add('blob', blob, {
        time: this._time(blob.getTimestamp()),
        source,
        channel: blob.getChannel(),
        typeId: blob.getTypeId(),
      });
    });
    return errors;
  }

  /**
   * @param {dataBufferPb.Event[]} events
   * @returns {dataBufferPb.RecordEventsResponse.Error[]}
   */
  recordEvents(events) {
    for (const event of events) {
      logger.info(`Event "${event.getType()}" from "${event.getSource()}": ${event.getDescription()}`);
      this._add('event', event, {
        time: this._time(event.getStartTime()),
        source: event.getSource(),
        typeId: event.getType(),
      });
    }
    return [];
  }

  /**
   * @param {dataBufferPb.SignalSchema} schema
   * @returns {string} The id of the schema (an uint64).
   */
  registerSignalSchema(schema) {
    for (const [id, existing] of this.schemas) {
      if (existing.serializeBinary().join() === schema.serializeBinary().join()) return id;
    }
    const id = String(this.nextSchemaId++);
    this.schemas.set(id, schema.clone());
    return id;
  }

  /**
   * @param {dataBufferPb.SignalTick[]} ticks
   * @returns {dataBufferPb.RecordSignalTicksResponse.Error[]}
   */
  recordSignalTicks(ticks) {
    const errors = [];
    const { Error } = dataBufferPb.RecordSignalTicksResponse;
    ticks.forEach((tick, index) => {
      if (!this.schemas.has(String(tick.getSchemaId()))) {
        errors.push(
          new Error()
            .setType(Error.Type.INVALID_SCHEMA_ID)
            .setMessage(`Unknown schema id ${tick.getSchemaId()}.`)
            .setIndex(index),
        );
        return;
      }
      this._add('signal', tick, {
        time: this._time(tick.getTimestamp()),
        source: tick.getSource(),
        channel: String(tick.getSchemaId()),
      });
    });
    return errors;
  }

  // Pages.

  /**
   * @param {object} record
   * @returns {string}
   */
  _pageId(record) {
    const minute = Math.floor(record.time / PAGE_SEC);
    const key = record.kind === 'blob' ? `${record.channel}/${record.typeId}` : record.kind;
    return `${minute}-${key}`;
  }

  /**
   * Groups the records in pages.
   * @param {(record: object) => boolean} filter
   * @returns {Map<string, {id: string, records: object[], start: number, end: number, bytes: number}>}
   */
  _pages(filter = () => true) {
    const pages = new Map();
    for (const record of this.records) {
      if (!filter(record)) continue;
      const id = this._pageId(record);
      let page = pages.get(id);
      if (!page) {
        page = { id, records: [], start: record.time, end: record.time, bytes: 0, kind: record.kind, first: record };
        pages.set(id, page);
      }
      page.records.push(record);
      page.start = Math.min(page.start, record.time);
      page.end = Math.max(page.end, record.time);
      page.bytes += record.bytes;
    }
    return pages;
  }

  _pageInfo(page) {
    const { PageInfo } = dataIndexPb;
    const open = Math.floor(page.end / PAGE_SEC) === Math.floor(this.robot.clock.now() / PAGE_SEC);
    return new PageInfo()
      .setId(page.id)
      .setPath(`data-buffer/${page.id}.bddf`)
      .setSource(page.first.source)
      .setTimeRange(new TimeRange().setStart(secToTimestamp(page.start)).setEnd(secToTimestamp(page.end)))
      .setNumTicks(page.records.length)
      .setTotalBytes(page.bytes)
      .setFormat(PageInfo.PageFormat.FORMAT_BDDF_FILE)
      .setCompression(PageInfo.Compression.COMPRESSION_NONE)
      .setIsOpen(open)
      .setIsDownloaded(false);
  }

  static _rangeProto(pages) {
    if (pages.length === 0) return new TimeRange();
    return new TimeRange()
      .setStart(secToTimestamp(Math.min(...pages.map(page => page.start))))
      .setEnd(secToTimestamp(Math.max(...pages.map(page => page.end))));
  }

  /**
   * GetDataIndex.
   * @param {dataIndexPb.DataQuery} query
   * @returns {dataIndexPb.DataIndex}
   */
  dataIndex(query) {
    const range = rangeOf(query?.getTimeRange());
    const inRange = record => record.time >= range.start && record.time <= range.end;
    const index = new dataIndexPb.DataIndex().setTimeRange(query?.getTimeRange() ?? new TimeRange());
    const kindPages = kind => [...this._pages(record => record.kind === kind && inRange(record)).values()];
    const pagesAndTimestamp = pages =>
      new dataIndexPb.PagesAndTimestamp()
        .setTimeRange(DataBuffer._rangeProto(pages))
        .setPagesList(pages.map(page => this._pageInfo(page)));
    if (query?.getTextMessages()) index.setTextMessages(pagesAndTimestamp(kindPages('text')));
    if (query?.getEvents()) index.setEvents(pagesAndTimestamp(kindPages('event')));
    if (query?.getComments()) index.setComments(pagesAndTimestamp(kindPages('comment')));
    for (const spec of query?.getBlobsList() ?? []) {
      const glob = spec.getChannelGlob() ? globToRegExp(spec.getChannelGlob()) : null;
      const matches = record =>
        record.kind === 'blob' &&
        inRange(record) &&
        (!spec.getChannel() || record.channel === spec.getChannel()) &&
        (!glob || glob.test(record.channel)) &&
        (!spec.getMessageType() || record.typeId === spec.getMessageType()) &&
        (!spec.getSource() || record.source === spec.getSource());
      const pages = [...this._pages(matches).values()];
      index.addBlobs(
        new dataIndexPb.BlobPages()
          .setTimeRange(DataBuffer._rangeProto(pages))
          .setPagesList(
            pages.map(page =>
              new dataIndexPb.BlobPage()
                .setSpec(
                  new dataIndexPb.BlobSpec()
                    .setSource(page.first.source)
                    .setMessageType(page.first.typeId)
                    .setChannel(page.first.channel),
                )
                .setPage(this._pageInfo(page)),
            ),
          ),
      );
    }
    return index;
  }

  /**
   * GetEventsComments.
   * @param {dataIndexPb.EventsCommentsSpec} spec
   * @returns {dataIndexPb.EventsComments}
   */
  eventsComments(spec) {
    const range = rangeOf(spec?.getTimeRange());
    const inRange = record => record.time >= range.start && record.time <= range.end;
    const result = new dataIndexPb.EventsComments().setTimeRange(spec?.getTimeRange() ?? new TimeRange());
    const eventSpecs = spec?.getEventsList() ?? [];
    if (eventSpecs.length > 0) {
      // The type and the level are exact matches (get_events.py of the Python SDK 5.2.0).
      const events = this.records.filter(
        record =>
          record.kind === 'event' &&
          inRange(record) &&
          eventSpecs.some(
            eventSpec =>
              (!eventSpec.getSource() || eventSpec.getSource() === record.source) &&
              (!eventSpec.getType() || eventSpec.getType() === record.typeId) &&
              (!eventSpec.hasLevel() || eventSpec.getLevel().getValue() === record.proto.getLevel()) &&
              (!eventSpec.getLogPreserveHint() || eventSpec.getLogPreserveHint() === record.proto.getLogPreserveHint()),
          ),
      );
      const max = spec.getMaxEvents() || Infinity;
      result.setEventsList(events.slice(0, max).map(record => record.proto)).setEventsLimited(events.length > max);
    }
    if (spec?.getComments()) {
      const comments = this.records.filter(record => record.kind === 'comment' && inRange(record));
      const max = spec.getMaxComments() || Infinity;
      result
        .setOperatorCommentsList(comments.slice(0, max).map(record => record.proto))
        .setOperatorCommentsLimited(comments.length > max);
    }
    return result;
  }

  /**
   * GetDataBufferStatus.
   * @param {boolean} withBlobSpecs
   * @returns {dataIndexPb.DataBufferStatus}
   */
  status(withBlobSpecs) {
    const pages = this._pages();
    const status = new dataIndexPb.DataBufferStatus()
      .setNumDataBufferPages(pages.size)
      .setDataBufferTotalBytes(this.records.reduce((sum, record) => sum + record.bytes, 0))
      .setNumComments(this.records.filter(record => record.kind === 'comment').length)
      .setNumEvents(this.records.filter(record => record.kind === 'event').length);
    if (withBlobSpecs) {
      const specs = new Map();
      for (const record of this.records) {
        if (record.kind !== 'blob') continue;
        specs.set(`${record.source}\n${record.typeId}\n${record.channel}`, record);
      }
      status.setBlobSpecsList(
        [...specs.values()].map(record =>
          new dataIndexPb.BlobSpec().setSource(record.source).setMessageType(record.typeId).setChannel(record.channel),
        ),
      );
    }
    return status;
  }

  /**
   * GetDataPages.
   * @param {?TimeRange} timeRange
   * @returns {dataIndexPb.PageInfo[]}
   */
  pages(timeRange) {
    const range = rangeOf(timeRange);
    return [...this._pages().values()]
      .filter(page => page.end >= range.start && page.start <= range.end)
      .map(page => this._pageInfo(page));
  }

  /**
   * DeleteDataPages.
   * @param {?TimeRange} timeRange
   * @param {string[]} pageIds
   * @returns {{bytesDeleted: number, statuses: dataIndexPb.DeletePageStatus[]}}
   */
  deletePages(timeRange, pageIds) {
    const { DeletePageStatus } = dataIndexPb;
    const pages = this._pages();
    const range = timeRange ? rangeOf(timeRange) : null;
    const targets = new Set(pageIds);
    if (range) {
      for (const page of pages.values()) if (page.end >= range.start && page.start <= range.end) targets.add(page.id);
    }
    let bytesDeleted = 0;
    const statuses = [];
    for (const id of targets) {
      const page = pages.get(id);
      if (!page) {
        statuses.push(new DeletePageStatus().setPageId(id).setStatus(DeletePageStatus.Status.STATUS_NOT_FOUND));
        continue;
      }
      const records = new Set(page.records);
      this.records = this.records.filter(record => !records.has(record));
      bytesDeleted += page.bytes;
      statuses.push(new DeletePageStatus().setPageId(id).setStatus(DeletePageStatus.Status.STATUS_DELETED));
    }
    return { bytesDeleted, statuses };
  }
}

module.exports = { DataBuffer };
