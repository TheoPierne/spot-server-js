'use strict';

const dataBufferPb = require('../bosdyn/api/data_buffer_pb');
const { DataBufferServiceService } = require('../bosdyn/api/data_buffer_service_grpc_pb');
const { unary } = require('../util');

module.exports = {
  service: DataBufferServiceService,
  func: {
    recordTextMessages: unary('RecordTextMessages', dataBufferPb.RecordTextMessagesResponse, (request, { robot }) =>
      new dataBufferPb.RecordTextMessagesResponse().setErrorsList(
        robot.dataBuffer.recordTextMessages(request.getTextMessagesList()),
      ),
    ),
    recordOperatorComments: unary(
      'RecordOperatorComments',
      dataBufferPb.RecordOperatorCommentsResponse,
      (request, { robot }) =>
        new dataBufferPb.RecordOperatorCommentsResponse().setErrorsList(
          robot.dataBuffer.recordOperatorComments(request.getOperatorCommentsList()),
        ),
    ),
    recordDataBlobs: unary('RecordDataBlobs', dataBufferPb.RecordDataBlobsResponse, (request, { robot, clientName }) =>
      new dataBufferPb.RecordDataBlobsResponse().setErrorsList(
        robot.dataBuffer.recordDataBlobs(request.getBlobDataList(), clientName),
      ),
    ),
    recordEvents: unary('RecordEvents', dataBufferPb.RecordEventsResponse, (request, { robot }) =>
      new dataBufferPb.RecordEventsResponse().setErrorsList(robot.dataBuffer.recordEvents(request.getEventsList())),
    ),
    registerSignalSchema: unary(
      'RegisterSignalSchema',
      dataBufferPb.RegisterSignalSchemaResponse,
      (request, { robot }) =>
        new dataBufferPb.RegisterSignalSchemaResponse().setSchemaId(
          robot.dataBuffer.registerSignalSchema(request.getSchema() ?? new dataBufferPb.SignalSchema()),
        ),
    ),
    recordSignalTicks: unary('RecordSignalTicks', dataBufferPb.RecordSignalTicksResponse, (request, { robot }) =>
      new dataBufferPb.RecordSignalTicksResponse().setErrorsList(
        robot.dataBuffer.recordSignalTicks(request.getTickDataList()),
      ),
    ),
  },
  directory: [{ name: 'data-buffer', type: 'bosdyn.api.DataBufferService', authority: 'buffer.spot.robot' }],
};
