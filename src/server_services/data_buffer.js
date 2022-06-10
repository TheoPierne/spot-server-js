'use strict';

const data_buffer_pb = require('../bosdyn/api/data_buffer_pb');
const data_buffer_service_grpc_pb = require('../bosdyn/api/data_buffer_service_grpc_pb');
const { LoggerUtil } = require('../loggerUtil');

const { populate_response_header } = require('../util');

const logger = LoggerUtil.getLogger('DATA_BUFFER');

let logs = {
  recordTextMessages: [],
  recordOperatorComments: [],
  recordDataBlobs: [],
  recordEvents: []
};

function recordTextMessages(call, callback){
  logger.info('New request /recordTextMessages !');
  let reply = new data_buffer_pb.RecordTextMessagesResponse();
  populate_response_header(reply, call.request);

  logs.recordTextMessages = logs.recordTextMessages.concat(call.request.getTextMessagesList());

  reply.setErrorsList([]);

  console.log(logs.recordTextMessages.map(e => e.toObject()))

  callback(null, reply);
}

function recordOperatorComments(call, callback){
  logger.info('New request /recordOperatorComments !');
  let reply = new data_buffer_pb.RecordOperatorCommentsResponse();
  populate_response_header(reply, call.request);

  logs.recordOperatorComments = logs.recordOperatorComments.concat(call.request.getOperatorCommentsList());

  reply.setErrorsList([]);

  console.log(logs.recordOperatorComments.map(e => e.toObject()))

  callback(null, reply);
}

function recordDataBlobs(call, callback){
  logger.info('New request /recordDataBlobs !');
  let reply = new data_buffer_pb.RecordDataBlobsResponse();
  populate_response_header(reply, call.request);

  logs.recordDataBlobs = logs.recordDataBlobs.concat(call.request.getBlobDataList());

  reply.setErrorsList([]);

  console.log(logs.recordDataBlobs.map(e => e.toObject()))

  callback(null, reply);
}

function recordEvents(call, callback){
  logger.info('New request /recordEvents !');
  let reply = new data_buffer_pb.RecordEventsResponse();
  populate_response_header(reply, call.request);

  logs.recordEvents = logs.recordEvents.concat(call.request.getEventsList());

  reply.setErrorsList([]);

  console.log(logs.recordEvents.map(e => e.toObject()))

  callback(null, reply);
}


module.exports = {
  service: data_buffer_service_grpc_pb.DataBufferServiceService,
  func: {
    recordTextMessages,
    recordOperatorComments,
    recordDataBlobs,
    recordEvents,
  },
};
