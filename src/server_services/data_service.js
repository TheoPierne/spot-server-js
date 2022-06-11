'use strict';

const data_buffer_pb = require('../bosdyn/api/data_buffer_pb');
const data_service_grpc_pb = require('../bosdyn/api/data_service_grpc_pb');
const { LoggerUtil } = require('../loggerUtil');

const { populate_response_header } = require('../util');

const logger = LoggerUtil.getLogger('DATA_SERVICE');

function getDataIndex(call, callback){
  logger.info('New request /getDataIndex !');
  let reply = new data_buffer_pb.RecordTextMessagesResponse();
  populate_response_header(reply, call.request);

  

  callback(null, reply);
}

function getEventsComments(call, callback){
  logger.info('New request /getEventsComments !');
  let reply = new data_buffer_pb.RecordTextMessagesResponse();
  populate_response_header(reply, call.request);

  

  callback(null, reply);
}

function getDataBufferStatus(call, callback){
  logger.info('New request /getDataBufferStatus !');
  let reply = new data_buffer_pb.RecordTextMessagesResponse();
  populate_response_header(reply, call.request);

  

  callback(null, reply);
}

function getDataPages(call, callback){
  logger.info('New request /getDataPages !');
  let reply = new data_buffer_pb.RecordTextMessagesResponse();
  populate_response_header(reply, call.request);

  

  callback(null, reply);
}

function deleteDataPages(call, callback){
  logger.info('New request /deleteDataPages !');
  let reply = new data_buffer_pb.RecordTextMessagesResponse();
  populate_response_header(reply, call.request);

  

  callback(null, reply);
}

module.exports = {
  service: data_service_grpc_pb.DataServiceService,
  func: {
    getDataIndex,
    getEventsComments,
    getDataBufferStatus,
    deleteDataPages,
  },
};