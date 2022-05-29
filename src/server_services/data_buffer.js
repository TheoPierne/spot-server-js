'use strict';

const data_buffer_pb = require('../bosdyn/api/data_buffer_pb');
const data_buffer_service_grpc_pb = require('../bosdyn/api/data_buffer_service_grpc_pb');
const { LoggerUtil } = require('../loggerUtil');

const { populate_response_header } = require('../util');

const logger = LoggerUtil.getLogger('DATA_BUFFER');

let operatorComments = [];

function recordTextMessages(call, callback){

}

function recordOperatorComments(call, callback){
	logger.info('New request /recordOperatorComments !');
	let reply = new data_buffer_pb.RecordOperatorCommentsResponse();
	populate_response_header(reply, call.request);

  operatorComments = operatorComments.concat(call.request.getOperatorCommentsList());

  reply.setErrorsList([]);

  callback(null, reply);
}


module.exports = {
	service: data_buffer_service_grpc_pb.DataBufferServiceService,
	func: {
		recordTextMessages,
		recordOperatorComments,
	},
};
