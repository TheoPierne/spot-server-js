'use strict';

const { IREnableDisableRequest, IREnableDisableResponse } = require('../bosdyn/api/ir_enable_disable_pb');
const { IREnableDisableServiceService } = require('../bosdyn/api/ir_enable_disable_service_grpc_pb');
const { LoggerUtil } = require('../loggerUtil');
const { invalidRequest, unary } = require('../util');

const logger = LoggerUtil.getLogger('IR');

/**
 * IREnableDisable: the infrared emitters of the body cameras.
 * @param {IREnableDisableRequest} request
 * @param {{robot: import('../robot').Robot}} context
 * @returns {IREnableDisableResponse}
 */
function iREnableDisable(request, { robot }) {
  const { Request } = IREnableDisableRequest;
  if (![Request.REQUEST_ON, Request.REQUEST_OFF].includes(request.getRequest())) {
    throw invalidRequest('Unknown request.');
  }
  robot.irEmittersEnabled = request.getRequest() === Request.REQUEST_ON;
  logger.info(`Infrared emitters ${robot.irEmittersEnabled ? 'enabled' : 'disabled'}`);
  return new IREnableDisableResponse();
}

module.exports = {
  service: IREnableDisableServiceService,
  func: {
    iREnableDisable: unary('IREnableDisable', IREnableDisableResponse, iREnableDisable),
  },
  directory: [
    {
      name: 'ir-enable-disable-service',
      type: 'bosdyn.api.IREnableDisableService',
      authority: 'ir-enable-disable.spot.robot',
    },
  ],
};
