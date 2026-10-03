'use strict';

const autoReturnPb = require('../bosdyn/api/auto_return/auto_return_pb');
const { AutoReturnServiceService } = require('../bosdyn/api/auto_return/auto_return_service_grpc_pb');
const { unary } = require('../util');

/**
 * Configure.
 * @param {autoReturnPb.ConfigureRequest} request
 * @param {{robot: import('../robot').Robot}} context
 * @returns {autoReturnPb.ConfigureResponse}
 */
function configure(request, { robot }) {
  const result = robot.autoReturn.configure(request);
  const response = new autoReturnPb.ConfigureResponse().setStatus(result.status);
  if (result.invalidParams) response.setInvalidParams(result.invalidParams);
  return response;
}

/**
 * GetConfiguration.
 * @param {autoReturnPb.GetConfigurationRequest} request
 * @param {{robot: import('../robot').Robot}} context
 * @returns {autoReturnPb.GetConfigurationResponse}
 */
function getConfiguration(request, { robot }) {
  const response = new autoReturnPb.GetConfigurationResponse().setEnabled(robot.autoReturn.enabled);
  if (robot.autoReturn.request) response.setRequest(robot.autoReturn.request);
  return response;
}

/**
 * Start.
 * @param {autoReturnPb.StartRequest} request
 * @param {{robot: import('../robot').Robot}} context
 * @returns {autoReturnPb.StartResponse}
 */
function start(request, { robot }) {
  const result = robot.autoReturn.start(request);
  const response = new autoReturnPb.StartResponse().setStatus(result.status);
  if (result.invalidParams) response.setInvalidParams(result.invalidParams);
  return response;
}

module.exports = {
  service: AutoReturnServiceService,
  func: {
    configure: unary('Configure', autoReturnPb.ConfigureResponse, configure),
    getConfiguration: unary('GetConfiguration', autoReturnPb.GetConfigurationResponse, getConfiguration),
    start: unary('Start', autoReturnPb.StartResponse, start),
  },
  directory: [
    { name: 'auto-return', type: 'bosdyn.api.auto_return.AutoReturnService', authority: 'auto-return.spot.robot' },
  ],
};
