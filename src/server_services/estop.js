'use strict';

const estopPb = require('../bosdyn/api/estop_pb');
const { EstopServiceService } = require('../bosdyn/api/estop_service_grpc_pb');
const { EstopSystem } = require('../sim/estop');
const { invalidRequest, unary } = require('../util');

/**
 * RegisterEstopEndpoint.
 * @param {estopPb.RegisterEstopEndpointRequest} request
 * @param {{robot: import('../robot').Robot}} context
 * @returns {estopPb.RegisterEstopEndpointResponse}
 */
function registerEstopEndpoint(request, { robot }) {
  const result = robot.estop.register(
    request.getTargetConfigId(),
    request.getTargetEndpoint() ?? new estopPb.EstopEndpoint(),
    request.getNewEndpoint() ?? new estopPb.EstopEndpoint(),
  );
  const response = new estopPb.RegisterEstopEndpointResponse().setRequest(request).setStatus(result.status);
  if (result.endpoint) response.setNewEndpoint(EstopSystem.endpointToProto(result.endpoint));
  return response;
}

/**
 * EstopCheckIn.
 * @param {estopPb.EstopCheckInRequest} request
 * @param {{robot: import('../robot').Robot}} context
 * @returns {estopPb.EstopCheckInResponse}
 */
function estopCheckIn(request, { robot }) {
  const result = robot.estop.checkIn(
    request.getEndpoint(),
    request.getChallenge(),
    request.getResponse(),
    request.getStopLevel(),
  );
  return new estopPb.EstopCheckInResponse().setRequest(request).setChallenge(result.challenge).setStatus(result.status);
}

/**
 * SetEstopConfig.
 * @param {estopPb.SetEstopConfigRequest} request
 * @param {{robot: import('../robot').Robot}} context
 * @returns {estopPb.SetEstopConfigResponse}
 */
function setEstopConfig(request, { robot }) {
  const result = robot.estop.setConfig(request.getConfig() ?? new estopPb.EstopConfig(), request.getTargetConfigId());
  if (result.error) throw invalidRequest(result.error);
  return new estopPb.SetEstopConfigResponse()
    .setRequest(request)
    .setStatus(result.status)
    .setActiveConfig(robot.estop.configToProto());
}

module.exports = {
  service: EstopServiceService,
  func: {
    registerEstopEndpoint: unary('RegisterEstopEndpoint', estopPb.RegisterEstopEndpointResponse, registerEstopEndpoint),
    deregisterEstopEndpoint: unary(
      'DeregisterEstopEndpoint',
      estopPb.DeregisterEstopEndpointResponse,
      (request, { robot }) =>
        new estopPb.DeregisterEstopEndpointResponse()
          .setRequest(request)
          .setStatus(
            robot.estop.deregister(
              request.getTargetConfigId(),
              request.getTargetEndpoint() ?? new estopPb.EstopEndpoint(),
            ),
          ),
    ),
    estopCheckIn: unary('EstopCheckIn', estopPb.EstopCheckInResponse, estopCheckIn),
    getEstopConfig: unary('GetEstopConfig', estopPb.GetEstopConfigResponse, (request, { robot }) =>
      new estopPb.GetEstopConfigResponse().setRequest(request).setActiveConfig(robot.estop.configToProto()),
    ),
    setEstopConfig: unary('SetEstopConfig', estopPb.SetEstopConfigResponse, setEstopConfig),
    getEstopSystemStatus: unary('GetEstopSystemStatus', estopPb.GetEstopSystemStatusResponse, (request, { robot }) =>
      new estopPb.GetEstopSystemStatusResponse().setStatus(robot.estop.systemStatusToProto()),
    ),
  },
  directory: [{ name: 'estop', type: 'bosdyn.api.EstopService', authority: 'estop.spot.robot' }],
};
