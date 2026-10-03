'use strict';

const keepalivePb = require('../bosdyn/api/keepalive/keepalive_pb');
const { KeepaliveServiceService } = require('../bosdyn/api/keepalive/keepalive_service_grpc_pb');
const { secToTimestamp } = require('../sim/clock');
const { KeepaliveManager } = require('../sim/keepalive');
const { invalidRequest, unary } = require('../util');

/**
 * ModifyPolicy.
 * @param {keepalivePb.ModifyPolicyRequest} request
 * @param {{robot: import('../robot').Robot, clientName: string}} context
 * @returns {keepalivePb.ModifyPolicyResponse}
 */
function modifyPolicy(request, { robot, clientName }) {
  const result = robot.keepalive.modify(
    request.hasToAdd() ? request.getToAdd() : null,
    request.getPolicyIdsToRemoveList(),
    clientName,
  );
  if (result.error) throw invalidRequest(result.error);
  const response = new keepalivePb.ModifyPolicyResponse()
    .setStatus(result.status)
    .setRemovedPoliciesList(result.removed.map(policy => KeepaliveManager.livePolicyToProto(policy)));
  if (result.added) response.setAddedPolicy(KeepaliveManager.livePolicyToProto(result.added));
  return response;
}

/**
 * CheckIn.
 * @param {keepalivePb.CheckInRequest} request
 * @param {{robot: import('../robot').Robot, clientName: string}} context
 * @returns {keepalivePb.CheckInResponse}
 */
function checkIn(request, { robot, clientName }) {
  const { Status } = keepalivePb.CheckInResponse;
  const time = robot.keepalive.checkIn(request.getPolicyId(), clientName);
  const response = new keepalivePb.CheckInResponse();
  if (time === null) return response.setStatus(Status.STATUS_INVALID_POLICY_ID);
  return response.setStatus(Status.STATUS_OK).setLastCheckin(secToTimestamp(time));
}

module.exports = {
  service: KeepaliveServiceService,
  func: {
    modifyPolicy: unary('ModifyPolicy', keepalivePb.ModifyPolicyResponse, modifyPolicy),
    checkIn: unary('CheckIn', keepalivePb.CheckInResponse, checkIn),
    getStatus: unary('GetStatus', keepalivePb.GetStatusResponse, (request, { robot }) =>
      robot.keepalive.statusToProto(),
    ),
  },
  directory: [{ name: 'keepalive', type: 'bosdyn.api.keepalive.KeepaliveService', authority: 'keepalive.spot.robot' }],
};
