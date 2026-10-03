'use strict';

const dockingPb = require('../bosdyn/api/docking/docking_pb');
const { DockingServiceService } = require('../bosdyn/api/docking/docking_service_grpc_pb');
const leasePb = require('../bosdyn/api/lease_pb');
const { timestampToSec } = require('../sim/clock');
const { LeaseManager, leaseFromProto } = require('../sim/lease');
const { invalidRequest, unary } = require('../util');

/**
 * DockingCommand: needs the body lease.
 * @param {dockingPb.DockingCommandRequest} request
 * @param {{robot: import('../robot').Robot}} context
 * @returns {dockingPb.DockingCommandResponse}
 */
function dockingCommand(request, { robot }) {
  const lease = robot.leases.use(leaseFromProto(request.getLease()), ['body']);
  const response = new dockingPb.DockingCommandResponse().setLeaseUseResult(LeaseManager.resultToProto(lease));
  if (lease.status !== leasePb.LeaseUseResult.Status.STATUS_OK) {
    return response.setStatus(dockingPb.DockingCommandResponse.Status.STATUS_ERROR_LEASE);
  }
  if (!request.hasEndTime()) throw invalidRequest('The end time of the docking command is required.');
  const synced = robot.timeSync.isSynced(request.getClockIdentifier());
  const result = robot.docking.command(request, timestampToSec(request.getEndTime()), synced);
  return response.setStatus(result.status).setDockingCommandId(result.id);
}

/**
 * DockingCommandFeedback.
 * @param {dockingPb.DockingCommandFeedbackRequest} request
 * @param {{robot: import('../robot').Robot}} context
 * @returns {dockingPb.DockingCommandFeedbackResponse}
 */
function dockingCommandFeedback(request, { robot }) {
  const update = request.getUpdateDockingParams();
  const newEndTime = update?.hasEndTime() ? timestampToSec(update.getEndTime()) : null;
  const status = robot.docking.feedback(request.getDockingCommandId(), newEndTime);
  if (status === null) throw invalidRequest(`Unknown docking command id ${request.getDockingCommandId()}.`);
  return new dockingPb.DockingCommandFeedbackResponse().setStatus(status);
}

module.exports = {
  service: DockingServiceService,
  func: {
    dockingCommand: unary('DockingCommand', dockingPb.DockingCommandResponse, dockingCommand),
    dockingCommandFeedback: unary(
      'DockingCommandFeedback',
      dockingPb.DockingCommandFeedbackResponse,
      dockingCommandFeedback,
    ),
    getDockingConfig: unary('GetDockingConfig', dockingPb.GetDockingConfigResponse, (request, { robot }) =>
      new dockingPb.GetDockingConfigResponse().setDockConfigsList(robot.docking.configToProto()),
    ),
    getDockingState: unary('GetDockingState', dockingPb.GetDockingStateResponse, (request, { robot }) =>
      new dockingPb.GetDockingStateResponse().setDockState(robot.docking.stateToProto()),
    ),
  },
  directory: [{ name: 'docking', type: 'bosdyn.api.docking.DockingService', authority: 'api.spot.robot' }],
};
