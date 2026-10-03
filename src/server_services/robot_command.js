'use strict';

const leasePb = require('../bosdyn/api/lease_pb');
const robotCommandPb = require('../bosdyn/api/robot_command_pb');
const { RobotCommandServiceService } = require('../bosdyn/api/robot_command_service_grpc_pb');
const { LeaseManager, leaseFromProto } = require('../sim/lease');
const { invalidRequest, unary } = require('../util');

/**
 * RobotCommand.
 * @param {robotCommandPb.RobotCommandRequest} request
 * @param {{robot: import('../robot').Robot, clientName: string}} context
 * @returns {robotCommandPb.RobotCommandResponse}
 */
function robotCommand(request, { robot, clientName }) {
  const result = robot.commands.submit(request, clientName);
  const response = new robotCommandPb.RobotCommandResponse()
    .setStatus(result.status)
    .setMessage(result.message)
    .setRobotCommandId(result.id);
  if (result.leaseUseResult) response.setLeaseUseResult(result.leaseUseResult);
  return response;
}

/**
 * RobotCommandFeedback.
 * @param {robotCommandPb.RobotCommandFeedbackRequest} request
 * @param {{robot: import('../robot').Robot}} context
 * @returns {robotCommandPb.RobotCommandFeedbackResponse}
 */
function robotCommandFeedback(request, { robot }) {
  const feedback = robot.commands.feedback(request.getRobotCommandId());
  if (!feedback) throw invalidRequest(`Unknown robot command id ${request.getRobotCommandId()}.`);
  return new robotCommandPb.RobotCommandFeedbackResponse().setFeedback(feedback);
}

/**
 * ClearBehaviorFault: needs the body lease.
 * @param {robotCommandPb.ClearBehaviorFaultRequest} request
 * @param {{robot: import('../robot').Robot}} context
 * @returns {robotCommandPb.ClearBehaviorFaultResponse}
 */
function clearBehaviorFault(request, { robot }) {
  const lease = robot.leases.use(leaseFromProto(request.getLease()), ['body']);
  const response = new robotCommandPb.ClearBehaviorFaultResponse().setLeaseUseResult(LeaseManager.resultToProto(lease));
  if (lease.status !== leasePb.LeaseUseResult.Status.STATUS_OK) return response;
  const result = robot.commands.clearBehaviorFault(request.getBehaviorFaultId());
  response.setStatus(result.status).setBlockingSystemFaultsList(result.blockingSystemFaults);
  if (result.fault) response.setBehaviorFault(result.fault);
  return response;
}

module.exports = {
  service: RobotCommandServiceService,
  func: {
    robotCommand: unary('RobotCommand', robotCommandPb.RobotCommandResponse, robotCommand),
    robotCommandFeedback: unary(
      'RobotCommandFeedback',
      robotCommandPb.RobotCommandFeedbackResponse,
      robotCommandFeedback,
    ),
    clearBehaviorFault: unary('ClearBehaviorFault', robotCommandPb.ClearBehaviorFaultResponse, clearBehaviorFault),
  },
  directory: [{ name: 'robot-command', type: 'bosdyn.api.RobotCommandService', authority: 'command.spot.robot' }],
};
