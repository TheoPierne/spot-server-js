'use strict';

const leasePb = require('../bosdyn/api/lease_pb');
const { LicenseInfo } = require('../bosdyn/api/license_pb');
const powerPb = require('../bosdyn/api/power_pb');
const { PowerServiceService } = require('../bosdyn/api/power_service_grpc_pb');
const { durationToSec, secToTimestamp } = require('../sim/clock');
const { LeaseManager, leaseFromProto } = require('../sim/lease');
const { invalidRequest, unary } = require('../util');

/**
 * PowerCommand: needs the body lease.
 * @param {powerPb.PowerCommandRequest} request
 * @param {{robot: import('../robot').Robot}} context
 * @returns {powerPb.PowerCommandResponse}
 */
function powerCommand(request, { robot }) {
  const lease = robot.leases.use(leaseFromProto(request.getLease()), ['body']);
  const response = new powerPb.PowerCommandResponse()
    .setLeaseUseResult(LeaseManager.resultToProto(lease))
    .setLicenseStatus(robot.license.isValid() ? LicenseInfo.Status.STATUS_VALID : robot.license.status);
  if (lease.status !== leasePb.LeaseUseResult.Status.STATUS_OK) {
    return response.setStatus(powerPb.PowerCommandStatus.STATUS_UNKNOWN);
  }
  const result = robot.power.command(request.getRequest());
  if (result.invalid) throw invalidRequest(`Unknown power request ${request.getRequest()}.`);
  return response.setStatus(result.status).setPowerCommandId(result.id).setBlockingFaultsList(result.blockingFaults);
}

/**
 * PowerCommandFeedback.
 * @param {powerPb.PowerCommandFeedbackRequest} request
 * @param {{robot: import('../robot').Robot}} context
 * @returns {powerPb.PowerCommandFeedbackResponse}
 */
function powerCommandFeedback(request, { robot }) {
  const feedback = robot.power.feedback(request.getPowerCommandId());
  if (!feedback) throw invalidRequest(`Unknown power command id ${request.getPowerCommandId()}.`);
  return new powerPb.PowerCommandFeedbackResponse()
    .setStatus(feedback.status)
    .setBlockingFaultsList(feedback.blockingFaults);
}

/**
 * FanPowerCommand: needs the body lease.
 * @param {powerPb.FanPowerCommandRequest} request
 * @param {{robot: import('../robot').Robot}} context
 * @returns {powerPb.FanPowerCommandResponse}
 */
function fanPowerCommand(request, { robot }) {
  const lease = robot.leases.use(leaseFromProto(request.getLease()), ['body']);
  const response = new powerPb.FanPowerCommandResponse().setLeaseUseResult(LeaseManager.resultToProto(lease));
  if (lease.status !== leasePb.LeaseUseResult.Status.STATUS_OK) return response;
  const result = robot.power.fanCommand(request.getPercentPower(), durationToSec(request.getDuration()) ?? 0);
  return response.setStatus(result.status).setCommandId(result.id).setDesiredEndTime(secToTimestamp(result.endTime));
}

/**
 * FanPowerCommandFeedback.
 * @param {powerPb.FanPowerCommandFeedbackRequest} request
 * @param {{robot: import('../robot').Robot}} context
 * @returns {powerPb.FanPowerCommandFeedbackResponse}
 */
function fanPowerCommandFeedback(request, { robot }) {
  const feedback = robot.power.fanFeedback(request.getCommandId());
  if (!feedback) throw invalidRequest(`Unknown fan command id ${request.getCommandId()}.`);
  return feedback;
}

/**
 * GetFanInformation.
 * @param {powerPb.GetFanInformationRequest} request
 * @param {{robot: import('../robot').Robot}} context
 * @returns {powerPb.GetFanInformationResponse}
 */
function getFanInformation(request, { robot }) {
  const response = new powerPb.GetFanInformationResponse();
  const map = response.getFanInformationMap();
  for (const [name, frequency] of Object.entries(robot.power.fanFrequencies())) {
    map.set(name, new powerPb.FanInformation().setFrequency(frequency));
  }
  return response;
}

/**
 * ResetSafetyStop: the simulated robot has no redundant safety stop.
 * @param {powerPb.ResetSafetyStopRequest} request
 * @param {{robot: import('../robot').Robot}} context
 * @returns {powerPb.ResetSafetyStopResponse}
 */
function resetSafetyStop(request, { robot }) {
  const lease = robot.leases.use(leaseFromProto(request.getLease()), ['body']);
  const response = new powerPb.ResetSafetyStopResponse().setLeaseUseResult(LeaseManager.resultToProto(lease));
  if (lease.status !== leasePb.LeaseUseResult.Status.STATUS_OK) return response;
  return response.setStatus(powerPb.ResetSafetyStopResponse.Status.STATUS_INCOMPATIBLE_HARDWARE_ERROR);
}

module.exports = {
  service: PowerServiceService,
  func: {
    powerCommand: unary('PowerCommand', powerPb.PowerCommandResponse, powerCommand),
    powerCommandFeedback: unary('PowerCommandFeedback', powerPb.PowerCommandFeedbackResponse, powerCommandFeedback),
    fanPowerCommand: unary('FanPowerCommand', powerPb.FanPowerCommandResponse, fanPowerCommand),
    fanPowerCommandFeedback: unary(
      'FanPowerCommandFeedback',
      powerPb.FanPowerCommandFeedbackResponse,
      fanPowerCommandFeedback,
    ),
    getFanInformation: unary('GetFanInformation', powerPb.GetFanInformationResponse, getFanInformation),
    resetSafetyStop: unary('ResetSafetyStop', powerPb.ResetSafetyStopResponse, resetSafetyStop),
  },
  directory: [{ name: 'power', type: 'bosdyn.api.PowerService', authority: 'power.spot.robot' }],
};
