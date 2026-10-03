'use strict';

const timeSyncPb = require('../bosdyn/api/time_sync_pb');
const { TimeSyncServiceService } = require('../bosdyn/api/time_sync_service_grpc_pb');
const { nsecToDuration, secToTimestamp } = require('../sim/clock');
const { unary } = require('../util');

/**
 * @param {?{rttNsec: bigint, skewNsec: bigint}} estimate
 * @returns {timeSyncPb.TimeSyncEstimate}
 */
function estimateToProto(estimate) {
  const proto = new timeSyncPb.TimeSyncEstimate();
  if (estimate) {
    proto.setRoundTripTime(nsecToDuration(estimate.rttNsec)).setClockSkew(nsecToDuration(estimate.skewNsec));
  }
  return proto;
}

/**
 * TimeSyncUpdate: the robot assigns an identifier to the clock of the client, and estimates the skew from the round
 * trips that the client reports (STATUS_MORE_SAMPLES_NEEDED until a few consistent measurements).
 * @param {timeSyncPb.TimeSyncUpdateRequest} request
 * @param {{robot: import('../robot').Robot, clientName: string}} context
 * @returns {timeSyncPb.TimeSyncUpdateResponse}
 */
function timeSyncUpdate(request, { robot, clientName }) {
  const result = robot.timeSync.update(
    request.getClockIdentifier(),
    request.hasPreviousRoundTrip() ? request.getPreviousRoundTrip() : null,
    clientName,
  );
  const { Status } = timeSyncPb.TimeSyncState;
  const state = new timeSyncPb.TimeSyncState()
    .setStatus(result.synced ? Status.STATUS_OK : Status.STATUS_MORE_SAMPLES_NEEDED)
    .setMeasurementTime(secToTimestamp(robot.clock.now()));
  if (result.synced) state.setBestEstimate(estimateToProto(result.best));
  return new timeSyncPb.TimeSyncUpdateResponse()
    .setPreviousEstimate(estimateToProto(result.previous))
    .setState(state)
    .setClockIdentifier(result.clockIdentifier);
}

module.exports = {
  service: TimeSyncServiceService,
  func: {
    timeSyncUpdate: unary('TimeSyncUpdate', timeSyncPb.TimeSyncUpdateResponse, timeSyncUpdate),
  },
  directory: [{ name: 'time-sync', type: 'bosdyn.api.TimeSyncService', authority: 'api.spot.robot' }],
};
