'use strict';

const leasePb = require('../bosdyn/api/lease_pb');
const choreographyPb = require('../bosdyn/api/spot/choreography_sequence_pb');
const { ChoreographyServiceService } = require('../bosdyn/api/spot/choreography_service_grpc_pb');
const { Choreography } = require('../sim/choreography');
const { LeaseManager, leaseFromProto } = require('../sim/lease');
const { invalidRequest, unary } = require('../util');

const { SavedState } = choreographyPb.SequenceInfo;

/**
 * UploadChoreography.
 * @param {choreographyPb.UploadChoreographyRequest} request
 * @param {{robot: import('../robot').Robot}} context
 * @returns {choreographyPb.UploadChoreographyResponse}
 */
function uploadChoreography(request, { robot }) {
  const result = robot.choreography.upload(request.getChoreographySequence(), request.getNonStrictParsing());
  if (result.error) throw invalidRequest(result.error);
  return new choreographyPb.UploadChoreographyResponse().setWarningsList(result.warnings);
}

/**
 * ExecuteChoreography: needs the body lease.
 * @param {choreographyPb.ExecuteChoreographyRequest} request
 * @param {{robot: import('../robot').Robot}} context
 * @returns {choreographyPb.ExecuteChoreographyResponse}
 */
function executeChoreography(request, { robot }) {
  const { Status } = choreographyPb.ExecuteChoreographyResponse;
  const lease = robot.leases.use(leaseFromProto(request.getLease()), ['body']);
  const response = new choreographyPb.ExecuteChoreographyResponse().setLeaseUseResult(
    LeaseManager.resultToProto(lease),
  );
  if (lease.status !== leasePb.LeaseUseResult.Status.STATUS_OK) return response.setStatus(Status.STATUS_LEASE_ERROR);
  const result = robot.choreography.execute(
    request.getChoreographySequenceName(),
    Choreography.timeOf(request.getStartTime()),
    request.getChoreographyStartingSlice(),
  );
  return response.setStatus(result.status).setExecutionId(result.id);
}

/**
 * GetChoreographySequence.
 * @param {choreographyPb.GetChoreographySequenceRequest} request
 * @param {{robot: import('../robot').Robot}} context
 * @returns {choreographyPb.GetChoreographySequenceResponse}
 */
function getChoreographySequence(request, { robot }) {
  const { Status } = choreographyPb.GetChoreographySequenceResponse;
  const entry = robot.choreography.sequences.get(request.getSequenceName());
  const response = new choreographyPb.GetChoreographySequenceResponse();
  if (!entry) return response.setStatus(Status.STATUS_UNKNOWN_SEQUENCE);
  return response.setStatus(Status.STATUS_OK).setChoreographySequence(entry.sequence);
}

/**
 * DeleteSequence.
 * @param {choreographyPb.DeleteSequenceRequest} request
 * @param {{robot: import('../robot').Robot}} context
 * @returns {choreographyPb.DeleteSequenceResponse}
 */
function deleteSequence(request, { robot }) {
  const { Status } = choreographyPb.DeleteSequenceResponse;
  const { sequences } = robot.choreography;
  const entry = sequences.get(request.getSequenceName());
  let status = Status.STATUS_OK;
  if (!entry) status = Status.STATUS_UNKNOWN_SEQUENCE;
  else if (entry.savedState === SavedState.SAVED_STATE_PERMANENT) status = Status.STATUS_PERMANENT_SEQUENCE;
  else if (entry.savedState === SavedState.SAVED_STATE_TEMPORARY) status = Status.STATUS_ALREADY_TEMPORARY;
  else entry.savedState = SavedState.SAVED_STATE_TEMPORARY;
  return new choreographyPb.DeleteSequenceResponse().setStatus(status);
}

/**
 * SaveSequence.
 * @param {choreographyPb.SaveSequenceRequest} request
 * @param {{robot: import('../robot').Robot}} context
 * @returns {choreographyPb.SaveSequenceResponse}
 */
function saveSequence(request, { robot }) {
  const { Status } = choreographyPb.SaveSequenceResponse;
  const entry = robot.choreography.sequences.get(request.getSequenceName());
  if (!entry) return new choreographyPb.SaveSequenceResponse().setStatus(Status.STATUS_UNKNOWN_SEQUENCE);
  if (entry.savedState === SavedState.SAVED_STATE_PERMANENT) {
    return new choreographyPb.SaveSequenceResponse().setStatus(Status.STATUS_PERMANENT_SEQUENCE);
  }
  entry.savedState = SavedState.SAVED_STATE_RETAINED;
  return new choreographyPb.SaveSequenceResponse().setStatus(Status.STATUS_OK);
}

/**
 * ClearAllSequenceFiles: the temporary sequences stay until the reboot, the retained ones become temporary.
 * @param {choreographyPb.ClearAllSequenceFilesRequest} request
 * @param {{robot: import('../robot').Robot}} context
 * @returns {choreographyPb.ClearAllSequenceFilesResponse}
 */
function clearAllSequenceFiles(request, { robot }) {
  for (const entry of robot.choreography.sequences.values()) {
    if (entry.savedState === SavedState.SAVED_STATE_RETAINED) entry.savedState = SavedState.SAVED_STATE_TEMPORARY;
  }
  return new choreographyPb.ClearAllSequenceFilesResponse().setStatus(
    choreographyPb.ClearAllSequenceFilesResponse.Status.STATUS_OK,
  );
}

/**
 * UploadAnimatedMove: the animation becomes a known move.
 * @param {choreographyPb.UploadAnimatedMoveRequest} request
 * @param {{robot: import('../robot').Robot}} context
 * @returns {choreographyPb.UploadAnimatedMoveResponse}
 */
function uploadAnimatedMove(request, { robot }) {
  const { Status } = choreographyPb.UploadAnimatedMoveResponse;
  const name = request.getAnimatedMove()?.getName();
  if (!name) {
    return new choreographyPb.UploadAnimatedMoveResponse().setStatus(Status.STATUS_ANIMATION_VALIDATION_FAILED);
  }
  robot.choreography.animations.add(name);
  return new choreographyPb.UploadAnimatedMoveResponse().setStatus(Status.STATUS_OK);
}

let nextRecordingSession = 1;

module.exports = {
  service: ChoreographyServiceService,
  func: {
    listAllMoves: unary('ListAllMoves', choreographyPb.ListAllMovesResponse, (request, { robot }) =>
      new choreographyPb.ListAllMovesResponse().setMovesList(robot.choreography.movesToProto()),
    ),
    listAllSequences: unary('ListAllSequences', choreographyPb.ListAllSequencesResponse, (request, { robot }) => {
      const infos = robot.choreography.sequencesToProto();
      return new choreographyPb.ListAllSequencesResponse()
        .setKnownSequencesList(infos.map(info => info.getName()))
        .setSequenceInfoList(infos);
    }),
    getChoreographySequence: unary(
      'GetChoreographySequence',
      choreographyPb.GetChoreographySequenceResponse,
      getChoreographySequence,
    ),
    deleteSequence: unary('DeleteSequence', choreographyPb.DeleteSequenceResponse, deleteSequence),
    saveSequence: unary('SaveSequence', choreographyPb.SaveSequenceResponse, saveSequence),
    clearAllSequenceFiles: unary(
      'ClearAllSequenceFiles',
      choreographyPb.ClearAllSequenceFilesResponse,
      clearAllSequenceFiles,
    ),
    uploadChoreography: unary('UploadChoreography', choreographyPb.UploadChoreographyResponse, uploadChoreography),
    uploadAnimatedMove: unary('UploadAnimatedMove', choreographyPb.UploadAnimatedMoveResponse, uploadAnimatedMove),
    executeChoreography: unary('ExecuteChoreography', choreographyPb.ExecuteChoreographyResponse, executeChoreography),
    startRecordingState: unary('StartRecordingState', choreographyPb.StartRecordingStateResponse, request =>
      new choreographyPb.StartRecordingStateResponse()
        .setStatus(choreographyPb.StartRecordingStateResponse.Status.STATUS_OK)
        .setRecordingSessionId(
          request.getRecordingSessionId() !== '0' ? request.getRecordingSessionId() : String(nextRecordingSession++),
        ),
    ),
    stopRecordingState: unary(
      'StopRecordingState',
      choreographyPb.StopRecordingStateResponse,
      () => new choreographyPb.StopRecordingStateResponse(),
    ),
    choreographyStatus: unary('ChoreographyStatus', choreographyPb.ChoreographyStatusResponse, (request, { robot }) =>
      robot.choreography.statusToProto(),
    ),
  },
  directory: [
    { name: 'choreography', type: 'bosdyn.api.spot.ChoreographyService', authority: 'choreography.spot.robot' },
  ],
};
