'use strict';

const missionPb = require('../bosdyn/api/mission/mission_pb');
const { MissionServiceService } = require('../bosdyn/api/mission/mission_service_grpc_pb');
const { LeaseManager, leaseFromProto } = require('../sim/lease');
const { clientStreaming, unary } = require('../util');

/**
 * The lease use results of the leases of a mission request.
 * @param {import('../robot').Robot} robot
 * @param {import('../bosdyn/api/lease_pb').Lease[]} leases
 * @returns {import('../bosdyn/api/lease_pb').LeaseUseResult[]}
 */
function useLeases(robot, leases) {
  return leases.map(lease =>
    LeaseManager.resultToProto(robot.leases.use(leaseFromProto(lease), null, { allowSuperLease: true })),
  );
}

/**
 * LoadMission.
 * @param {missionPb.LoadMissionRequest} request
 * @param {{robot: import('../robot').Robot}} context
 * @returns {missionPb.LoadMissionResponse}
 */
function loadMission(request, { robot }) {
  const { Status } = missionPb.LoadMissionResponse;
  const response = new missionPb.LoadMissionResponse().setLeaseUseResultsList(
    useLeases(robot, request.getLeasesList()),
  );
  if (!request.hasRoot()) {
    return response
      .setStatus(Status.STATUS_COMPILE_ERROR)
      .setFailedNodesList([new missionPb.FailedNode().setName('').setError('The mission has no root node.')]);
  }
  return response.setStatus(Status.STATUS_OK).setMissionInfo(robot.missions.load(request.getRoot()));
}

/**
 * LoadMissionAsChunks: the chunks hold a serialized LoadMissionRequest.
 * @param {import('../bosdyn/api/data_chunk_pb').DataChunk[]} chunks
 * @param {{robot: import('../robot').Robot}} context
 * @returns {missionPb.LoadMissionResponse}
 */
function loadMissionAsChunks(chunks, context) {
  const parts = chunks.map(chunk => chunk.getData_asU8());
  const total = parts.reduce((sum, part) => sum + part.length, 0);
  const data = new Uint8Array(total);
  let offset = 0;
  for (const part of parts) {
    data.set(part, offset);
    offset += part.length;
  }
  return loadMission(missionPb.LoadMissionRequest.deserializeBinary(data), context);
}

/**
 * PlayMission / RestartMission.
 * @param {boolean} restart
 * @returns {function(any, object): any}
 */
function play(restart) {
  const Response = restart ? missionPb.RestartMissionResponse : missionPb.PlayMissionResponse;
  return (request, { robot }) => {
    const results = useLeases(robot, request.getLeasesList());
    const played = robot.missions.play(request.getPauseTime(), restart);
    return new Response()
      .setStatus(played ? Response.Status.STATUS_OK : Response.Status.STATUS_NO_MISSION)
      .setLeaseUseResultsList(results);
  };
}

/**
 * PauseMission / StopMission.
 * @param {boolean} stop
 * @returns {function(any, object): any}
 */
function pauseOrStop(stop) {
  const Response = stop ? missionPb.StopMissionResponse : missionPb.PauseMissionResponse;
  return (request, { robot }) => {
    const response = new Response();
    if (request.hasLease()) {
      response.setLeaseUseResult(
        LeaseManager.resultToProto(
          robot.leases.use(leaseFromProto(request.getLease()), null, { allowSuperLease: true }),
        ),
      );
    }
    const done = stop ? robot.missions.stop() : robot.missions.pause();
    return response.setStatus(done ? Response.Status.STATUS_OK : Response.Status.STATUS_NO_MISSION_PLAYING);
  };
}

module.exports = {
  service: MissionServiceService,
  func: {
    loadMission: unary('LoadMission', missionPb.LoadMissionResponse, loadMission),
    loadMissionAsChunks: clientStreaming('LoadMissionAsChunks', missionPb.LoadMissionResponse, loadMissionAsChunks),
    playMission: unary('PlayMission', missionPb.PlayMissionResponse, play(false)),
    restartMission: unary('RestartMission', missionPb.RestartMissionResponse, play(true)),
    pauseMission: unary('PauseMission', missionPb.PauseMissionResponse, pauseOrStop(false)),
    stopMission: unary('StopMission', missionPb.StopMissionResponse, pauseOrStop(true)),
    getState: unary('GetState', missionPb.GetStateResponse, (request, { robot }) =>
      new missionPb.GetStateResponse().setState(robot.missions.stateToProto()),
    ),
    getInfo: unary('GetInfo', missionPb.GetInfoResponse, (request, { robot }) => {
      const response = new missionPb.GetInfoResponse();
      if (robot.missions.mission) response.setMissionInfo(robot.missions.mission.info);
      return response;
    }),
    getMission: unary('GetMission', missionPb.GetMissionResponse, (request, { robot }) => {
      const response = new missionPb.GetMissionResponse();
      const { mission } = robot.missions;
      if (mission) response.setRoot(mission.root).setId(mission.info.getId());
      return response;
    }),
    answerQuestion: unary('AnswerQuestion', missionPb.AnswerQuestionResponse, () =>
      new missionPb.AnswerQuestionResponse().setStatus(
        missionPb.AnswerQuestionResponse.Status.STATUS_INVALID_QUESTION_ID,
      ),
    ),
  },
  directory: [
    { name: 'robot-mission', type: 'bosdyn.api.mission.MissionService', authority: 'robot-mission.spot.robot' },
  ],
};
