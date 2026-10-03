'use strict';

const robotIdPb = require('../bosdyn/api/robot_id_pb');
const { RobotIdServiceService } = require('../bosdyn/api/robot_id_service_grpc_pb');
const { secToTimestamp } = require('../sim/clock');
const { unary } = require('../util');

/**
 * GetRobotId: the identity of the robot (no user token needed, like a real robot).
 * @param {robotIdPb.RobotIdRequest} request
 * @param {{robot: import('../robot').Robot}} context
 * @returns {robotIdPb.RobotIdResponse}
 */
function getRobotId(request, { robot }) {
  const config = robot.config.robot;
  const { major, minor, patch } = config.softwareVersion;
  const release = new robotIdPb.RobotSoftwareRelease()
    .setVersion(new robotIdPb.SoftwareVersion().setMajorVersion(major).setMinorVersion(minor).setPatchLevel(patch))
    .setName(config.softwareName)
    .setType('release')
    .setChangeset('5f3a1c2')
    .setChangesetDate(secToTimestamp(Date.parse('2025-10-01T12:00:00Z') / 1000))
    .setApiVersion(config.apiVersion)
    .setBuildInformation('spot-server-js simulator')
    .setInstallDate(secToTimestamp(Date.parse('2025-11-15T09:30:00Z') / 1000));
  const robotId = new robotIdPb.RobotId()
    .setSerialNumber(config.serialNumber)
    .setSpecies(config.species)
    .setVersion(config.version)
    .setSoftwareRelease(release)
    .setNickname(config.nickname)
    .setComputerSerialNumber(config.computerSerialNumber);
  return new robotIdPb.RobotIdResponse().setRobotId(robotId);
}

module.exports = {
  service: RobotIdServiceService,
  func: {
    getRobotId: unary('GetRobotId', robotIdPb.RobotIdResponse, getRobotId, { tokenRequired: false }),
  },
  directory: [
    { name: 'robot-id', type: 'bosdyn.api.RobotIdService', authority: 'id.spot.robot', userTokenRequired: false },
  ],
};
