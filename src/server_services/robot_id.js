'use strict';

const robot_id_pb = require('../bosdyn/api/robot_id_pb');
const robot_id_service_grpc_pb = require('../bosdyn/api/robot_id_service_grpc_pb');
const { LoggerUtil } = require('../loggerUtil');
const { populate_response_header, now_timestamp } = require('../util');

const logger = LoggerUtil.getLogger('ID');

function getRobotId(call, callback) {
  logger.info('New request /getRobotId !');
  let reply = new robot_id_pb.RobotIdResponse();
  populate_response_header(reply, call.request);

  const versionSoftwareRelease = new robot_id_pb.SoftwareVersion()
    .setMajorVersion(1)
    .setMinorVersion(0)
    .setPatchLevel(0);

  const softwareRelease = new robot_id_pb.RobotSoftwareRelease()
    .setName('TEST_RELEASE')
    .setType('TEST_TYPE')
    .setChangeset('TEST CHANGE SET')
    .setChangesetDate(now_timestamp())
    .setInstallDate(now_timestamp())
    .setApiVersion('v1.0.0')
    .setVersion(versionSoftwareRelease);

  const robotId = new robot_id_pb.RobotId()
    .setSerialNumber('1203')
    .setNickname('TEST_SERVER')
    .setSpecies('SERVER')
    .setVersion('1.0.0')
    .setSoftwareRelease(softwareRelease);

  reply.setRobotId(robotId);
  callback(null, reply);
}

module.exports = {
  service: robot_id_service_grpc_pb.RobotIdServiceService,
  func: {
    getRobotId,
  },
};
