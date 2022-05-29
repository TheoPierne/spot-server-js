'use strict';

const _ = require('lodash');

const geometry_pb = require('../bosdyn/api/geometry_pb');
const world_object_pb = require('../bosdyn/api/world_object_pb');
const world_object_service_grpc_pb = require('../bosdyn/api/world_object_service_grpc_pb');
const { LoggerUtil } = require('../loggerUtil');

const { populate_response_header, now_timestamp } = require('../util');

const logger = LoggerUtil.getLogger('WORLD_OBJECTS');

function listWorldObjects(call, callback) {
  logger.info('New request /listWorldObjects !');
  let reply = new world_object_pb.ListWorldObjectResponse();
  populate_response_header(reply, call.request);

  for (const i of Array.from({length: _.random(0, 10)}, ((_, i) => i))) {

    const frameTreeSnapshot = new geometry_pb.FrameTreeSnapshot();

    for (const u of Array.from({length: _.random(1, 5)}, ((_, u) => u))) {

      const pos = new geometry_pb.Vec3()
      .setX(3)
      .setY(1)
      .setZ(2);

      const quat = new geometry_pb.Quaternion()
      .setX(1)
      .setY(0)
      .setZ(0)
      .setW(0);

      const  SE3Pose = new geometry_pb.SE3Pose()
      .setPosition(pos)
      .setRotation(quat);

      const child = new geometry_pb.FrameTreeSnapshot.ParentEdge()
      .setParentFrameName(`PARENT_FRAME_NAME_${i}-${u}`)
      .setParentTformChild(SE3Pose);

      frameTreeSnapshot.getChildToParentEdgeMapMap()
      .set(`FRAME_TREE_SNAPSHOT_${i}-${u}`, child)
    }

    const worldObject = new world_object_pb.WorldObject()
    .setId(i)
    .setName(`OBJECT_${i}`)
    .setAcquisitionTime(now_timestamp())
    .setTransformsSnapshot(frameTreeSnapshot);

    reply.addWorldObjects(worldObject);
  }

  callback(null, reply);
}

module.exports = {
  service: world_object_service_grpc_pb.WorldObjectServiceService,
  func: {
    listWorldObjects,
  },
};
