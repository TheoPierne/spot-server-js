'use strict';

const { durationToSec, secToTimestamp, timestampToSec } = require('./clock');
const { frameTreeSnapshot, pose, poseInv, poseMul, quatFromAxes, quatFromYaw } = require('./math');
const { DockType } = require('../bosdyn/api/docking/docking_pb');
const geometryPb = require('../bosdyn/api/geometry_pb');
const worldObjectPb = require('../bosdyn/api/world_object_pb');
const { LoggerUtil } = require('../loggerUtil');

const logger = LoggerUtil.getLogger('WORLD');

const { WorldObjectType } = worldObjectPb;

// The robot reports the objects it saw in the last 15 seconds.
const OBJECT_MEMORY_SEC = 15;
// Distance between the base of a dock (where the robot sits) and its fiducial, and height of the fiducial.
const DOCK_TAG_OFFSET = 0.5;
const DOCK_TAG_HEIGHT = 0.45;

/**
 * The pose of a fiducial facing a direction: z out of the tag, x up, y to the left of a viewer.
 * @param {number} x
 * @param {number} y
 * @param {number} z
 * @param {number} yaw The direction the tag faces.
 * @returns {import('./math').Pose}
 */
function fiducialPose(x, y, z, yaw) {
  const c = Math.cos(yaw);
  const s = Math.sin(yaw);
  return pose(x, y, z, quatFromAxes({ x: 0, y: 0, z: 1 }, { x: s, y: -c, z: 0 }, { x: c, y: s, z: 0 }));
}

/**
 * @param {worldObjectPb.WorldObject} object
 * @returns {number[]} The WorldObjectType of an object.
 */
function objectTypes(object) {
  const types = [];
  if (object.getDrawablePropertiesList().length > 0) types.push(WorldObjectType.WORLD_OBJECT_DRAWABLE);
  if (object.hasApriltagProperties()) types.push(WorldObjectType.WORLD_OBJECT_APRILTAG);
  if (object.hasImageProperties()) types.push(WorldObjectType.WORLD_OBJECT_IMAGE_COORDINATES);
  if (object.hasDockProperties()) types.push(WorldObjectType.WORLD_OBJECT_DOCK);
  if (object.hasTrackedEntityProperties()) types.push(WorldObjectType.WORLD_OBJECT_TRACKED_ENTITY);
  if (object.hasNogoRegionProperties()) types.push(WorldObjectType.WORLD_OBJECT_USER_NOGO);
  if (object.hasStaircaseProperties()) types.push(WorldObjectType.WORLD_OBJECT_STAIRCASE);
  return types;
}

/**
 * The world around the robot: the room, the AprilTag fiducials and the dock that the robot detects with its cameras
 * when it is close enough, and the objects that clients add with MutateWorldObjects.
 */
class World {
  /**
   * @param {import('../robot')} robot
   */
  constructor(robot) {
    this.robot = robot;
    const config = robot.config;
    this.sizeMm = config.world.fiducialSizeMm;
    this.range = config.world.detectionRange;
    const dock = config.dock;
    this.docks = dock
      ? [
          {
            id: dock.id,
            base: { x: dock.x, y: dock.y, yaw: dock.yaw },
            tagPose: fiducialPose(
              dock.x - DOCK_TAG_OFFSET * Math.cos(dock.yaw),
              dock.y - DOCK_TAG_OFFSET * Math.sin(dock.yaw),
              DOCK_TAG_HEIGHT,
              dock.yaw,
            ),
          },
        ]
      : [];
    /** @type {{id: number, pose: import('./math').Pose, dock: ?object}[]} */
    this.fiducials = [
      ...config.world.fiducials.map(f => ({ id: f.id, pose: fiducialPose(f.x, f.y, f.z, f.yaw), dock: null })),
      ...this.docks.map(d => ({ id: d.id, pose: d.tagPose, dock: d })),
    ];
    this.nextObjectId = 1 + Math.floor(Math.random() * 100);
    /** @type {Map<number, {objectId: number, dockObjectId: ?number, lastSeen: number, observations: number,
     *   camera: string}>} */
    this.detections = new Map();
    /** @type {Map<number, {proto: worldObjectPb.WorldObject, expires: ?number}>} */
    this.userObjects = new Map();
    this._lastDetection = 0;
  }

  /** Forgets the detections and the user objects (reboot). */
  reset() {
    this.detections.clear();
    this.userObjects.clear();
  }

  /**
   * The camera which sees a point of the world, from the body.
   * @param {{x: number, y: number}} point
   * @returns {string}
   */
  cameraSeeing(point) {
    const body = this.robot.body.bodyPoseWorld();
    const local = poseMul(poseInv(body), pose(point.x, point.y, 0));
    const angle = Math.atan2(local.y, local.x);
    if (Math.abs(angle) < Math.PI / 4) return angle > 0 ? 'frontleft_fisheye' : 'frontright_fisheye';
    if (Math.abs(angle) > (3 * Math.PI) / 4) return 'back_fisheye';
    return angle > 0 ? 'left_fisheye' : 'right_fisheye';
  }

  /**
   * Whether the robot sees a fiducial: close enough, in front of the tag.
   * @param {{pose: import('./math').Pose}} fiducial
   * @returns {boolean}
   */
  isVisible(fiducial) {
    if (this.robot.body.isFallen()) return false;
    const body = this.robot.body.bodyPoseWorld();
    const dx = body.x - fiducial.pose.x;
    const dy = body.y - fiducial.pose.y;
    const distance = Math.hypot(dx, dy);
    if (distance > this.range || distance < 0.2) return false;
    // The normal of the tag (its z axis) points toward the robot.
    const { w, x, y, z } = fiducial.pose.rot;
    const normal = { x: 2 * (x * z + w * y), y: 2 * (y * z - w * x) };
    return (normal.x * dx + normal.y * dy) / distance > 0.25;
  }

  /**
   * Updates the detections (a few times per second).
   * @param {number} now
   */
  update(now) {
    if (now - this._lastDetection < 0.2) return;
    this._lastDetection = now;
    for (const fiducial of this.fiducials) {
      if (!this.isVisible(fiducial)) continue;
      let detection = this.detections.get(fiducial.id);
      if (!detection) {
        detection = {
          objectId: this.nextObjectId++,
          dockObjectId: fiducial.dock ? this.nextObjectId++ : null,
          lastSeen: now,
          observations: 0,
          camera: '',
        };
        this.detections.set(fiducial.id, detection);
        logger.info(`Fiducial ${fiducial.id} detected${fiducial.dock ? ' (dock)' : ''}`);
      }
      detection.lastSeen = now;
      detection.observations += 1;
      detection.camera = this.cameraSeeing(fiducial.pose);
    }
    for (const [id, entry] of this.userObjects) {
      if (entry.expires !== null && now > entry.expires) this.userObjects.delete(id);
    }
  }

  /**
   * @param {number} dockId
   * @returns {?object} The dock, if the robot saw it recently.
   */
  detectedDock(dockId) {
    const dock = this.docks.find(d => d.id === dockId);
    if (!dock) return null;
    const detection = this.detections.get(dockId);
    if (!detection || this.robot.clock.now() - detection.lastSeen > OBJECT_MEMORY_SEC) return null;
    return dock;
  }

  /**
   * @param {number} dockId
   * @returns {?object} The dock of this id (seen or not).
   */
  dock(dockId) {
    return this.docks.find(d => d.id === dockId) ?? null;
  }

  /**
   * The edges of the inertial frames, for the snapshots of the objects (body root, vision and odom).
   * @returns {Object<string, [string, ?import('./math').Pose]>}
   */
  _baseEdges() {
    const { body } = this.robot;
    const bodyTworld = poseInv(body.bodyPoseWorld());
    return {
      body: ['', null],
      vision: ['body', poseMul(bodyTworld, body.visionInWorld)],
      odom: ['body', poseMul(bodyTworld, body.odomInWorld())],
    };
  }

  /**
   * @param {object} fiducial
   * @param {object} detection
   * @returns {worldObjectPb.WorldObject[]} The AprilTag object, and the dock object for a dock.
   */
  _fiducialObjects(fiducial, detection) {
    const { body } = this.robot;
    const visionTworld = poseInv(body.visionInWorld);
    const tagInVision = poseMul(visionTworld, fiducial.pose);
    // The raw detection is a little noisy.
    const noise = Math.sin(detection.observations * 1.7) * 0.004;
    const rawInVision = { ...tagInVision, x: tagInVision.x + noise, y: tagInVision.y - noise };
    const edges = {
      ...this._baseEdges(),
      [`fiducial_${fiducial.id}`]: ['vision', rawInVision],
      [`filtered_fiducial_${fiducial.id}`]: ['vision', tagInVision],
    };
    const { AprilTagPoseStatus } = worldObjectPb.AprilTagProperties;
    const acquisition = secToTimestamp(detection.lastSeen);
    const tag = new worldObjectPb.WorldObject()
      .setId(detection.objectId)
      .setName(`world_obj_apriltag_${fiducial.id}`)
      .setAcquisitionTime(acquisition)
      .setTransformsSnapshot(frameTreeSnapshot(edges))
      .setApriltagProperties(
        new worldObjectPb.AprilTagProperties()
          .setTagId(fiducial.id)
          .setDimensions(new geometryPb.Vec2().setX(this.sizeMm).setY(this.sizeMm))
          .setFrameNameFiducial(`fiducial_${fiducial.id}`)
          .setFiducialPoseStatus(AprilTagPoseStatus.STATUS_OK)
          .setFrameNameFiducialFiltered(`filtered_fiducial_${fiducial.id}`)
          .setFiducialFilteredPoseStatus(AprilTagPoseStatus.STATUS_OK)
          .setFrameNameCamera(detection.camera)
          .setDetectionCovarianceReferenceFrame(`fiducial_${fiducial.id}`)
          .setNumObservations(detection.observations),
      );
    const objects = [tag];
    if (fiducial.dock) {
      const dock = fiducial.dock;
      const dockInVision = poseMul(visionTworld, pose(dock.base.x, dock.base.y, 0, quatFromYaw(dock.base.yaw)));
      objects.push(
        new worldObjectPb.WorldObject()
          .setId(detection.dockObjectId)
          .setName(`dock_${dock.id}`)
          .setAcquisitionTime(acquisition)
          .setTransformsSnapshot(
            frameTreeSnapshot({ ...this._baseEdges(), [`dock_${dock.id}`]: ['vision', dockInVision] }),
          )
          .setDockProperties(
            new worldObjectPb.DockProperties()
              .setDockId(dock.id)
              .setType(DockType.DOCK_TYPE_SPOT_DOCK)
              .setFrameNameDock(`dock_${dock.id}`)
              .setUnavailable(false)
              .setFromPrior(false),
          ),
      );
    }
    return objects;
  }

  /**
   * ListWorldObjects.
   * @param {number[]} types Filter on the types (all when empty).
   * @param {?number} after Only the objects acquired after this time.
   * @returns {worldObjectPb.WorldObject[]}
   */
  list(types, after) {
    const now = this.robot.clock.now();
    const objects = [];
    for (const fiducial of this.fiducials) {
      const detection = this.detections.get(fiducial.id);
      if (detection && now - detection.lastSeen <= OBJECT_MEMORY_SEC) {
        objects.push(...this._fiducialObjects(fiducial, detection));
      }
    }
    for (const entry of this.userObjects.values()) objects.push(entry.proto);
    return objects.filter(object => {
      if (types.length > 0 && !objectTypes(object).some(type => types.includes(type))) return false;
      if (after !== null && (timestampToSec(object.getAcquisitionTime()) ?? 0) <= after) return false;
      return true;
    });
  }

  /**
   * MutateWorldObjects.
   * @param {worldObjectPb.MutateWorldObjectRequest.Mutation} mutation
   * @returns {{status: number, id: number}}
   */
  mutate(mutation) {
    const Status = worldObjectPb.MutateWorldObjectResponse.Status;
    const { Action } = worldObjectPb.MutateWorldObjectRequest;
    const object = mutation?.getObject();
    const now = this.robot.clock.now();
    const expiresOf = proto => {
      const lifetime = durationToSec(proto.getObjectLifetime());
      return lifetime ? now + lifetime : null;
    };
    switch (mutation?.getAction()) {
      case Action.ACTION_ADD: {
        if (!object) return { status: Status.STATUS_INVALID_WORLD_OBJECT, id: 0 };
        const id = this.nextObjectId++;
        const proto = object.clone().setId(id);
        if (!proto.hasAcquisitionTime()) proto.setAcquisitionTime(secToTimestamp(now));
        this.userObjects.set(id, { proto, expires: expiresOf(proto) });
        logger.info(`Object ${id} "${proto.getName()}" added`);
        return { status: Status.STATUS_OK, id };
      }
      case Action.ACTION_CHANGE: {
        const id = object?.getId();
        if (!this.userObjects.has(id)) {
          return {
            status: this._isRobotObject(id) ? Status.STATUS_NO_PERMISSION : Status.STATUS_INVALID_MUTATION_ID,
            id: 0,
          };
        }
        const proto = object.clone();
        if (!proto.hasAcquisitionTime()) proto.setAcquisitionTime(secToTimestamp(now));
        this.userObjects.set(id, { proto, expires: expiresOf(proto) });
        return { status: Status.STATUS_OK, id };
      }
      case Action.ACTION_DELETE: {
        const id = object?.getId();
        if (!this.userObjects.delete(id)) {
          return {
            status: this._isRobotObject(id) ? Status.STATUS_NO_PERMISSION : Status.STATUS_INVALID_MUTATION_ID,
            id: 0,
          };
        }
        logger.info(`Object ${id} deleted`);
        return { status: Status.STATUS_OK, id };
      }
      default:
        return { status: Status.STATUS_UNKNOWN, id: 0, invalid: true };
    }
  }

  _isRobotObject(id) {
    return [...this.detections.values()].some(detection => detection.objectId === id || detection.dockObjectId === id);
  }

  /**
   * The user no-go regions (boxes), in the world, for the obstacles of the robot.
   * @returns {{object: worldObjectPb.WorldObject}[]}
   */
  nogoRegions() {
    return [...this.userObjects.values()].filter(entry => entry.proto.hasNogoRegionProperties());
  }

  /** @returns {string[]} */
  describe() {
    const now = this.robot.clock.now();
    const lines = [];
    for (const [id, detection] of this.detections) {
      const age = now - detection.lastSeen;
      if (age <= OBJECT_MEMORY_SEC) lines.push(`fiducial ${id} seen ${age.toFixed(1)} s ago by ${detection.camera}`);
    }
    for (const entry of this.userObjects.values()) {
      lines.push(`object ${entry.proto.getId()} "${entry.proto.getName()}"`);
    }
    return lines;
  }
}

module.exports = { World, fiducialPose, objectTypes };
