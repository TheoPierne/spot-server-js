'use strict';

const { DoubleValue } = require('google-protobuf/google/protobuf/wrappers_pb');

const {
  approach,
  clamp,
  eulerZXYFromQuat,
  planarDistance,
  pose,
  poseInv,
  poseMul,
  quatFromEulerZXY,
  quatFromYaw,
  transformPoint,
  vec3ToProto,
  velocityToProto,
  wrapAngle,
  yawOf,
} = require('./math');
const robotStatePb = require('../bosdyn/api/robot_state_pb');
const { LoggerUtil } = require('../loggerUtil');

const logger = LoggerUtil.getLogger('BODY');

// Geometry of the legs (from the URDF of Spot): hips, lateral offset of the upper legs, lengths of the legs.
const HIP_X = 0.29785;
const HIP_Y = 0.055;
const LEG_Y = 0.110945;
const UPPER_LEG = 0.32;
const LOWER_LEG = 0.35;
const FOOT_Y = HIP_Y + LEG_Y;

const LEGS = [
  { name: 'fl', x: HIP_X, side: 1 },
  { name: 'fr', x: HIP_X, side: -1 },
  { name: 'hl', x: -HIP_X, side: 1 },
  { name: 'hr', x: -HIP_X, side: -1 },
];
const LEG_JOINTS = LEGS.flatMap(leg => [`${leg.name}.hx`, `${leg.name}.hy`, `${leg.name}.kn`]);

// The center of the robot stays this far from the walls.
const WALL_MARGIN = 0.6;

/**
 * Postures of the robot.
 * @enum {string}
 */
const Posture = {
  SITTING: 'sitting',
  STANDING_UP: 'standing_up',
  STANDING: 'standing',
  SITTING_DOWN: 'sitting_down',
  FALLEN: 'fallen',
  SELF_RIGHTING: 'self_righting',
  ROLLING_OVER: 'rolling_over',
  ROLLED_OVER: 'rolled_over',
};

/**
 * Inverse kinematics of a leg in its sagittal plane.
 * @param {{x: number, y: number, z: number}} footRtHip Position of the foot relative to the hip, in the body frame.
 * @param {number} side 1 for the left legs, -1 for the right legs.
 * @returns {{hx: number, hy: number, kn: number}}
 */
function legIK(footRtHip, side) {
  const lateral = footRtHip.y - side * LEG_Y;
  const hx = Math.atan2(lateral, -footRtHip.z) * side;
  const dz = -Math.hypot(footRtHip.z, lateral);
  const dx = footRtHip.x;
  const reach = clamp(Math.hypot(dx, dz), Math.abs(UPPER_LEG - LOWER_LEG) + 1e-3, UPPER_LEG + LOWER_LEG - 1e-3);
  const kneeInner = Math.acos(
    clamp((UPPER_LEG ** 2 + LOWER_LEG ** 2 - reach ** 2) / (2 * UPPER_LEG * LOWER_LEG), -1, 1),
  );
  const kn = -(Math.PI - kneeInner);
  const alpha = Math.atan2(dx, -dz);
  const beta = Math.acos(clamp((UPPER_LEG ** 2 + reach ** 2 - LOWER_LEG ** 2) / (2 * UPPER_LEG * reach), -1, 1));
  return { hx, hy: beta - alpha, kn };
}

/**
 * The body of the robot: its posture (sitting, standing, fallen...), its position in the world, the motion of the
 * mobility commands (velocity, trajectories, body offsets, stance), the odometry ("odom" drifts slowly from "vision"),
 * the legs (joint angles from the inverse kinematics, foot contacts while walking).
 *
 * The world frame of the simulation has its origin on the floor; the "vision" and "odom" frames have their origin at
 * the body of the robot when it boots, like on a real robot.
 */
class Body {
  /**
   * @param {import('../robot')} robot
   */
  constructor(robot) {
    this.robot = robot;
    this.config = robot.config.mobility;
    this.footprint = { x: 0, y: 0, yaw: 0 };
    this.posture = Posture.SITTING;
    this.boot();
  }

  /**
   * Resets the localization and the motion (boot of the robot, which keeps its physical position).
   */
  boot() {
    if (this.posture !== Posture.FALLEN && this.posture !== Posture.ROLLED_OVER) this.posture = Posture.SITTING;
    // Body offset relative to the footprint (and its target), body height relative to the nominal stand height.
    this.offset = { x: 0, y: 0, height: 0, yaw: 0, roll: 0, pitch: 0 };
    this.offsetTarget = { ...this.offset };
    this.velocity = { x: 0, y: 0, yaw: 0 };
    this.transition = null;
    this.behavior = { kind: 'idle', owner: null, state: {} };
    this.stanceOffsets = Object.fromEntries(LEGS.map(leg => [leg.name, { x: 0, y: 0 }]));
    this.gaitPhase = 0;
    this.blocked = false;
    this.distanceWalked = 0;
    this.lastStill = this.robot.clock.now();
    // Localization: the vision frame at the body of the robot at boot, odom drifting from it.
    const bodyWorld = this.bodyPoseWorld();
    this.visionInWorld = pose(bodyWorld.x, bodyWorld.y, bodyWorld.z, quatFromYaw(yawOf(bodyWorld.rot)));
    this.odomDrift = { x: 0, y: 0, yaw: 0 };
    this._legState = null;
  }

  /** @returns {number} The height of the body above the floor. */
  get height() {
    const { sitHeight, standHeight } = this.config;
    switch (this.posture) {
      case Posture.STANDING:
        return standHeight + this.offset.height;
      case Posture.STANDING_UP:
      case Posture.SITTING_DOWN: {
        const t = this._transitionFraction();
        const standing = standHeight + this.offset.height;
        return this.posture === Posture.STANDING_UP
          ? sitHeight + (standing - sitHeight) * t
          : standing + (sitHeight - standing) * t;
      }
      case Posture.FALLEN:
      case Posture.ROLLED_OVER:
      case Posture.ROLLING_OVER:
        return 0.22;
      default:
        return sitHeight;
    }
  }

  _transitionFraction() {
    if (!this.transition) return 1;
    return clamp((this.robot.clock.now() - this.transition.start) / this.transition.duration, 0, 1);
  }

  /** @returns {number} The roll of the body (on its side when fallen or rolled over for a battery change). */
  get roll() {
    const side = this.transition?.side ?? this.rollSide ?? 1;
    switch (this.posture) {
      case Posture.FALLEN:
      case Posture.ROLLED_OVER:
        return (side * Math.PI) / 2;
      case Posture.ROLLING_OVER:
        return (side * Math.PI * this._transitionFraction()) / 2;
      case Posture.SELF_RIGHTING:
        return (side * Math.PI * (1 - this._transitionFraction())) / 2;
      default:
        return this.offset.roll;
    }
  }

  /**
   * The pose of the body in the world.
   * @returns {import('./math').Pose}
   */
  bodyPoseWorld() {
    const c = Math.cos(this.footprint.yaw);
    const s = Math.sin(this.footprint.yaw);
    const offset = this.offset ?? { x: 0, y: 0, yaw: 0, pitch: 0 };
    const upright = this.posture === Posture.STANDING || this.posture === Posture.STANDING_UP;
    const ox = upright ? offset.x : 0;
    const oy = upright ? offset.y : 0;
    return pose(
      this.footprint.x + c * ox - s * oy,
      this.footprint.y + s * ox + c * oy,
      this.height,
      quatFromEulerZXY(this.footprint.yaw + offset.yaw, this.roll ?? 0, upright ? offset.pitch : 0),
    );
  }

  /** @returns {boolean} */
  isStanding() {
    return this.posture === Posture.STANDING;
  }

  /** @returns {boolean} Sitting (or lying), not moving. */
  isResting() {
    return [Posture.SITTING, Posture.FALLEN, Posture.ROLLED_OVER].includes(this.posture);
  }

  /** @returns {boolean} */
  isWalking() {
    return Math.hypot(this.velocity.x, this.velocity.y) > 0.02 || Math.abs(this.velocity.yaw) > 0.02;
  }

  /** @returns {boolean} Lying on its side. */
  isFallen() {
    return this.posture === Posture.FALLEN;
  }

  // Frames.

  /** @returns {import('./math').Pose} world_tform_odom */
  odomInWorld() {
    const drift = pose(this.odomDrift.x, this.odomDrift.y, 0, quatFromYaw(this.odomDrift.yaw));
    return poseMul(this.visionInWorld, drift);
  }

  /**
   * The pose of an inertial or body frame in the world.
   * @param {string} frame 'vision', 'odom', 'body', 'flat_body', 'gpe'.
   * @returns {?import('./math').Pose}
   */
  frameInWorld(frame) {
    switch (frame) {
      case 'vision':
        return this.visionInWorld;
      case 'odom':
        return this.odomInWorld();
      case 'body':
        return this.bodyPoseWorld();
      case 'flat_body':
      case 'gravity_aligned_body': {
        const body = this.bodyPoseWorld();
        return pose(body.x, body.y, body.z, quatFromYaw(yawOf(body.rot)));
      }
      case 'gpe': {
        const body = this.bodyPoseWorld();
        return pose(body.x, body.y, 0);
      }
      case 'footprint':
      case 'feet_center':
        return pose(this.footprint.x, this.footprint.y, 0, quatFromYaw(this.footprint.yaw));
      default:
        return this.robot.arm?.frameInWorld(frame) ?? null;
    }
  }

  /**
   * A planar pose expressed in a frame, converted to the world.
   * @param {string} frame
   * @param {{x: number, y: number, yaw: number}} planar
   * @returns {?{x: number, y: number, yaw: number}}
   */
  planarToWorld(frame, planar) {
    const frameWorld = this.frameInWorld(frame === 'body' ? 'flat_body' : frame);
    if (!frameWorld) return null;
    const p = transformPoint(frameWorld, { x: planar.x, y: planar.y, z: 0 });
    return { x: p.x, y: p.y, yaw: wrapAngle(yawOf(frameWorld.rot) + planar.yaw) };
  }

  /**
   * The edges of the frame tree of the robot state (the "body" frame is the root).
   * @returns {Object<string, [string, ?import('./math').Pose]>}
   */
  frameTreeEdges() {
    const worldTbody = this.bodyPoseWorld();
    const bodyTworld = poseInv(worldTbody);
    const edges = {
      body: ['', null],
      flat_body: ['body', poseMul(bodyTworld, this.frameInWorld('flat_body'))],
      odom: ['body', poseMul(bodyTworld, this.odomInWorld())],
      vision: ['body', poseMul(bodyTworld, this.visionInWorld)],
      gpe: ['odom', poseMul(poseInv(this.odomInWorld()), this.frameInWorld('gpe'))],
    };
    if (this.robot.arm) Object.assign(edges, this.robot.arm.frameTreeEdges());
    return edges;
  }

  // Behaviors.

  /**
   * Starts a mobility behavior (the previous one is overridden).
   * @param {{kind: string, owner: any}} behavior
   * @returns {{kind: string, owner: any}} The previous behavior.
   */
  start(behavior) {
    const previous = this.behavior;
    this.behavior = { state: {}, ...behavior, startTime: this.robot.clock.now() };
    if (behavior.kind !== 'stance') {
      // Walking resets a custom stance.
      for (const leg of LEGS) this.stanceOffsets[leg.name] = { x: 0, y: 0 };
    }
    return previous;
  }

  /** Stops the current behavior: the robot keeps its posture. */
  idle() {
    this.behavior = { kind: 'idle', owner: null, state: {}, startTime: this.robot.clock.now() };
  }

  _startTransition(posture, duration, extra = {}) {
    this.posture = posture;
    this.transition = { start: this.robot.clock.now(), duration, ...extra };
  }

  /** Stands up, if sitting. Returns true once standing. */
  _ensureStanding() {
    if (this.posture === Posture.SITTING) {
      this._startTransition(Posture.STANDING_UP, this.robot.config.durations.standUp);
    } else if (this.posture === Posture.SITTING_DOWN) {
      // Stands back up from where the body is.
      const fraction = 1 - this._transitionFraction();
      this._startTransition(Posture.STANDING_UP, this.robot.config.durations.standUp);
      this.transition.start -= fraction * this.transition.duration;
    }
    return this.posture === Posture.STANDING;
  }

  /** Sits down (once stopped). Returns true once sitting. */
  _ensureSitting() {
    if (this.posture === Posture.STANDING) {
      if (this.isWalking()) return false;
      this._startTransition(Posture.SITTING_DOWN, this.robot.config.durations.sitDown);
    } else if (this.posture === Posture.STANDING_UP) {
      const fraction = 1 - this._transitionFraction();
      this._startTransition(Posture.SITTING_DOWN, this.robot.config.durations.sitDown);
      this.transition.start -= fraction * this.transition.duration;
    }
    return this.posture === Posture.SITTING;
  }

  /**
   * Velocity limits of a behavior (MobilityParams.vel_limit), in the frame of the robot.
   * @param {{maxVel?: {x: number, y: number, yaw: number}}} behavior
   * @returns {{x: number, y: number, yaw: number}}
   */
  _limits(behavior) {
    const { maxVelX, maxVelY, maxVelYaw } = this.config;
    const max = behavior.maxVel ?? {};
    return {
      x: Math.min(maxVelX, max.x ?? maxVelX),
      y: Math.min(maxVelY, max.y ?? maxVelY),
      yaw: Math.min(maxVelYaw, max.yaw ?? maxVelYaw),
    };
  }

  /**
   * The desired velocity of the footprint (in the world) for the current behavior.
   * @param {number} now
   * @returns {{x: number, y: number, yaw: number, instant?: boolean}}
   */
  _desiredVelocity(now) {
    const behavior = this.behavior;
    const zero = { x: 0, y: 0, yaw: 0 };
    switch (behavior.kind) {
      case 'velocity': {
        if (!this._ensureStanding() || now > behavior.endTime) return zero;
        const limits = this._limits(behavior);
        // The velocity in the frame of the robot is clamped, then turned into the world.
        let vx = behavior.velocity.x;
        let vy = behavior.velocity.y;
        if (behavior.frame !== 'flat_body' && behavior.frame !== 'body') {
          const frameYaw = yawOf(this.frameInWorld(behavior.frame).rot) - this.footprint.yaw;
          [vx, vy] = [
            Math.cos(frameYaw) * vx - Math.sin(frameYaw) * vy,
            Math.sin(frameYaw) * vx + Math.cos(frameYaw) * vy,
          ];
        }
        vx = clamp(vx, -limits.x, limits.x);
        vy = clamp(vy, -limits.y, limits.y);
        const c = Math.cos(this.footprint.yaw);
        const s = Math.sin(this.footprint.yaw);
        return { x: c * vx - s * vy, y: s * vx + c * vy, yaw: clamp(behavior.velocity.yaw, -limits.yaw, limits.yaw) };
      }
      case 'trajectory':
        return this._trajectoryVelocity(now, behavior);
      case 'freeze':
        return { ...zero, instant: true };
      default:
        return zero;
    }
  }

  /**
   * Follows the points of an SE2 trajectory (in the world), the last one being the goal.
   * @param {number} now
   * @param {object} behavior
   * @returns {{x: number, y: number, yaw: number}}
   */
  _trajectoryVelocity(now, behavior) {
    const zero = { x: 0, y: 0, yaw: 0 };
    const state = behavior.state;
    if (!this._ensureStanding()) {
      state.status = 'in_progress';
      return zero;
    }
    if (now > behavior.endTime || state.status === 'stopped') return zero;
    // Stopped by a wall for a while: the goal is not achievable.
    if (this.blocked) {
      state.blockedSince ??= now;
      if (now - state.blockedSince > 1) {
        state.status = 'stopped';
        state.finalGoal = 'blocked';
        return zero;
      }
    } else {
      state.blockedSince = null;
    }
    const points = behavior.points;
    state.index ??= 0;
    const goal = points[points.length - 1];
    // Intermediate points are passed through with a looser tolerance.
    while (state.index < points.length - 1 && planarDistance(this.footprint, points[state.index]) < 0.25) {
      state.index += 1;
    }
    const target = points[state.index];
    const dist = planarDistance(this.footprint, target);
    const yawError = wrapAngle(target.yaw - this.footprint.yaw);
    const goalDist = planarDistance(this.footprint, goal);
    const isGoal = state.index === points.length - 1;
    if (isGoal && dist < this.config.goalPositionTolerance && Math.abs(yawError) < this.config.goalYawTolerance) {
      state.status = 'stopped';
      state.finalGoal = 'achievable';
      return zero;
    }
    state.status = goalDist < this.config.nearGoalDistance ? 'stopping' : 'in_progress';
    state.finalGoal = 'in_progress';
    const limits = this._limits(behavior);
    const speedLimit = Math.min(limits.x, behavior.maxVel ? limits.x : this.config.defaultTrajectoryVel);
    const speed = Math.min(speedLimit, 1.5 * dist + (isGoal ? 0.02 : 0.3));
    const direction = Math.atan2(target.y - this.footprint.y, target.x - this.footprint.x);
    // Lateral motion is slower: the speed is limited by the lateral component in the frame of the robot.
    const lateral = Math.abs(Math.sin(direction - this.footprint.yaw));
    const lateralSpeed = lateral > 1e-3 ? Math.min(speed, limits.y / lateral) : speed;
    const v = dist > 1e-4 ? Math.min(speed, lateralSpeed) : 0;
    return {
      x: v * Math.cos(direction),
      y: v * Math.sin(direction),
      yaw: clamp(2 * yawError, -limits.yaw, limits.yaw),
    };
  }

  /**
   * The body offset (height, orientation) of the stand behaviors.
   * @param {number} now
   */
  _updateOffsetTarget(now) {
    const behavior = this.behavior;
    if (!behavior.bodyControl) return;
    const { points, rootFrame, referenceTime } = behavior.bodyControl;
    const elapsed = now - referenceTime;
    // The point of the trajectory at this time (linear interpolation between the points).
    let target = points[0];
    for (let i = 0; i < points.length; i++) {
      if (elapsed >= points[i].t) target = points[i];
      if (elapsed < points[i].t) {
        const previous = i > 0 ? points[i - 1] : { t: 0, pose: this._currentOffsetPose(rootFrame) };
        const fraction = clamp((elapsed - previous.t) / Math.max(points[i].t - previous.t, 1e-6), 0, 1);
        target = { t: elapsed, pose: this._interpolatePose(previous.pose, points[i].pose, fraction) };
        break;
      }
    }
    this.offsetTarget = this._offsetFromPose(target.pose, rootFrame);
    behavior.state.trajectoryDone = elapsed >= points[points.length - 1].t;
  }

  _interpolatePose(a, b, t) {
    const ea = eulerZXYFromQuat(a.rot);
    const eb = eulerZXYFromQuat(b.rot);
    return pose(
      a.x + (b.x - a.x) * t,
      a.y + (b.y - a.y) * t,
      a.z + (b.z - a.z) * t,
      quatFromEulerZXY(
        ea.yaw + wrapAngle(eb.yaw - ea.yaw) * t,
        ea.roll + (eb.roll - ea.roll) * t,
        ea.pitch + (eb.pitch - ea.pitch) * t,
      ),
    );
  }

  /**
   * The current body offset, as a pose in the root frame of a body control.
   * @param {?string} rootFrame Null for an offset relative to the footprint.
   * @returns {import('./math').Pose}
   */
  _currentOffsetPose(rootFrame) {
    if (!rootFrame) {
      return pose(
        this.offset.x,
        this.offset.y,
        this.offset.height,
        quatFromEulerZXY(this.offset.yaw, this.offset.roll, this.offset.pitch),
      );
    }
    return poseMul(poseInv(this.frameInWorld(rootFrame)), this.bodyPoseWorld());
  }

  /**
   * Converts a desired body pose to an offset relative to the footprint (clamped to what the legs allow).
   * @param {import('./math').Pose} target
   * @param {?string} rootFrame Null for an offset relative to the footprint (the z is relative to the nominal height).
   * @returns {{x: number, y: number, height: number, yaw: number, roll: number, pitch: number}}
   */
  _offsetFromPose(target, rootFrame) {
    let x;
    let y;
    let height;
    let euler;
    if (rootFrame) {
      const world = poseMul(this.frameInWorld(rootFrame), target);
      const c = Math.cos(this.footprint.yaw);
      const s = Math.sin(this.footprint.yaw);
      const dx = world.x - this.footprint.x;
      const dy = world.y - this.footprint.y;
      x = c * dx + s * dy;
      y = -s * dx + c * dy;
      height = world.z - this.config.standHeight;
      euler = eulerZXYFromQuat(world.rot);
      euler.yaw = wrapAngle(euler.yaw - this.footprint.yaw);
    } else {
      ({ x, y } = target);
      height = target.z;
      euler = eulerZXYFromQuat(target.rot);
    }
    return {
      x: clamp(x, -0.15, 0.15),
      y: clamp(y, -0.1, 0.1),
      height: clamp(height, this.config.minBodyHeightOffset, this.config.maxBodyHeightOffset),
      yaw: clamp(euler.yaw, -0.5, 0.5),
      roll: clamp(euler.roll, -0.4, 0.4),
      pitch: clamp(euler.pitch, -0.5, 0.5),
    };
  }

  /**
   * One step of the simulation.
   * @param {number} now
   * @param {number} dt
   */
  update(now, dt) {
    const motorsOn = this.robot.power.motorsOn();
    this._updateTransition();
    let desired = { x: 0, y: 0, yaw: 0 };
    if (motorsOn) {
      this._runBehavior(now);
      desired = this._desiredVelocity(now);
    }
    if (!motorsOn || desired.instant || !this.isStanding()) {
      this.velocity = { x: desired.x, y: desired.y, yaw: desired.yaw };
      if (!this.isStanding()) this.velocity = { x: 0, y: 0, yaw: 0 };
    } else {
      const { maxAccel, maxYawAccel } = this.config;
      this.velocity.x = approach(this.velocity.x, desired.x, maxAccel * dt);
      this.velocity.y = approach(this.velocity.y, desired.y, maxAccel * dt);
      this.velocity.yaw = approach(this.velocity.yaw, desired.yaw, maxYawAccel * dt);
    }
    this._integrate(dt);
    this._updateOffsets(dt, motorsOn);
    if (this.isWalking()) {
      this.gaitPhase = (this.gaitPhase + dt * 1.8) % 1;
      this.lastStill = now;
    } else {
      this.gaitPhase = 0;
    }
  }

  _updateTransition() {
    if (!this.transition || this._transitionFraction() < 1) return;
    const { posture } = this;
    this.transition = null;
    if (posture === Posture.STANDING_UP) {
      this.posture = Posture.STANDING;
    } else if (posture === Posture.SITTING_DOWN) {
      this.posture = Posture.SITTING;
    } else if (posture === Posture.ROLLING_OVER) {
      this.posture = Posture.ROLLED_OVER;
    } else if (posture === Posture.SELF_RIGHTING) {
      this.posture = Posture.SITTING;
      this.robot.faults.clearBehaviorFaultsOfCause(robotStatePb.BehaviorFault.Cause.CAUSE_FALL);
      logger.info('Self-right completed');
    }
    if (this.posture === Posture.SITTING) this.offset = { ...this.offset, x: 0, y: 0, yaw: 0, roll: 0, pitch: 0 };
    this.robot.emit('posture', this.posture);
  }

  /**
   * Posture goals of the behaviors.
   * @param {number} now
   */
  _runBehavior(now) {
    const behavior = this.behavior;
    const state = behavior.state;
    if (['stand', 'follow_arm', 'velocity', 'trajectory', 'stop', 'stance'].includes(behavior.kind)) {
      this._updateOffsetTarget(now);
    }
    switch (behavior.kind) {
      case 'stand':
      case 'follow_arm':
        this._ensureStanding();
        break;
      case 'payload_estimation': {
        this._ensureStanding();
        state.progress = clamp((now - behavior.startTime) / this.robot.config.durations.payloadEstimation, 0, 1);
        // Small motions of the body while estimating.
        const phase = (now - behavior.startTime) * 2;
        this.offsetTarget = { ...this.offsetTarget, pitch: 0.1 * Math.sin(phase), roll: 0.08 * Math.sin(phase * 1.3) };
        if (state.progress >= 1) this.offsetTarget = { ...this.offsetTarget, pitch: 0, roll: 0 };
        break;
      }
      case 'sit':
        this._ensureSitting();
        break;
      case 'stop':
      case 'freeze':
        break;
      case 'stance':
        this._runStance(now, behavior);
        break;
      case 'selfright':
        if (this.posture === Posture.FALLEN || this.posture === Posture.ROLLED_OVER) {
          this._startTransition(Posture.SELF_RIGHTING, this.robot.config.durations.selfRight, { side: this.rollSide });
          logger.info('Self-righting...');
        }
        state.done = this.posture === Posture.SITTING;
        break;
      case 'battery_change':
        if (this.posture === Posture.STANDING || this.posture === Posture.STANDING_UP) {
          this._ensureSitting();
        } else if (this.posture === Posture.SITTING) {
          this.rollSide = behavior.side;
          this._startTransition(Posture.ROLLING_OVER, this.robot.config.durations.batteryChangePose, {
            side: behavior.side,
          });
        }
        state.done = this.posture === Posture.ROLLED_OVER;
        break;
      case 'safe_power_off':
        if (this._ensureSitting() && this.robot.power.motorsOn()) {
          this.robot.power.cutMotorPower(behavior.reason ?? 'safe power off');
        }
        break;
      default:
        break;
    }
  }

  _runStance(now, behavior) {
    const state = behavior.state;
    if (!this._ensureStanding()) return;
    if (state.status === undefined) {
      // The targets of the feet, relative to their nominal position under the hips.
      const footprintWorld = this.frameInWorld('footprint');
      const footprintInv = poseInv(footprintWorld);
      let tooFar = false;
      state.targets = {};
      for (const leg of LEGS) {
        const target = behavior.feet[leg.name];
        if (!target) continue;
        const local = transformPoint(footprintInv, target);
        const offset = { x: local.x - leg.x, y: local.y - leg.side * FOOT_Y };
        if (Math.hypot(offset.x, offset.y) > 0.35) tooFar = true;
        state.targets[leg.name] = offset;
      }
      state.status = tooFar ? 'too_far' : 'going';
      state.start = now;
    }
    if (state.status === 'going') {
      // One foot after the other.
      const fraction = clamp((now - state.start) / 1.2, 0, 1);
      LEGS.forEach((leg, index) => {
        const target = state.targets[leg.name];
        if (target && fraction >= (index + 1) / LEGS.length) this.stanceOffsets[leg.name] = { ...target };
      });
      if (fraction >= 1) state.status = 'stanced';
    }
  }

  /**
   * Moves the footprint, stopped by the walls of the room.
   * @param {number} dt
   */
  _integrate(dt) {
    // Not moving: the robot stays blocked if it was (it still pushes against the wall).
    if (!this.isWalking()) return;
    const room = this.robot.config.world.room;
    let x = this.footprint.x + this.velocity.x * dt;
    let y = this.footprint.y + this.velocity.y * dt;
    const minX = room.minX + WALL_MARGIN;
    const maxX = room.maxX - WALL_MARGIN;
    const minY = room.minY + WALL_MARGIN;
    const maxY = room.maxY - WALL_MARGIN;
    this.blocked = false;
    if (x < minX || x > maxX) {
      x = clamp(x, minX, maxX);
      this.velocity.x = 0;
      this.blocked = true;
    }
    if (y < minY || y > maxY) {
      y = clamp(y, minY, maxY);
      this.velocity.y = 0;
      this.blocked = true;
    }
    const step = Math.hypot(x - this.footprint.x, y - this.footprint.y);
    this.footprint.x = x;
    this.footprint.y = y;
    this.footprint.yaw = wrapAngle(this.footprint.yaw + this.velocity.yaw * dt);
    this.distanceWalked += step;
    // The odometry drifts slowly (a few centimeters per meter), the vision frame does not.
    const drift = this.config.odomDriftPerMeter;
    this.odomDrift.x += drift * step * Math.cos(this.footprint.yaw + 1);
    this.odomDrift.y += drift * step * Math.sin(this.footprint.yaw + 1);
    this.odomDrift.yaw = wrapAngle(this.odomDrift.yaw + drift * 0.2 * (step + Math.abs(this.velocity.yaw * dt) * 0.3));
    this.robot.emit('moved', step);
  }

  _updateOffsets(dt, motorsOn) {
    if (!motorsOn) return;
    if (!this.isStanding() && this.posture !== Posture.STANDING_UP) {
      this.offsetTarget = { ...this.offsetTarget, x: 0, y: 0, yaw: 0, roll: 0, pitch: 0 };
    }
    const linear = 0.4 * dt;
    const angular = 1.0 * dt;
    this.offset.x = approach(this.offset.x, this.offsetTarget.x, linear);
    this.offset.y = approach(this.offset.y, this.offsetTarget.y, linear);
    this.offset.height = approach(this.offset.height, this.offsetTarget.height, linear);
    this.offset.yaw = approach(this.offset.yaw, this.offsetTarget.yaw, angular);
    this.offset.roll = approach(this.offset.roll, this.offsetTarget.roll, angular);
    this.offset.pitch = approach(this.offset.pitch, this.offsetTarget.pitch, angular);
  }

  /** @returns {boolean} The body offset has reached its target. */
  offsetSettled() {
    return ['x', 'y', 'height', 'yaw', 'roll', 'pitch'].every(
      key => Math.abs(this.offset[key] - this.offsetTarget[key]) < 1e-3,
    );
  }

  /**
   * The motor power was cut: a standing robot collapses on its belly.
   */
  onMotorPowerCut() {
    if ([Posture.STANDING, Posture.STANDING_UP, Posture.SITTING_DOWN].includes(this.posture)) {
      logger.warn('The motor power was cut while standing: the robot collapses');
      this.posture = Posture.SITTING;
      this.transition = null;
    }
    this.velocity = { x: 0, y: 0, yaw: 0 };
    this.offset = { ...this.offset, x: 0, y: 0, yaw: 0, roll: 0, pitch: 0 };
    this.idle();
  }

  /**
   * The robot falls on its side (console of the simulator): a behavior fault, the robot must self-right.
   * @param {number} [side=1] 1 for the left side, -1 for the right side.
   */
  fall(side = 1) {
    this.posture = Posture.FALLEN;
    this.rollSide = side;
    this.transition = null;
    this.velocity = { x: 0, y: 0, yaw: 0 };
    this.idle();
    this.robot.faults.addBehaviorFault(robotStatePb.BehaviorFault.Cause.CAUSE_FALL, true);
    this.robot.commands.onRobotFell();
    logger.warn('The robot fell');
  }

  /**
   * Moves the robot (console of the simulator, docking): the odometry does not see the move.
   * @param {number} x
   * @param {number} y
   * @param {number} yaw
   */
  teleport(x, y, yaw) {
    this.footprint = { x, y, yaw: wrapAngle(yaw) };
  }

  // Legs.

  /**
   * The joint angles and the feet of the legs.
   * @returns {{joints: Object<string, number>, feet: {name: string, position: {x: number, y: number, z: number},
   *   contact: boolean}[]}}
   */
  legState() {
    const worldTbody = this.bodyPoseWorld();
    const bodyTworld = poseInv(worldTbody);
    const footprintWorld = this.frameInWorld('footprint');
    const lying = [Posture.FALLEN, Posture.ROLLED_OVER, Posture.ROLLING_OVER, Posture.SELF_RIGHTING].includes(
      this.posture,
    );
    const joints = {};
    const feet = [];
    for (const leg of LEGS) {
      const stance = this.stanceOffsets[leg.name];
      // The swing of the feet while walking: diagonal pairs.
      const pairPhase = leg.name === 'fl' || leg.name === 'hr' ? this.gaitPhase : (this.gaitPhase + 0.5) % 1;
      const swinging = this.isWalking() && pairPhase < 0.5;
      const lift = swinging ? 0.08 * Math.sin(pairPhase * 2 * Math.PI) : 0;
      const footWorld = transformPoint(footprintWorld, {
        x: leg.x + stance.x,
        y: leg.side * FOOT_Y + stance.y,
        z: lift,
      });
      let footBody = transformPoint(bodyTworld, footWorld);
      if (lying) footBody = { x: leg.x * 1.1, y: leg.side * (FOOT_Y + 0.25), z: -0.2 };
      const ik = legIK({ x: footBody.x - leg.x, y: footBody.y - leg.side * HIP_Y, z: footBody.z }, leg.side);
      joints[`${leg.name}.hx`] = ik.hx;
      joints[`${leg.name}.hy`] = ik.hy;
      joints[`${leg.name}.kn`] = ik.kn;
      feet.push({ name: leg.name, position: footBody, contact: !lying && !swinging });
    }
    return { joints, feet };
  }

  /**
   * Joint states of the legs, with velocities from the previous call and loads.
   * @param {number} now
   * @returns {robotStatePb.JointState[]}
   */
  legJointStates(now) {
    const { joints, feet } = this.legState();
    const previous = this._legState;
    const dt = previous ? now - previous.time : 0;
    this._legState = { time: now, joints };
    const motorsOn = this.robot.power.motorsOn();
    const contacts = Object.fromEntries(feet.map(foot => [foot.name, foot.contact]));
    return LEG_JOINTS.map(name => {
      const velocity = previous && dt > 1e-3 ? (joints[name] - previous.joints[name]) / dt : 0;
      const legName = name.split('.')[0];
      let load = 0;
      if (motorsOn && contacts[legName] && !this.isResting()) {
        if (name.endsWith('.kn')) load = -18 - 4 * Math.abs(joints[name] + 1.6);
        else if (name.endsWith('.hy')) load = 6;
        else load = LEGS.find(leg => leg.name === legName).side * 1.5;
      }
      return new robotStatePb.JointState()
        .setName(name)
        .setPosition(new DoubleValue().setValue(joints[name]))
        .setVelocity(new DoubleValue().setValue(velocity))
        .setAcceleration(new DoubleValue().setValue(0))
        .setLoad(new DoubleValue().setValue(load));
    });
  }

  /**
   * @returns {robotStatePb.FootState[]}
   */
  footStatesToProto() {
    const { feet } = this.legState();
    const { FootState } = robotStatePb;
    return feet.map(foot =>
      new FootState()
        .setFootPositionRtBody(vec3ToProto(foot.position))
        .setContact(foot.contact ? FootState.Contact.CONTACT_MADE : FootState.Contact.CONTACT_LOST)
        .setTerrain(
          new FootState.TerrainState()
            .setGroundMuEst(foot.contact ? 0.8 : 0)
            .setFrameName('odom')
            .setFootSlipDistanceRtFrame(vec3ToProto({ x: 0, y: 0, z: 0 }))
            .setFootSlipVelocityRtFrame(vec3ToProto({ x: 0, y: 0, z: 0 }))
            .setGroundContactNormalRtFrame(vec3ToProto({ x: 0, y: 0, z: 1 }))
            .setVisualSurfaceGroundPenetrationMean(0)
            .setVisualSurfaceGroundPenetrationStd(0.005),
        ),
    );
  }

  /**
   * The velocity of the body, in the vision or odom frame.
   * @param {string} frame
   * @returns {import('../bosdyn/api/geometry_pb').SE3Velocity}
   */
  velocityInFrame(frame) {
    const frameYaw = yawOf(this.frameInWorld(frame).rot);
    const c = Math.cos(-frameYaw);
    const s = Math.sin(-frameYaw);
    return velocityToProto(
      { x: c * this.velocity.x - s * this.velocity.y, y: s * this.velocity.x + c * this.velocity.y, z: 0 },
      { x: 0, y: 0, z: this.velocity.yaw },
    );
  }

  /**
   * The behavior state of the robot state.
   * @returns {number} A BehaviorState.State.
   */
  behaviorStateValue() {
    const { State } = robotStatePb.BehaviorState;
    if (!this.robot.power.motorsOn() || this.isResting()) return State.STATE_NOT_READY;
    if (this.transition) return State.STATE_TRANSITION;
    return this.isWalking() ? State.STATE_STEPPING : State.STATE_STANDING;
  }

  toJSON() {
    return {
      footprint: { ...this.footprint },
      posture: this.isResting() ? this.posture : Posture.SITTING,
      rollSide: this.rollSide ?? 1,
    };
  }

  loadFromJSON(json) {
    const footprint = json?.footprint;
    if (footprint && [footprint.x, footprint.y, footprint.yaw].every(Number.isFinite)) {
      this.footprint = { ...footprint };
    }
    if (Object.values(Posture).includes(json?.posture)) this.posture = json.posture;
    if (json?.rollSide === 1 || json?.rollSide === -1) this.rollSide = json.rollSide;
    this.boot();
  }

  /** @returns {string} */
  describe() {
    const vision = poseMul(poseInv(this.visionInWorld), this.bodyPoseWorld());
    const position = [vision.x, vision.y, vision.z].map(value => value.toFixed(2)).join(', ');
    return [
      `${this.posture}${this.isWalking() ? ', walking' : ''}`,
      `body at (${position}) yaw ${yawOf(vision.rot).toFixed(2)} in vision`,
      `behavior "${this.behavior.kind}"`,
    ].join(', ');
  }
}

module.exports = { Body, FOOT_Y, HIP_X, LEGS, LEG_JOINTS, Posture, legIK };
