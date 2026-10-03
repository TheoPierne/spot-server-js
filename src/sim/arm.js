'use strict';

const { DoubleValue } = require('google-protobuf/google/protobuf/wrappers_pb');

const {
  approach,
  clamp,
  eulerZXYFromQuat,
  pose,
  poseInv,
  poseMul,
  quatFromAxisAngle,
  quatFromEulerZXY,
  quatMul,
  quatRotate,
  velocityToProto,
  vec3ToProto,
} = require('./math');
const robotStatePb = require('../bosdyn/api/robot_state_pb');

// Kinematics of the arm (approximations of the Spot Arm): shoulder in the body frame, link lengths, and the hand
// frame relative to the wrist (arm0.link_wr1).
const SHOULDER = { x: 0.292, y: 0, z: 0.188 };
const UPPER_ARM = 0.3385;
const FOREARM = 0.4033;
const WRIST_TO_HAND = 0.19557;

const ARM_JOINTS = ['sh0', 'sh1', 'hr0', 'el0', 'el1', 'wr0', 'wr1'];
const JOINT_LIMITS = {
  sh0: [-2.618, 3.141],
  sh1: [-3.141, 0.523],
  hr0: [-1e-3, 1e-3],
  el0: [0, 3.141],
  el1: [-2.792, 2.792],
  wr0: [-1.832, 1.832],
  wr1: [-2.879, 2.879],
};

const NAMED_POSITIONS = {
  stow: { sh0: 0, sh1: -3.115, hr0: 0, el0: 3.13, el1: 0, wr0: 0, wr1: 0 },
  ready: { sh0: 0, sh1: -1.6, hr0: 0, el0: 2.4, el1: 0, wr0: -0.8, wr1: 0 },
  carry: { sh0: 0, sh1: -2.2, hr0: 0, el0: 2.7, el1: 0, wr0: 0.2, wr1: 0 },
};

// Gripper angle: 0 closed, -1.5708 fully open.
const GRIPPER_OPEN = -1.5708;
const JOINT_SPEED = 1.8;
const GRIPPER_SPEED = 2.5;

/**
 * Forward kinematics: the hand and the wrist in the body frame.
 * @param {Object<string, number>} q
 * @returns {{hand: import('./math').Pose, wrist: import('./math').Pose}}
 */
function forwardKinematics(q) {
  const theta2 = q.sh1 + q.el0;
  const theta3 = theta2 + q.wr0;
  const r = UPPER_ARM * Math.cos(q.sh1) + FOREARM * Math.cos(theta2);
  const z = -UPPER_ARM * Math.sin(q.sh1) - FOREARM * Math.sin(theta2);
  const c = Math.cos(q.sh0);
  const s = Math.sin(q.sh0);
  const roll = q.el1 + q.wr1;
  const rot = quatMul(
    quatMul(quatFromAxisAngle({ x: 0, y: 0, z: 1 }, q.sh0), quatFromAxisAngle({ x: 0, y: 1, z: 0 }, theta3)),
    quatFromAxisAngle({ x: 1, y: 0, z: 0 }, roll),
  );
  const wrist = pose(SHOULDER.x + c * r, SHOULDER.y + s * r, SHOULDER.z + z, rot);
  const tip = quatRotate(rot, { x: WRIST_TO_HAND, y: 0, z: 0 });
  const hand = pose(wrist.x + tip.x, wrist.y + tip.y, wrist.z + tip.z, rot);
  return { hand, wrist };
}

/**
 * Inverse kinematics of a hand pose in the body frame (elbow up). The yaw of the hand follows the shoulder.
 * @param {import('./math').Pose} hand
 * @returns {{q: Object<string, number>, reachable: boolean}}
 */
function inverseKinematics(hand) {
  const euler = eulerZXYFromQuat(hand.rot);
  // Pitch of the hand (positive down) and its roll.
  const handX = quatRotate(hand.rot, { x: 1, y: 0, z: 0 });
  const theta3 = Math.atan2(-handX.z, Math.hypot(handX.x, handX.y));
  const sh0 = Math.atan2(hand.y - SHOULDER.y, hand.x - SHOULDER.x);
  const r = Math.hypot(hand.x - SHOULDER.x, hand.y - SHOULDER.y) - WRIST_TO_HAND * Math.cos(theta3);
  const z = hand.z - SHOULDER.z + WRIST_TO_HAND * Math.sin(theta3);
  let distance = Math.hypot(r, z);
  const reachable = distance <= UPPER_ARM + FOREARM - 1e-3 && distance >= FOREARM - UPPER_ARM + 1e-3;
  distance = clamp(distance, FOREARM - UPPER_ARM + 1e-3, UPPER_ARM + FOREARM - 1e-3);
  const phi = Math.atan2(-z, r);
  const a = Math.acos(clamp((UPPER_ARM ** 2 + distance ** 2 - FOREARM ** 2) / (2 * UPPER_ARM * distance), -1, 1));
  const gamma = Math.acos(clamp((UPPER_ARM ** 2 + FOREARM ** 2 - distance ** 2) / (2 * UPPER_ARM * FOREARM), -1, 1));
  const sh1 = phi - a;
  const el0 = Math.PI - gamma;
  const q = { sh0, sh1, hr0: 0, el0, el1: 0, wr0: theta3 - (sh1 + el0), wr1: euler.roll };
  let withinLimits = true;
  for (const joint of ARM_JOINTS) {
    const [min, max] = JOINT_LIMITS[joint];
    if (q[joint] < min - 1e-6 || q[joint] > max + 1e-6) withinLimits = false;
    q[joint] = clamp(q[joint], min, max);
  }
  return { q, reachable: reachable && withinLimits };
}

/**
 * Distance between two joint configurations.
 * @param {Object<string, number>} a
 * @param {Object<string, number>} b
 * @returns {number}
 */
function jointDistance(a, b) {
  return Math.max(...ARM_JOINTS.map(joint => Math.abs(a[joint] - b[joint])));
}

/**
 * The arm and the gripper of the robot.
 */
class Arm {
  /**
   * @param {import('../robot')} robot
   */
  constructor(robot) {
    this.robot = robot;
    this.q = { ...NAMED_POSITIONS.stow };
    this.gripper = 0;
    this.gripperTarget = 0;
    this.gripperSpeed = GRIPPER_SPEED;
    this.holdingItem = false;
    this.behavior = { kind: 'stow', owner: null, state: { status: 'complete' } };
    this.gripperBehavior = { owner: null, state: { status: 'at_goal' } };
    this._handHistory = null;
  }

  /** Arm stowed (boot of the robot). */
  boot() {
    this.behavior = { kind: 'stow', owner: null, state: { status: 'complete' } };
    this.gripperBehavior = { owner: null, state: { status: 'at_goal' } };
  }

  /** @returns {boolean} */
  isStowed() {
    return jointDistance(this.q, NAMED_POSITIONS.stow) < 0.05;
  }

  /** @returns {number} Opening of the gripper, in percent. */
  gripperOpenPercentage() {
    return clamp((100 * this.gripper) / GRIPPER_OPEN, 0, 100);
  }

  /**
   * Starts an arm behavior (the previous one is overridden).
   * @param {object} behavior
   */
  start(behavior) {
    this.behavior = { state: {}, ...behavior, startTime: this.robot.clock.now(), from: { ...this.q } };
    if (behavior.kind === 'stow' && this.holdingItem) this.behavior.state.status = 'stalled_holding_item';
  }

  /**
   * Starts a gripper motion.
   * @param {number} target Angle of the gripper.
   * @param {?number} maxVelocity
   * @param {any} owner
   */
  startGripper(target, maxVelocity, owner) {
    this.gripperTarget = clamp(target, GRIPPER_OPEN, 0);
    this.gripperSpeed = maxVelocity > 0 ? Math.min(maxVelocity, 5) : GRIPPER_SPEED;
    this.gripperBehavior = { owner, state: { status: 'in_progress' } };
  }

  /**
   * The target of the joints for the current behavior.
   * @param {number} now
   * @returns {?Object<string, number>} Null to hold the current position.
   */
  _targetJoints(now) {
    const behavior = this.behavior;
    const state = behavior.state;
    switch (behavior.kind) {
      case 'stow':
        return state.status === 'stalled_holding_item' ? null : NAMED_POSITIONS.stow;
      case 'ready':
      case 'carry':
        return NAMED_POSITIONS[behavior.kind];
      case 'joint_move': {
        // Linear interpolation between the points of the trajectory.
        const elapsed = now - behavior.referenceTime;
        const points = behavior.points;
        let previous = { t: 0, q: behavior.from };
        for (const point of points) {
          if (elapsed < point.t) {
            const fraction = clamp((elapsed - previous.t) / Math.max(point.t - previous.t, 1e-6), 0, 1);
            return Object.fromEntries(
              ARM_JOINTS.map(j => [j, previous.q[j] + (point.q[j] - previous.q[j]) * fraction]),
            );
          }
          previous = point;
        }
        return points[points.length - 1].q;
      }
      case 'cartesian':
      case 'gaze':
      case 'impedance': {
        const handTarget = this._handTargetInBody(now);
        if (!handTarget) return null;
        const { q, reachable } = inverseKinematics(handTarget);
        state.reachable = reachable;
        return q;
      }
      case 'velocity': {
        if (now > behavior.endTime) return null;
        const { hand } = forwardKinematics(this.q);
        const bodyTworld = poseInv(this.robot.body.bodyPoseWorld());
        const velocityBody = quatRotate(bodyTworld.rot, behavior.velocityWorld(this));
        const dt = 0.1;
        const target = pose(
          hand.x + velocityBody.x * dt,
          hand.y + velocityBody.y * dt,
          hand.z + velocityBody.z * dt,
          hand.rot,
        );
        return inverseKinematics(target).q;
      }
      default:
        return null;
    }
  }

  /**
   * The hand target of a Cartesian behavior, in the body frame (interpolated along the trajectory).
   * @param {number} now
   * @returns {?import('./math').Pose}
   */
  _handTargetInBody(now) {
    const behavior = this.behavior;
    const elapsed = now - behavior.referenceTime;
    const points = behavior.points;
    let target = points[points.length - 1].pose;
    let previous = { t: 0, pose: behavior.startHandRoot };
    for (const point of points) {
      if (elapsed < point.t) {
        const fraction = clamp((elapsed - previous.t) / Math.max(point.t - previous.t, 1e-6), 0, 1);
        const a = previous.pose;
        const b = point.pose;
        const ea = eulerZXYFromQuat(a.rot);
        const eb = eulerZXYFromQuat(b.rot);
        target = pose(
          a.x + (b.x - a.x) * fraction,
          a.y + (b.y - a.y) * fraction,
          a.z + (b.z - a.z) * fraction,
          quatFromEulerZXY(
            ea.yaw + (eb.yaw - ea.yaw) * fraction,
            ea.roll + (eb.roll - ea.roll) * fraction,
            ea.pitch + (eb.pitch - ea.pitch) * fraction,
          ),
        );
        break;
      }
      previous = point;
    }
    behavior.state.trajectoryDone = elapsed >= points[points.length - 1].t;
    const rootWorld = this.robot.body.frameInWorld(behavior.rootFrame);
    if (!rootWorld) return null;
    const handWorld = poseMul(rootWorld, target);
    return poseMul(poseInv(this.robot.body.bodyPoseWorld()), handWorld);
  }

  /**
   * @param {number} now
   * @param {number} dt
   */
  update(now, dt) {
    if (!this.robot.power.motorsOn()) return;
    const target = this._targetJoints(now);
    const behavior = this.behavior;
    if (target) {
      const step = JOINT_SPEED * dt;
      for (const joint of ARM_JOINTS) this.q[joint] = approach(this.q[joint], target[joint], step);
    }
    const reached = target ? jointDistance(this.q, target) < 0.01 : true;
    const state = behavior.state;
    switch (behavior.kind) {
      case 'stow':
      case 'ready':
      case 'carry':
        if (state.status !== 'stalled_holding_item') state.status = reached ? 'complete' : 'in_progress';
        break;
      case 'joint_move':
        state.status =
          reached && now - behavior.referenceTime >= behavior.points[behavior.points.length - 1].t
            ? 'complete'
            : 'in_progress';
        break;
      case 'cartesian':
      case 'gaze':
      case 'impedance':
        if (state.reachable === false) state.status = 'stalled';
        else state.status = reached && state.trajectoryDone ? 'complete' : 'in_progress';
        break;
      default:
        break;
    }

    // The gripper stops on a held item.
    const gripperState = this.gripperBehavior.state;
    const stopAngle = this.holdingItem ? Math.min(this.gripperTarget, -0.35) : this.gripperTarget;
    const blocked = this.holdingItem && this.gripperTarget > -0.35;
    this.gripper = approach(this.gripper, blocked ? stopAngle : this.gripperTarget, this.gripperSpeed * dt);
    if (Math.abs(this.gripper - (blocked ? stopAngle : this.gripperTarget)) < 1e-3) {
      gripperState.status = blocked ? 'applying_force' : 'at_goal';
    } else {
      gripperState.status = 'in_progress';
    }
  }

  /**
   * The pose of a frame of the arm in the world.
   * @param {string} frame 'hand' or 'arm0.link_wr1'.
   * @returns {?import('./math').Pose}
   */
  frameInWorld(frame) {
    const { hand, wrist } = forwardKinematics(this.q);
    const body = this.robot.body.bodyPoseWorld();
    if (frame === 'hand') return poseMul(body, hand);
    if (frame === 'arm0.link_wr1' || frame === 'link_wr1') return poseMul(body, wrist);
    return null;
  }

  /**
   * @returns {Object<string, [string, import('./math').Pose]>}
   */
  frameTreeEdges() {
    const { hand, wrist } = forwardKinematics(this.q);
    return { hand: ['body', hand], 'arm0.link_wr1': ['body', wrist] };
  }

  /**
   * @returns {robotStatePb.JointState[]}
   */
  jointStates() {
    const motorsOn = this.robot.power.motorsOn();
    const states = ARM_JOINTS.map(joint =>
      new robotStatePb.JointState()
        .setName(`arm0.${joint}`)
        .setPosition(new DoubleValue().setValue(this.q[joint]))
        .setVelocity(new DoubleValue().setValue(0))
        .setAcceleration(new DoubleValue().setValue(0))
        .setLoad(new DoubleValue().setValue(motorsOn && !this.isStowed() && joint === 'sh1' ? -8 : 0)),
    );
    states.push(
      new robotStatePb.JointState()
        .setName('arm0.f1x')
        .setPosition(new DoubleValue().setValue(this.gripper))
        .setVelocity(new DoubleValue().setValue(0))
        .setAcceleration(new DoubleValue().setValue(0))
        .setLoad(
          new DoubleValue().setValue(
            this.holdingItem && this.gripperBehavior.state.status === 'applying_force' ? 2 : 0,
          ),
        ),
    );
    return states;
  }

  /**
   * @param {number} now
   * @returns {robotStatePb.ManipulatorState}
   */
  manipulatorStateToProto(now) {
    const { ManipulatorState } = robotStatePb;
    const handWorld = this.frameInWorld('hand');
    let linear = { x: 0, y: 0, z: 0 };
    if (this._handHistory && now - this._handHistory.time > 1e-3) {
      const dt = now - this._handHistory.time;
      linear = {
        x: (handWorld.x - this._handHistory.pose.x) / dt,
        y: (handWorld.y - this._handHistory.pose.y) / dt,
        z: (handWorld.z - this._handHistory.pose.z) / dt,
      };
    }
    this._handHistory = { time: now, pose: handWorld };
    const toFrame = frame => {
      const rot = this.robot.body.frameInWorld(frame).rot;
      const inv = { w: rot.w, x: -rot.x, y: -rot.y, z: -rot.z };
      return velocityToProto(quatRotate(inv, linear), { x: 0, y: 0, z: 0 });
    };
    return new ManipulatorState()
      .setGripperOpenPercentage(this.gripperOpenPercentage())
      .setIsGripperHoldingItem(this.holdingItem)
      .setEstimatedEndEffectorForceInHand(vec3ToProto({ x: 0, y: 0, z: this.holdingItem ? -2.5 : 0 }))
      .setStowState(
        this.isStowed() ? ManipulatorState.StowState.STOWSTATE_STOWED : ManipulatorState.StowState.STOWSTATE_DEPLOYED,
      )
      .setVelocityOfHandInVision(toFrame('vision'))
      .setVelocityOfHandInOdom(toFrame('odom'))
      .setCarryState(
        this.holdingItem
          ? ManipulatorState.CarryState.CARRY_STATE_CARRIABLE
          : ManipulatorState.CarryState.CARRY_STATE_CARRIABLE_AND_STOWABLE,
      );
  }

  /** @returns {string} */
  describe() {
    const { hand } = forwardKinematics(this.q);
    return (
      `${this.isStowed() ? 'stowed' : 'deployed'} (behavior "${this.behavior.kind}"), hand at ` +
      `(${hand.x.toFixed(2)}, ${hand.y.toFixed(2)}, ${hand.z.toFixed(2)}) in body, gripper ` +
      `${this.gripperOpenPercentage().toFixed(0)} % open${this.holdingItem ? ', holding an item' : ''}`
    );
  }
}

module.exports = {
  ARM_JOINTS,
  JOINT_LIMITS,
  Arm,
  GRIPPER_OPEN,
  NAMED_POSITIONS,
  WRIST_TO_HAND,
  forwardKinematics,
  inverseKinematics,
};
