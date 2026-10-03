'use strict';

const { JOINT_LIMITS, WRIST_TO_HAND } = require('./arm');
const { durationToSec, secToDuration, timestampToSec } = require('./clock');
const { leaseFromProto, LeaseManager } = require('./lease');
const { pose, poseFromProto, poseInv, poseMul, quatFromEulerZXY, quatRotate } = require('./math');
const armCommandPb = require('../bosdyn/api/arm_command_pb');
const basicCommandPb = require('../bosdyn/api/basic_command_pb');
const fullBodyCommandPb = require('../bosdyn/api/full_body_command_pb');
const gripperCommandPb = require('../bosdyn/api/gripper_command_pb');
const leasePb = require('../bosdyn/api/lease_pb');
const mobilityCommandPb = require('../bosdyn/api/mobility_command_pb');
const payloadEstimationPb = require('../bosdyn/api/payload_estimation_pb');
const robotCommandPb = require('../bosdyn/api/robot_command_pb');
const spotCommandPb = require('../bosdyn/api/spot/robot_command_pb');
const synchronizedCommandPb = require('../bosdyn/api/synchronized_command_pb');
const { LoggerUtil } = require('../loggerUtil');

const logger = LoggerUtil.getLogger('COMMAND');

const Status = robotCommandPb.RobotCommandResponse.Status;
const FeedbackStatus = basicCommandPb.RobotCommandFeedbackStatus.Status;
const MobilityCase = mobilityCommandPb.MobilityCommand.Request.CommandCase;
const FullBodyCase = fullBodyCommandPb.FullBodyCommand.Request.CommandCase;
const ArmCase = armCommandPb.ArmCommand.Request.CommandCase;

// Commands kept for their feedback.
const MAX_COMMANDS = 200;

const INERTIAL_FRAMES = ['vision', 'odom'];
const KNOWN_FRAMES = ['vision', 'odom', 'body', 'flat_body', 'gpe', 'hand', 'arm0.link_wr1', 'feet_center'];

/** A rejected command. */
class CommandError extends Error {
  /**
   * @param {number} status A RobotCommandResponse.Status.
   * @param {string} message
   */
  constructor(status, message) {
    super(message);
    this.status = status;
  }
}

/**
 * @param {string} frame
 * @param {string[]} allowed
 * @param {string} what
 */
function checkFrame(frame, allowed, what) {
  if (!KNOWN_FRAMES.includes(frame)) {
    throw new CommandError(Status.STATUS_UNKNOWN_FRAME, `Unknown frame "${frame}" for ${what}.`);
  }
  if (!allowed.includes(frame)) {
    throw new CommandError(
      Status.STATUS_INVALID_REQUEST,
      `The frame of ${what} must be one of: ${allowed.join(', ')}.`,
    );
  }
}

/**
 * @param {?import('google-protobuf/google/protobuf/wrappers_pb').DoubleValue} value
 * @returns {?number}
 */
function doubleValue(value) {
  return value ? value.getValue() : null;
}

/**
 * The robot command service: validation of the commands (lease, time sync, end times, motor power, behavior faults,
 * dock, frames), the commands that override each other (mobility, arm, gripper, full body), and their feedback.
 */
class CommandManager {
  /**
   * @param {import('../robot')} robot
   */
  constructor(robot) {
    this.robot = robot;
    this.nextId = 1 + Math.floor(Math.random() * 1000);
    /** @type {Map<number, object>} */
    this.commands = new Map();
  }

  /** Forgets the commands (reboot). */
  reset() {
    this.commands.clear();
  }

  /**
   * RobotCommand.
   * @param {robotCommandPb.RobotCommandRequest} request
   * @param {string} clientName
   * @returns {{status: number, message: string, id: number, leaseUseResult: ?leasePb.LeaseUseResult}}
   */
  submit(request, clientName) {
    const now = this.robot.clock.now();
    const command = request.getCommand();
    let parsed;
    try {
      if (!command || command.getCommandCase() === robotCommandPb.RobotCommand.CommandCase.COMMAND_NOT_SET) {
        throw new CommandError(Status.STATUS_INVALID_REQUEST, 'The command is empty.');
      }
      parsed = this._parse(command, now);
    } catch (err) {
      if (err instanceof CommandError) return { status: err.status, message: err.message, id: 0, leaseUseResult: null };
      throw err;
    }

    const lease = leaseFromProto(request.getLease());
    const leaseCheck = this.robot.leases.use(lease, parsed.resources, { record: false });
    if (leaseCheck.status !== leasePb.LeaseUseResult.Status.STATUS_OK) {
      return {
        status: Status.STATUS_UNKNOWN,
        message: 'The lease was rejected.',
        id: 0,
        leaseUseResult: LeaseManager.resultToProto(leaseCheck),
      };
    }
    const reject = (status, message) => ({
      status,
      message,
      id: 0,
      leaseUseResult: LeaseManager.resultToProto(leaseCheck),
    });

    for (const endTime of parsed.endTimes) {
      if (!this.robot.timeSync.isSynced(request.getClockIdentifier())) {
        return reject(Status.STATUS_NO_TIMESYNC, 'Time sync with the robot is required for commands with an end time.');
      }
      if (endTime < now) {
        return reject(Status.STATUS_EXPIRED, `The end time of the command passed ${(now - endTime).toFixed(3)} s ago.`);
      }
      if (endTime > now + this.robot.config.mobility.maxCommandDurationSec) {
        return reject(
          Status.STATUS_TOO_DISTANT,
          `The end time of the command is ${(endTime - now).toFixed(1)} s away.`,
        );
      }
    }
    if (!parsed.allowedUnpowered && !this.robot.power.motorsOn()) {
      return reject(Status.STATUS_NOT_POWERED_ON, 'The robot must be powered on to accept a command.');
    }
    if (!parsed.allowedWithFaults && this.robot.faults.hasBehaviorFaults()) {
      return reject(Status.STATUS_BEHAVIOR_FAULT, 'The robot has uncleared behavior faults.');
    }
    if (parsed.refusedDocked && this.robot.docking.isDocked()) {
      return reject(Status.STATUS_DOCKED, 'The robot cannot execute this command on the dock.');
    }

    const leaseResult = this.robot.leases.use(lease, parsed.resources, { record: true });
    const id = this.nextId++;
    const record = { id, clientName, kind: parsed.kind, parts: parsed.parts, created: now };
    this.commands.set(id, record);
    if (this.commands.size > MAX_COMMANDS) this.commands.delete(this.commands.keys().next().value);
    this._start(record);
    logger.info(
      `Command ${id} from "${clientName}": ${Object.values(parsed.parts)
        .map(part => part.type)
        .join(' + ')}`,
    );
    return { status: Status.STATUS_OK, message: '', id, leaseUseResult: LeaseManager.resultToProto(leaseResult) };
  }

  /**
   * Starts the behaviors of an accepted command.
   * @param {object} record
   */
  _start(record) {
    const { body, arm, docking, autoReturn } = this.robot;
    const { parts } = record;
    if (parts.mobility || parts.fullBody) {
      const part = parts.mobility ?? parts.fullBody;
      docking.onMobilityOverridden();
      autoReturn.onMobilityOverridden();
      body.start({ ...part.behavior, owner: record.id });
    }
    if (arm && parts.fullBody) {
      // The full body commands control the arm too: stopped, or stowed before sitting down.
      const stow = ['safe_power_off', 'battery_change'].includes(parts.fullBody.type);
      arm.start({ kind: stow ? 'stow' : 'hold', owner: record.id });
    }
    if (arm && parts.arm && !parts.arm.incompatible) {
      arm.start({ ...parts.arm.behavior, owner: record.id });
    }
    if (arm && parts.gripper && !parts.gripper.incompatible) {
      arm.startGripper(parts.gripper.target, parts.gripper.maxVelocity, record.id);
    }
  }

  /**
   * @param {robotCommandPb.RobotCommand} command
   * @param {number} now
   * @returns {{kind: string, parts: object, resources: string[], endTimes: number[], allowedUnpowered: boolean,
   *   allowedWithFaults: boolean, refusedDocked: boolean}}
   */
  _parse(command, now) {
    const parsed = {
      kind: '',
      parts: {},
      resources: [],
      endTimes: [],
      allowedUnpowered: false,
      allowedWithFaults: false,
      refusedDocked: false,
    };
    if (command.hasFullBodyCommand()) {
      parsed.kind = 'full_body';
      parsed.resources = ['body'];
      parsed.parts.fullBody = this._parseFullBody(command.getFullBodyCommand(), parsed);
      return parsed;
    }
    parsed.kind = 'synchronized';
    const synchronized = command.getSynchronizedCommand();
    if (!synchronized.hasMobilityCommand() && !synchronized.hasArmCommand() && !synchronized.hasGripperCommand()) {
      throw new CommandError(Status.STATUS_INVALID_REQUEST, 'The synchronized command is empty.');
    }
    if (synchronized.hasMobilityCommand()) {
      parsed.resources.push('mobility');
      parsed.parts.mobility = this._parseMobility(synchronized.getMobilityCommand(), now, parsed);
    }
    if (synchronized.hasArmCommand()) {
      parsed.resources.push('arm');
      parsed.parts.arm = this.robot.arm
        ? this._parseArm(synchronized.getArmCommand(), now, parsed)
        : { type: 'arm', incompatible: true };
    }
    if (synchronized.hasGripperCommand()) {
      parsed.resources.push('gripper');
      parsed.parts.gripper = this.robot.arm
        ? this._parseGripper(synchronized.getGripperCommand())
        : { type: 'gripper', incompatible: true };
    }
    return parsed;
  }

  /**
   * MobilityParams of a mobility command: velocity limits and body control.
   * @param {?import('google-protobuf/google/protobuf/any_pb').Any} any
   * @param {number} now
   * @returns {{maxVel: ?{x: number, y: number, yaw: number}, bodyControl: object}}
   */
  _mobilityParams(any, now) {
    const nominal = { points: [{ t: 0, pose: pose() }], rootFrame: null, referenceTime: now };
    if (!any || !any.getTypeUrl()) return { maxVel: null, bodyControl: nominal };
    let params;
    try {
      params = any.unpack(spotCommandPb.MobilityParams.deserializeBinary, 'bosdyn.api.spot.MobilityParams');
    } catch {
      params = null;
    }
    if (!params) {
      throw new CommandError(
        Status.STATUS_INVALID_REQUEST,
        'The mobility params are not bosdyn.api.spot.MobilityParams.',
      );
    }
    let maxVel = null;
    const max = params.getVelLimit()?.getMaxVel();
    if (max) {
      maxVel = {
        x: Math.abs(max.getLinear()?.getX() ?? 0) || Infinity,
        y: Math.abs(max.getLinear()?.getY() ?? 0) || Infinity,
        yaw: Math.abs(max.getAngular()) || Infinity,
      };
    }
    let bodyControl = nominal;
    const control = params.getBodyControl();
    const ParamCase = spotCommandPb.BodyControlParams.ParamCase;
    if (control && control.getParamCase() === ParamCase.BASE_OFFSET_RT_FOOTPRINT) {
      bodyControl = this._se3Trajectory(control.getBaseOffsetRtFootprint(), null, now);
    } else if (control && control.getParamCase() === ParamCase.BODY_POSE) {
      const rootFrame = control.getBodyPose().getRootFrameName();
      checkFrame(rootFrame, ['odom', 'vision', 'flat_body', 'body', 'gpe'], 'the body pose');
      bodyControl = this._se3Trajectory(control.getBodyPose().getBaseOffsetRtRoot(), rootFrame, now);
    }
    return { maxVel, bodyControl };
  }

  /**
   * @param {?import('../bosdyn/api/trajectory_pb').SE3Trajectory} trajectory
   * @param {?string} rootFrame
   * @param {number} now
   * @returns {{points: {t: number, pose: import('./math').Pose}[], rootFrame: ?string, referenceTime: number}}
   */
  _se3Trajectory(trajectory, rootFrame, now) {
    const points = (trajectory?.getPointsList() ?? []).map(point => ({
      t: durationToSec(point.getTimeSinceReference()) ?? 0,
      pose: poseFromProto(point.getPose()),
    }));
    if (points.length === 0) points.push({ t: 0, pose: pose() });
    points.sort((a, b) => a.t - b.t);
    const referenceTime = trajectory?.hasReferenceTime() ? timestampToSec(trajectory.getReferenceTime()) : now;
    return { points, rootFrame, referenceTime };
  }

  _endTime(request, parsed, what) {
    if (!request.hasEndTime()) {
      throw new CommandError(Status.STATUS_INVALID_REQUEST, `The end time of ${what} is required.`);
    }
    const endTime = timestampToSec(request.getEndTime());
    parsed.endTimes.push(endTime);
    return endTime;
  }

  /**
   * @param {mobilityCommandPb.MobilityCommand.Request} request
   * @param {number} now
   * @param {object} parsed
   * @returns {object} The mobility part.
   */
  _parseMobility(request, now, parsed) {
    const { maxVel, bodyControl } = this._mobilityParams(request.getParams(), now);
    const { body } = this.robot;
    switch (request.getCommandCase()) {
      case MobilityCase.SE2_TRAJECTORY_REQUEST: {
        const req = request.getSe2TrajectoryRequest();
        const frame = req.getSe2FrameName();
        checkFrame(frame, ['vision', 'odom', 'body', 'flat_body'], 'the trajectory');
        const endTime = this._endTime(req, parsed, 'the trajectory');
        const points = (req.getTrajectory()?.getPointsList() ?? []).map(point =>
          body.planarToWorld(frame, {
            x: point.getPose()?.getPosition()?.getX() ?? 0,
            y: point.getPose()?.getPosition()?.getY() ?? 0,
            yaw: point.getPose()?.getAngle() ?? 0,
          }),
        );
        if (points.length === 0) throw new CommandError(Status.STATUS_INVALID_REQUEST, 'The trajectory has no point.');
        return {
          type: 'se2_trajectory',
          endTime,
          request: req,
          behavior: { kind: 'trajectory', points, endTime, maxVel, bodyControl },
        };
      }
      case MobilityCase.SE2_VELOCITY_REQUEST: {
        const req = request.getSe2VelocityRequest();
        const frame = req.getSe2FrameName();
        checkFrame(frame, ['vision', 'odom', 'flat_body', 'body'], 'the velocity');
        const endTime = this._endTime(req, parsed, 'the velocity');
        const velocity = {
          x: req.getVelocity()?.getLinear()?.getX() ?? 0,
          y: req.getVelocity()?.getLinear()?.getY() ?? 0,
          yaw: req.getVelocity()?.getAngular() ?? 0,
        };
        return {
          type: 'se2_velocity',
          endTime,
          request: req,
          behavior: { kind: 'velocity', velocity, frame, endTime, maxVel, bodyControl },
        };
      }
      case MobilityCase.SIT_REQUEST:
        return { type: 'sit', endTime: null, behavior: { kind: 'sit' } };
      case MobilityCase.STAND_REQUEST:
        return { type: 'stand', endTime: null, behavior: { kind: 'stand', bodyControl } };
      case MobilityCase.STANCE_REQUEST: {
        const req = request.getStanceRequest();
        const endTime = this._endTime(req, parsed, 'the stance');
        const stance = req.getStance();
        const frame = stance?.getSe2FrameName() ?? '';
        checkFrame(frame, ['vision', 'odom', 'body', 'flat_body'], 'the stance');
        const frameWorld = body.frameInWorld(frame === 'body' ? 'flat_body' : frame);
        const feet = {};
        for (const [name, position] of stance.getFootPositionsMap().entries()) {
          feet[name] = poseMul(frameWorld, pose(position.getX(), position.getY(), 0));
        }
        if (!['fl', 'fr', 'hl', 'hr'].every(name => feet[name])) {
          throw new CommandError(Status.STATUS_INVALID_REQUEST, 'The stance needs the positions of fl, fr, hl and hr.');
        }
        return { type: 'stance', endTime, behavior: { kind: 'stance', feet, bodyControl } };
      }
      case MobilityCase.STOP_REQUEST:
        return { type: 'stop', endTime: null, behavior: { kind: 'stop', bodyControl } };
      case MobilityCase.FOLLOW_ARM_REQUEST:
        return { type: 'follow_arm', endTime: null, behavior: { kind: 'follow_arm', bodyControl } };
      case MobilityCase.FREEZE_REQUEST:
        return { type: 'freeze', endTime: null, behavior: { kind: 'freeze' } };
      default:
        throw new CommandError(Status.STATUS_INVALID_REQUEST, 'The mobility command is empty.');
    }
  }

  /**
   * @param {fullBodyCommandPb.FullBodyCommand.Request} request
   * @param {object} parsed
   * @returns {object} The full body part.
   */
  _parseFullBody(request, parsed) {
    switch (request.getCommandCase()) {
      case FullBodyCase.STOP_REQUEST:
        return { type: 'stop', endTime: null, behavior: { kind: 'stop', bodyControl: null } };
      case FullBodyCase.FREEZE_REQUEST:
        return { type: 'freeze', endTime: null, behavior: { kind: 'freeze' } };
      case FullBodyCase.SELFRIGHT_REQUEST:
        parsed.allowedWithFaults = true;
        parsed.refusedDocked = true;
        return { type: 'selfright', endTime: null, behavior: { kind: 'selfright' } };
      case FullBodyCase.SAFE_POWER_OFF_REQUEST:
        parsed.allowedUnpowered = true;
        parsed.allowedWithFaults = true;
        return {
          type: 'safe_power_off',
          endTime: null,
          behavior: { kind: 'safe_power_off', reason: 'safe power off command' },
        };
      case FullBodyCase.BATTERY_CHANGE_POSE_REQUEST: {
        parsed.refusedDocked = true;
        const { DirectionHint } = basicCommandPb.BatteryChangePoseCommand.Request;
        const hint = request.getBatteryChangePoseRequest().getDirectionHint();
        return {
          type: 'battery_change_pose',
          endTime: null,
          behavior: { kind: 'battery_change', side: hint === DirectionHint.HINT_LEFT ? 1 : -1 },
        };
      }
      case FullBodyCase.PAYLOAD_ESTIMATION_REQUEST:
        return { type: 'payload_estimation', endTime: null, behavior: { kind: 'payload_estimation' } };
      case FullBodyCase.CONSTRAINED_MANIPULATION_REQUEST:
      case FullBodyCase.JOINT_REQUEST:
        throw new CommandError(Status.STATUS_UNSUPPORTED, 'The simulator does not support this command.');
      default:
        throw new CommandError(Status.STATUS_INVALID_REQUEST, 'The full body command is empty.');
    }
  }

  /**
   * A target of the hand, as hand poses in a root frame.
   * @param {string} rootFrame
   * @param {?import('../bosdyn/api/geometry_pb').SE3Pose} rootTformTask
   * @param {?import('../bosdyn/api/geometry_pb').SE3Pose} wristTformTool
   * @param {?import('../bosdyn/api/trajectory_pb').SE3Trajectory} trajectory
   * @param {number} now
   * @returns {{points: {t: number, pose: import('./math').Pose}[], referenceTime: number, rootFrame: string,
   *   startHandRoot: import('./math').Pose}}
   */
  _handTrajectory(rootFrame, rootTformTask, wristTformTool, trajectory, now) {
    const rootTtask = rootTformTask ? poseFromProto(rootTformTask) : pose();
    const toolTwrist = wristTformTool ? poseInv(poseFromProto(wristTformTool)) : poseInv(pose(WRIST_TO_HAND, 0, 0));
    const wristThand = pose(WRIST_TO_HAND, 0, 0);
    const { points, referenceTime } = this._se3Trajectory(trajectory, rootFrame, now);
    const handPoints = points.map(point => ({
      t: point.t,
      pose: poseMul(poseMul(poseMul(rootTtask, point.pose), toolTwrist), wristThand),
    }));
    const rootWorld = this.robot.body.frameInWorld(rootFrame);
    const startHandRoot = poseMul(poseInv(rootWorld), this.robot.arm.frameInWorld('hand'));
    // Without times, the hand moves at a moderate speed.
    for (const point of handPoints) {
      if (point.t <= 0) {
        const dist = Math.hypot(
          point.pose.x - startHandRoot.x,
          point.pose.y - startHandRoot.y,
          point.pose.z - startHandRoot.z,
        );
        point.t = Math.max(0.5, dist / 0.5);
      }
    }
    return { points: handPoints, referenceTime, rootFrame, startHandRoot };
  }

  /**
   * @param {armCommandPb.ArmCommand.Request} request
   * @param {number} now
   * @param {object} parsed
   * @returns {object} The arm part.
   */
  _parseArm(request, now, parsed) {
    const { arm } = this.robot;
    switch (request.getCommandCase()) {
      case ArmCase.NAMED_ARM_POSITION_COMMAND: {
        const { Positions } = armCommandPb.NamedArmPositionsCommand;
        const position = request.getNamedArmPositionCommand().getPosition();
        const kind = {
          [Positions.POSITIONS_STOW]: 'stow',
          [Positions.POSITIONS_READY]: 'ready',
          [Positions.POSITIONS_CARRY]: 'carry',
        }[position];
        if (!kind) throw new CommandError(Status.STATUS_INVALID_REQUEST, 'Unknown named arm position.');
        return { type: 'named_arm_position', behavior: { kind } };
      }
      case ArmCase.ARM_JOINT_MOVE_COMMAND: {
        const trajectory = request.getArmJointMoveCommand().getTrajectory();
        const maxVelocity = doubleValue(trajectory?.getMaximumVelocity()) || 2.5;
        let previous = { ...arm.q };
        let time = 0;
        let modified = false;
        const points = (trajectory?.getPointsList() ?? []).map(point => {
          const position = point.getPosition();
          const q = { ...previous };
          for (const joint of ['sh0', 'sh1', 'el0', 'el1', 'wr0', 'wr1']) {
            const value = doubleValue(position?.[`get${joint[0].toUpperCase()}${joint.slice(1)}`]());
            if (value !== null) q[joint] = value;
          }
          for (const [joint, [min, max]] of Object.entries(JOINT_LIMITS)) {
            if (joint === 'hr0') continue;
            if (q[joint] < min || q[joint] > max) {
              q[joint] = Math.min(max, Math.max(min, q[joint]));
              modified = true;
            }
          }
          const minTime = Math.max(...Object.keys(q).map(joint => Math.abs(q[joint] - previous[joint]))) / maxVelocity;
          const t = point.hasTimeSinceReference() ? durationToSec(point.getTimeSinceReference()) : time + minTime;
          time = Math.max(t, time + minTime);
          previous = q;
          return { t: time, q, proto: point };
        });
        if (points.length === 0) {
          throw new CommandError(Status.STATUS_INVALID_REQUEST, 'The joint trajectory has no point.');
        }
        const referenceTime = trajectory.hasReferenceTime() ? timestampToSec(trajectory.getReferenceTime()) : now;
        return { type: 'arm_joint_move', behavior: { kind: 'joint_move', points, referenceTime, modified } };
      }
      case ArmCase.ARM_CARTESIAN_COMMAND: {
        const req = request.getArmCartesianCommand();
        const rootFrame = req.getRootFrameName() || 'odom';
        checkFrame(rootFrame, ['odom', 'vision', 'body', 'flat_body', 'gpe'], 'the arm Cartesian command');
        if (!req.hasPoseTrajectoryInTask()) {
          throw new CommandError(Status.STATUS_INVALID_REQUEST, 'The arm Cartesian command needs a pose trajectory.');
        }
        const trajectory = this._handTrajectory(
          rootFrame,
          req.getRootTformTask(),
          req.getWristTformTool(),
          req.getPoseTrajectoryInTask(),
          now,
        );
        return { type: 'arm_cartesian', behavior: { kind: 'cartesian', ...trajectory } };
      }
      case ArmCase.ARM_IMPEDANCE_COMMAND: {
        const req = request.getArmImpedanceCommand();
        const rootFrame = req.getRootFrameName() || 'odom';
        checkFrame(rootFrame, ['odom', 'vision', 'body', 'flat_body', 'gpe'], 'the arm impedance command');
        const trajectory = this._handTrajectory(
          rootFrame,
          req.getRootTformTask(),
          req.getWristTformTool(),
          req.getTaskTformDesiredTool(),
          now,
        );
        return { type: 'arm_impedance', behavior: { kind: 'impedance', ...trajectory } };
      }
      case ArmCase.ARM_GAZE_COMMAND: {
        const req = request.getArmGazeCommand();
        const frame1 = req.getFrame1Name();
        checkFrame(frame1, ['odom', 'vision', 'body', 'flat_body', 'gpe'], 'the gaze target');
        const targets = req.getTargetTrajectoryInFrame1()?.getPointsList() ?? [];
        if (targets.length === 0) {
          throw new CommandError(Status.STATUS_INVALID_REQUEST, 'The gaze command needs a target.');
        }
        const target = targets[targets.length - 1].getPoint();
        const targetWorld = poseMul(
          this.robot.body.frameInWorld(frame1),
          pose(target.getX(), target.getY(), target.getZ()),
        );
        let handWorld;
        if (req.hasToolTrajectoryInFrame2()) {
          const frame2 = req.getFrame2Name() || 'odom';
          checkFrame(frame2, ['odom', 'vision', 'body', 'flat_body', 'gpe'], 'the gaze tool trajectory');
          const tools = req.getToolTrajectoryInFrame2().getPointsList();
          handWorld = poseMul(this.robot.body.frameInWorld(frame2), poseFromProto(tools[tools.length - 1]?.getPose()));
        } else {
          handWorld = arm.frameInWorld('hand');
        }
        // The hand looks at the target.
        const dx = targetWorld.x - handWorld.x;
        const dy = targetWorld.y - handWorld.y;
        const dz = targetWorld.z - handWorld.z;
        const rot = quatFromEulerZXY(Math.atan2(dy, dx), 0, Math.atan2(-dz, Math.hypot(dx, dy)));
        const goal = pose(handWorld.x, handWorld.y, handWorld.z, rot);
        const vision = this.robot.body.frameInWorld('vision');
        const goalVision = poseMul(poseInv(vision), goal);
        const startHandRoot = poseMul(poseInv(vision), arm.frameInWorld('hand'));
        return {
          type: 'arm_gaze',
          behavior: {
            kind: 'gaze',
            points: [{ t: 1, pose: goalVision }],
            referenceTime: now,
            rootFrame: 'vision',
            startHandRoot,
          },
        };
      }
      case ArmCase.ARM_VELOCITY_COMMAND: {
        const req = request.getArmVelocityCommand();
        const endTime = this._endTime(req, parsed, 'the arm velocity');
        const { CommandCase } = armCommandPb.ArmVelocityCommand.Request;
        let velocityWorld;
        if (req.getCommandCase() === CommandCase.CYLINDRICAL_VELOCITY) {
          const cylindrical = req.getCylindricalVelocity();
          const linear = cylindrical.getLinearVelocity();
          const maxLinear = doubleValue(cylindrical.getMaxLinearVelocity()) ?? 0.5;
          velocityWorld = currentArm => {
            const shoulderWorld = poseMul(this.robot.body.bodyPoseWorld(), pose(0.292, 0, 0.188));
            const hand = currentArm.frameInWorld('hand');
            const dx = hand.x - shoulderWorld.x;
            const dy = hand.y - shoulderWorld.y;
            const radius = Math.max(Math.hypot(dx, dy), 1e-3);
            const radial = { x: dx / radius, y: dy / radius };
            const v = {
              x: (linear?.getR() ?? 0) * radial.x - (linear?.getTheta() ?? 0) * radius * radial.y,
              y: (linear?.getR() ?? 0) * radial.y + (linear?.getTheta() ?? 0) * radius * radial.x,
              z: linear?.getZ() ?? 0,
            };
            const speed = Math.hypot(v.x, v.y, v.z);
            const scale = speed > maxLinear ? maxLinear / speed : 1;
            return { x: v.x * scale, y: v.y * scale, z: v.z * scale };
          };
        } else if (req.getCommandCase() === CommandCase.CARTESIAN_VELOCITY) {
          const cartesian = req.getCartesianVelocity();
          const frame = cartesian.getFrameName();
          checkFrame(frame, KNOWN_FRAMES, 'the arm velocity');
          const v = cartesian.getVelocityInFrameName();
          velocityWorld = () =>
            quatRotate(this.robot.body.frameInWorld(frame).rot, {
              x: v?.getX() ?? 0,
              y: v?.getY() ?? 0,
              z: v?.getZ() ?? 0,
            });
        } else {
          throw new CommandError(Status.STATUS_INVALID_REQUEST, 'The arm velocity command is empty.');
        }
        return { type: 'arm_velocity', behavior: { kind: 'velocity', endTime, velocityWorld } };
      }
      case ArmCase.ARM_STOP_COMMAND:
        return { type: 'arm_stop', behavior: { kind: 'hold' } };
      case ArmCase.ARM_DRAG_COMMAND:
        return { type: 'arm_drag', behavior: { kind: 'drag' } };
      default:
        throw new CommandError(Status.STATUS_INVALID_REQUEST, 'The arm command is empty.');
    }
  }

  /**
   * @param {gripperCommandPb.GripperCommand.Request} request
   * @returns {object} The gripper part.
   */
  _parseGripper(request) {
    if (!request.hasClawGripperCommand()) {
      throw new CommandError(Status.STATUS_INVALID_REQUEST, 'The gripper command is empty.');
    }
    const claw = request.getClawGripperCommand();
    const points = claw.getTrajectory()?.getPointsList() ?? [];
    if (points.length === 0) {
      throw new CommandError(Status.STATUS_INVALID_REQUEST, 'The gripper trajectory has no point.');
    }
    return {
      type: 'claw_gripper',
      target: points[points.length - 1].getPoint(),
      maxVelocity: doubleValue(claw.getMaximumOpenCloseVelocity()),
    };
  }

  // Interventions of the robot itself.

  /**
   * Sits the robot down and powers off its motors (E-Stop SETTLE_THEN_CUT, low battery, keepalive).
   * @param {string} reason
   */
  startInternalSafePowerOff(reason) {
    const { body, arm } = this.robot;
    if (body.behavior.kind === 'safe_power_off') return;
    logger.info(`Sitting down to power off (${reason})`);
    this.robot.docking.onMobilityOverridden();
    this.robot.autoReturn.onMobilityOverridden();
    body.start({ kind: 'safe_power_off', owner: 'robot', reason });
    if (arm) arm.start({ kind: 'stow', owner: 'robot' });
  }

  /**
   * Stops the motion of the robot, which keeps standing (keepalive HaltRobot).
   * @param {string} reason
   */
  halt(reason) {
    const { body } = this.robot;
    if (!body.isStanding()) return;
    logger.info(`Halting (${reason})`);
    this.robot.docking.onMobilityOverridden();
    this.robot.autoReturn.onMobilityOverridden();
    body.start({ kind: 'stop', owner: 'robot', bodyControl: null });
  }

  /** The motor power is off: the motions stop (the commands are overridden), except a finished safe power off. */
  onMotorPowerLost() {
    const { body, arm } = this.robot;
    if (body.behavior.kind !== 'safe_power_off') body.idle();
    if (arm) {
      if (arm.behavior.owner !== body.behavior.owner) arm.behavior = { kind: 'hold', owner: null, state: {} };
      arm.gripperBehavior = { owner: null, state: { status: 'at_goal' } };
    }
  }

  /** The robot fell: the commands are overridden. */
  onRobotFell() {
    const { arm } = this.robot;
    if (arm) arm.behavior = { kind: 'hold', owner: null, state: {} };
  }

  // Feedback.

  /**
   * RobotCommandFeedback.
   * @param {number} id
   * @returns {?robotCommandPb.RobotCommandFeedback} Null for an unknown command.
   */
  feedback(id) {
    const record = this.commands.get(id);
    if (!record) return null;
    const now = this.robot.clock.now();
    const feedback = new robotCommandPb.RobotCommandFeedback();
    if (record.kind === 'full_body') {
      feedback.setFullBodyFeedback(this._fullBodyFeedback(record, now));
      return feedback;
    }
    const synchronized = new synchronizedCommandPb.SynchronizedCommand.Feedback();
    if (record.parts.mobility) synchronized.setMobilityCommandFeedback(this._mobilityFeedback(record, now));
    if (record.parts.arm) synchronized.setArmCommandFeedback(this._armFeedback(record, now));
    if (record.parts.gripper) synchronized.setGripperCommandFeedback(this._gripperFeedback(record));
    feedback.setSynchronizedFeedback(synchronized);
    return feedback;
  }

  _slotStatus(owned, endTime, now) {
    if (!owned) return FeedbackStatus.STATUS_COMMAND_OVERRIDDEN;
    if (endTime !== null && endTime !== undefined && now > endTime) return FeedbackStatus.STATUS_COMMAND_TIMED_OUT;
    return FeedbackStatus.STATUS_PROCESSING;
  }

  _mobilityFeedback(record, now) {
    const { body } = this.robot;
    const part = record.parts.mobility;
    const owned = body.behavior.owner === record.id;
    const feedback = new mobilityCommandPb.MobilityCommand.Feedback().setStatus(
      this._slotStatus(owned, part.endTime, now),
    );
    const state = owned ? body.behavior.state : {};
    switch (part.type) {
      case 'stand': {
        const { StandCommand } = basicCommandPb;
        const standing = owned && body.isStanding() && body.offsetSettled() && state.trajectoryDone !== false;
        feedback.setStandFeedback(
          new StandCommand.Feedback()
            .setStatus(
              standing
                ? StandCommand.Feedback.Status.STATUS_IS_STANDING
                : StandCommand.Feedback.Status.STATUS_IN_PROGRESS,
            )
            .setStandingState(StandCommand.Feedback.StandingState.STANDING_CONTROLLED),
        );
        break;
      }
      case 'sit': {
        const { SitCommand } = basicCommandPb;
        const sitting = owned && body.posture === 'sitting';
        feedback.setSitFeedback(
          new SitCommand.Feedback().setStatus(
            sitting ? SitCommand.Feedback.Status.STATUS_IS_SITTING : SitCommand.Feedback.Status.STATUS_IN_PROGRESS,
          ),
        );
        break;
      }
      case 'se2_trajectory': {
        const { Feedback } = basicCommandPb.SE2TrajectoryCommand;
        const status =
          {
            stopped: Feedback.Status.STATUS_STOPPED,
            stopping: Feedback.Status.STATUS_STOPPING,
          }[state.status] ?? Feedback.Status.STATUS_IN_PROGRESS;
        const settled = state.status === 'stopped' && !body.isWalking();
        const finalGoal =
          {
            achievable: Feedback.FinalGoalStatus.FINAL_GOAL_STATUS_ACHIEVABLE,
            blocked: Feedback.FinalGoalStatus.FINAL_GOAL_STATUS_BLOCKED,
          }[state.finalGoal] ?? Feedback.FinalGoalStatus.FINAL_GOAL_STATUS_IN_PROGRESS;
        feedback.setSe2TrajectoryFeedback(
          new Feedback()
            .setStatus(status)
            .setBodyMovementStatus(
              settled
                ? Feedback.BodyMovementStatus.BODY_STATUS_SETTLED
                : Feedback.BodyMovementStatus.BODY_STATUS_MOVING,
            )
            .setFinalGoalStatus(finalGoal)
            .setRequestInformation(part.request),
        );
        break;
      }
      case 'se2_velocity':
        feedback.setSe2VelocityFeedback(
          new basicCommandPb.SE2VelocityCommand.Feedback().setRequestInformation(part.request),
        );
        break;
      case 'stance': {
        const { StanceCommand } = basicCommandPb;
        const status =
          {
            stanced: StanceCommand.Feedback.Status.STATUS_STANCED,
            too_far: StanceCommand.Feedback.Status.STATUS_TOO_FAR_AWAY,
          }[state.status] ?? StanceCommand.Feedback.Status.STATUS_GOING_TO_STANCE;
        feedback.setStanceFeedback(new StanceCommand.Feedback().setStatus(status));
        break;
      }
      case 'stop':
        feedback.setStopFeedback(new basicCommandPb.StopCommand.Feedback());
        break;
      case 'follow_arm':
        feedback.setFollowArmFeedback(new basicCommandPb.FollowArmCommand.Feedback());
        break;
      case 'freeze':
        feedback.setFreezeFeedback(new basicCommandPb.FreezeCommand.Feedback());
        break;
      default:
        break;
    }
    return feedback;
  }

  _fullBodyFeedback(record, now) {
    const { body } = this.robot;
    const part = record.parts.fullBody;
    const owned = body.behavior.owner === record.id;
    const feedback = new fullBodyCommandPb.FullBodyCommand.Feedback().setStatus(this._slotStatus(owned, null, now));
    const state = owned ? body.behavior.state : {};
    switch (part.type) {
      case 'stop':
        feedback.setStopFeedback(new basicCommandPb.StopCommand.Feedback());
        break;
      case 'freeze':
        feedback.setFreezeFeedback(new basicCommandPb.FreezeCommand.Feedback());
        break;
      case 'selfright': {
        const { SelfRightCommand } = basicCommandPb;
        feedback.setSelfrightFeedback(
          new SelfRightCommand.Feedback().setStatus(
            state.done
              ? SelfRightCommand.Feedback.Status.STATUS_COMPLETED
              : SelfRightCommand.Feedback.Status.STATUS_IN_PROGRESS,
          ),
        );
        break;
      }
      case 'safe_power_off': {
        const { SafePowerOffCommand } = basicCommandPb;
        const off = !this.robot.power.motorsOnOrPowering();
        feedback.setSafePowerOffFeedback(
          new SafePowerOffCommand.Feedback().setStatus(
            off
              ? SafePowerOffCommand.Feedback.Status.STATUS_POWERED_OFF
              : SafePowerOffCommand.Feedback.Status.STATUS_IN_PROGRESS,
          ),
        );
        break;
      }
      case 'battery_change_pose': {
        const { BatteryChangePoseCommand } = basicCommandPb;
        feedback.setBatteryChangePoseFeedback(
          new BatteryChangePoseCommand.Feedback().setStatus(
            state.done
              ? BatteryChangePoseCommand.Feedback.Status.STATUS_COMPLETED
              : BatteryChangePoseCommand.Feedback.Status.STATUS_IN_PROGRESS,
          ),
        );
        break;
      }
      case 'payload_estimation': {
        const { Feedback } = payloadEstimationPb.PayloadEstimationCommand;
        const done = (state.progress ?? 0) >= 1;
        const estimation = new Feedback()
          .setStatus(done ? Feedback.Status.STATUS_SMALL_MASS : Feedback.Status.STATUS_IN_PROGRESS)
          .setProgress(state.progress ?? 0)
          .setError(Feedback.Error.ERROR_NONE);
        feedback.setPayloadEstimationFeedback(estimation);
        break;
      }
      default:
        break;
    }
    return feedback;
  }

  _armFeedback(record, now) {
    const { arm } = this.robot;
    const { ArmCommand } = armCommandPb;
    const part = record.parts.arm;
    if (part.incompatible) return new ArmCommand.Feedback().setStatus(FeedbackStatus.STATUS_INCOMPATIBLE_HARDWARE);
    const owned = arm.behavior.owner === record.id;
    const feedback = new ArmCommand.Feedback().setStatus(this._slotStatus(owned, part.behavior.endTime ?? null, now));
    const state = owned ? arm.behavior.state : {};
    switch (part.type) {
      case 'named_arm_position': {
        const { Status: NamedStatus } = armCommandPb.NamedArmPositionsCommand.Feedback;
        const status =
          {
            complete: NamedStatus.STATUS_COMPLETE,
            stalled_holding_item: NamedStatus.STATUS_STALLED_HOLDING_ITEM,
          }[state.status] ?? NamedStatus.STATUS_IN_PROGRESS;
        feedback.setNamedArmPositionFeedback(new armCommandPb.NamedArmPositionsCommand.Feedback().setStatus(status));
        break;
      }
      case 'arm_joint_move': {
        const { Feedback } = armCommandPb.ArmJointMoveCommand;
        const behavior = part.behavior;
        const remaining = Math.max(0, behavior.referenceTime + behavior.points[behavior.points.length - 1].t - now);
        feedback.setArmJointMoveFeedback(
          new Feedback()
            .setStatus(
              state.status === 'complete' ? Feedback.Status.STATUS_COMPLETE : Feedback.Status.STATUS_IN_PROGRESS,
            )
            .setPlannerStatus(
              behavior.modified
                ? Feedback.PlannerStatus.PLANNER_STATUS_MODIFIED
                : Feedback.PlannerStatus.PLANNER_STATUS_SUCCESS,
            )
            .setPlannedPointsList(behavior.points.map(point => point.proto))
            .setTimeToGoal(secToDuration(remaining)),
        );
        break;
      }
      case 'arm_cartesian': {
        const { Feedback } = armCommandPb.ArmCartesianCommand;
        const status =
          {
            complete: Feedback.Status.STATUS_TRAJECTORY_COMPLETE,
            stalled: Feedback.Status.STATUS_TRAJECTORY_STALLED,
          }[state.status] ?? Feedback.Status.STATUS_IN_PROGRESS;
        const distances = this._handGoalDistances(part.behavior);
        feedback.setArmCartesianFeedback(
          new Feedback()
            .setStatus(owned ? status : Feedback.Status.STATUS_TRAJECTORY_CANCELLED)
            .setMeasuredPosTrackingError(0)
            .setMeasuredRotTrackingError(0)
            .setMeasuredPosDistanceToGoal(distances.position)
            .setMeasuredRotDistanceToGoal(distances.rotation),
        );
        break;
      }
      case 'arm_impedance': {
        const { Feedback } = armCommandPb.ArmImpedanceCommand;
        const status =
          {
            complete: Feedback.Status.STATUS_TRAJECTORY_COMPLETE,
            stalled: Feedback.Status.STATUS_TRAJECTORY_STALLED,
          }[state.status] ?? Feedback.Status.STATUS_IN_PROGRESS;
        feedback.setArmImpedanceFeedback(
          new Feedback().setStatus(owned ? status : Feedback.Status.STATUS_TRAJECTORY_CANCELLED),
        );
        break;
      }
      case 'arm_gaze': {
        const { Feedback } = armCommandPb.GazeCommand;
        const complete = state.status === 'complete';
        const distances = this._handGoalDistances(part.behavior);
        feedback.setArmGazeFeedback(
          new Feedback()
            .setStatus(
              {
                complete: Feedback.Status.STATUS_TRAJECTORY_COMPLETE,
                stalled: Feedback.Status.STATUS_TOOL_TRAJECTORY_STALLED,
              }[state.status] ?? Feedback.Status.STATUS_IN_PROGRESS,
            )
            .setGazingAtTarget(complete)
            .setGazeToTargetRotationMeasured(distances.rotation)
            .setHandPositionAtGoal(distances.position < 0.02)
            .setHandDistanceToGoalMeasured(distances.position)
            .setHandRollAtGoal(true)
            .setHandRollToTargetRollMeasured(0),
        );
        break;
      }
      case 'arm_velocity':
        feedback.setArmVelocityFeedback(new armCommandPb.ArmVelocityCommand.Feedback());
        break;
      case 'arm_stop':
        feedback.setArmStopFeedback(new armCommandPb.ArmStopCommand.Feedback());
        break;
      case 'arm_drag': {
        const { ArmDragCommand } = basicCommandPb;
        feedback.setArmDragFeedback(
          new ArmDragCommand.Feedback().setStatus(
            arm.holdingItem
              ? ArmDragCommand.Feedback.Status.STATUS_DRAGGING
              : ArmDragCommand.Feedback.Status.STATUS_OTHER_FAILURE,
          ),
        );
        break;
      }
      default:
        break;
    }
    return feedback;
  }

  /**
   * Distance between the hand and the goal of a Cartesian behavior.
   * @param {object} behavior
   * @returns {{position: number, rotation: number}}
   */
  _handGoalDistances(behavior) {
    const rootWorld = this.robot.body.frameInWorld(behavior.rootFrame);
    const goal = behavior.points[behavior.points.length - 1].pose;
    const hand = poseMul(poseInv(rootWorld), this.robot.arm.frameInWorld('hand'));
    const position = Math.hypot(goal.x - hand.x, goal.y - hand.y, goal.z - hand.z);
    const dot = Math.abs(
      goal.rot.w * hand.rot.w + goal.rot.x * hand.rot.x + goal.rot.y * hand.rot.y + goal.rot.z * hand.rot.z,
    );
    return { position, rotation: 2 * Math.acos(Math.min(1, dot)) };
  }

  _gripperFeedback(record) {
    const { arm } = this.robot;
    const { GripperCommand, ClawGripperCommand } = gripperCommandPb;
    const part = record.parts.gripper;
    if (part.incompatible) return new GripperCommand.Feedback().setStatus(FeedbackStatus.STATUS_INCOMPATIBLE_HARDWARE);
    const owned = arm.gripperBehavior.owner === record.id;
    const status =
      {
        at_goal: ClawGripperCommand.Feedback.Status.STATUS_AT_GOAL,
        applying_force: ClawGripperCommand.Feedback.Status.STATUS_APPLYING_FORCE,
      }[owned ? arm.gripperBehavior.state.status : 'in_progress'] ??
      ClawGripperCommand.Feedback.Status.STATUS_IN_PROGRESS;
    return new GripperCommand.Feedback()
      .setStatus(owned ? FeedbackStatus.STATUS_PROCESSING : FeedbackStatus.STATUS_COMMAND_OVERRIDDEN)
      .setClawGripperFeedback(new ClawGripperCommand.Feedback().setStatus(status));
  }

  /**
   * ClearBehaviorFault.
   * @param {number} faultId
   * @returns {{status: number, fault: ?any, blockingSystemFaults: any[]}}
   */
  clearBehaviorFault(faultId) {
    const ClearStatus = robotCommandPb.ClearBehaviorFaultResponse.Status;
    const result = this.robot.faults.clearBehaviorFault(faultId);
    return {
      status: result.cleared ? ClearStatus.STATUS_CLEARED : ClearStatus.STATUS_NOT_CLEARED,
      fault: result.fault,
      blockingSystemFaults: result.blockingSystemFaults,
    };
  }
}

module.exports = { CommandManager, INERTIAL_FRAMES };
