'use strict';

const dockingPb = require('../bosdyn/api/docking/docking_pb');
const { LoggerUtil } = require('../loggerUtil');

const logger = LoggerUtil.getLogger('DOCKING');

const { DockState, DockType, PrepPoseBehavior } = dockingPb;
const FeedbackStatus = dockingPb.DockingCommandFeedbackResponse.Status;
const ResponseStatus = dockingPb.DockingCommandResponse.Status;

// The prep pose is in front of the dock, facing its fiducial.
const PREP_DISTANCE = 1.0;
const DOCK_ID_RANGE = [520, 549];

/**
 * The docking service: the robot walks to the pose in front of the dock, walks onto the dock, sits down and powers off
 * its motors; the battery charges while docked. Undocking stands up and walks back to the prep pose.
 */
class DockingSystem {
  /**
   * @param {import('../robot')} robot
   */
  constructor(robot) {
    this.robot = robot;
    this.docked = false;
    this.dockId = 0;
    this.linkConnectedAt = null;
    this.nextId = 1 + Math.floor(Math.random() * 1000);
    /** @type {Map<number, object>} */
    this.commands = new Map();
    this.active = null;
  }

  /** Forgets the commands (reboot), the robot stays on its dock. */
  reset() {
    this.commands.clear();
    this.active = null;
  }

  /**
   * Puts the robot on the dock (start of the simulation, console).
   * @param {number} dockId
   * @returns {boolean}
   */
  placeOnDock(dockId) {
    const dock = this.robot.world.dock(dockId);
    if (!dock) return false;
    this.robot.body.teleport(dock.base.x, dock.base.y, dock.base.yaw + Math.PI);
    this.docked = true;
    this.dockId = dockId;
    this.linkConnectedAt = this.robot.clock.now();
    return true;
  }

  /** @returns {boolean} */
  isDocked() {
    return this.docked;
  }

  /** @returns {boolean} */
  isCharging() {
    return this.docked && this.linkConnectedAt !== null && this.robot.clock.now() - this.linkConnectedAt > 2;
  }

  _prepPose(dock) {
    return {
      x: dock.base.x + PREP_DISTANCE * Math.cos(dock.base.yaw),
      y: dock.base.y + PREP_DISTANCE * Math.sin(dock.base.yaw),
      yaw: dock.base.yaw + Math.PI,
    };
  }

  /**
   * DockingCommand (the lease and the time sync are checked by the service).
   * @param {dockingPb.DockingCommandRequest} request
   * @param {number} endTime
   * @param {boolean} synced The clock of the client is synchronized.
   * @returns {{status: number, id: number}}
   */
  command(request, endTime, synced) {
    const behavior = request.getPrepPoseBehavior();
    const undock = behavior === PrepPoseBehavior.PREP_POSE_UNDOCK;
    const { arm, world } = this.robot;
    if (arm?.holdingItem && !undock) return { status: ResponseStatus.STATUS_ERROR_GRIPPER_HOLDING_ITEM, id: 0 };
    let dock;
    if (undock) {
      if (!this.docked) return { status: ResponseStatus.STATUS_ERROR_NOT_DOCKED, id: 0 };
      dock = world.dock(this.dockId);
    } else {
      const dockId = request.getDockingStationId();
      dock = this.docked && this.dockId === dockId ? world.dock(dockId) : world.detectedDock(dockId);
      if (!dock) return { status: ResponseStatus.STATUS_ERROR_DOCK_NOT_FOUND, id: 0 };
    }
    const previous = this.commands.get(this.active);
    if (previous && previous.status === FeedbackStatus.STATUS_IN_PROGRESS) {
      previous.status = FeedbackStatus.STATUS_OLD_DOCKING_COMMAND;
    }
    const id = this.nextId++;
    const now = this.robot.clock.now();
    const command = {
      id,
      dock,
      undock,
      onlyPose: behavior === PrepPoseBehavior.PREP_POSE_ONLY_POSE,
      skipPose: behavior === PrepPoseBehavior.PREP_POSE_SKIP_POSE,
      endTime,
      phase: 'start',
      status: FeedbackStatus.STATUS_IN_PROGRESS,
    };
    this.commands.set(id, command);
    this.active = id;
    if (!synced) {
      command.status = FeedbackStatus.STATUS_ERROR_NO_TIMESYNC;
    } else if (endTime > now + this.robot.config.mobility.maxCommandDurationSec) {
      command.status = FeedbackStatus.STATUS_ERROR_TOO_DISTANT;
    } else if (!this.robot.power.motorsOn()) {
      logger.warn('Docking command refused: the motors are off');
      command.status = FeedbackStatus.STATUS_ERROR_SYSTEM;
    } else if (!undock && this.docked && this.dockId === dock.id) {
      command.status = FeedbackStatus.STATUS_DOCKED;
    } else {
      logger.info(`${undock ? 'Undocking' : `Docking at dock ${dock.id}`} (command ${id})`);
    }
    return { status: ResponseStatus.STATUS_OK, id };
  }

  /**
   * DockingCommandFeedback.
   * @param {number} id
   * @param {?number} newEndTime UpdateDockingParams.end_time.
   * @returns {?number} The status, null for an unknown command.
   */
  feedback(id, newEndTime = null) {
    const command = this.commands.get(id);
    if (!command) return null;
    if (newEndTime !== null && command.status === FeedbackStatus.STATUS_IN_PROGRESS) command.endTime = newEndTime;
    return command.status;
  }

  /** A robot command took the mobility: the docking command stops. */
  onMobilityOverridden() {
    const command = this.commands.get(this.active);
    if (
      command &&
      command.status === FeedbackStatus.STATUS_IN_PROGRESS &&
      this.robot.body.behavior.owner === 'docking'
    ) {
      logger.warn(`Docking command ${command.id} interrupted by another command`);
      command.status = FeedbackStatus.STATUS_ERROR_SYSTEM;
    }
  }

  _walk(points, maxVel, endTime) {
    this.robot.body.start({ kind: 'trajectory', owner: 'docking', points, endTime, maxVel, bodyControl: null });
  }

  /**
   * Runs the active docking command.
   * @param {number} now
   */
  update(now) {
    const { body, power } = this.robot;
    // Walking off the dock undocks the robot.
    if (this.docked) {
      const dock = this.robot.world.dock(this.dockId);
      if (dock && Math.hypot(body.footprint.x - dock.base.x, body.footprint.y - dock.base.y) > 0.4) {
        this.docked = false;
        this.linkConnectedAt = null;
        logger.info(`Robot off dock ${this.dockId}`);
      }
    }
    const command = this.commands.get(this.active);
    if (!command || command.status !== FeedbackStatus.STATUS_IN_PROGRESS) return;
    if (now > command.endTime) {
      command.status = FeedbackStatus.STATUS_ERROR_COMMAND_TIMED_OUT;
      if (body.behavior.owner === 'docking') body.start({ kind: 'stop', owner: 'docking', bodyControl: null });
      logger.warn(`Docking command ${command.id} timed out`);
      return;
    }
    if (!power.motorsOn() && command.phase !== 'sit') {
      command.status = FeedbackStatus.STATUS_ERROR_SYSTEM;
      return;
    }
    const { dock } = command;
    const prep = this._prepPose(dock);
    const base = { x: dock.base.x, y: dock.base.y, yaw: dock.base.yaw + Math.PI };
    const trajectoryDone = () => body.behavior.owner === 'docking' && body.behavior.state.status === 'stopped';
    switch (command.phase) {
      case 'start':
        if (command.undock || !command.skipPose) {
          this._walk([prep], null, command.endTime);
          command.phase = 'prep';
        } else {
          this._walk([base], { x: 0.3, y: 0.2, yaw: 0.5 }, command.endTime);
          command.phase = 'onto_dock';
        }
        break;
      case 'prep':
        if (!trajectoryDone()) break;
        if (body.behavior.state.finalGoal === 'blocked') {
          command.status = FeedbackStatus.STATUS_ERROR_STUCK;
        } else if (command.undock || command.onlyPose) {
          command.status = FeedbackStatus.STATUS_AT_PREP_POSE;
          body.start({ kind: 'stand', owner: 'docking', bodyControl: null });
          logger.info(command.undock ? 'Undocked' : 'At the prep pose');
        } else {
          this._walk([base], { x: 0.3, y: 0.2, yaw: 0.5 }, command.endTime);
          command.phase = 'onto_dock';
        }
        break;
      case 'onto_dock':
        if (!trajectoryDone()) break;
        body.start({ kind: 'sit', owner: 'docking' });
        command.phase = 'sit';
        break;
      case 'sit':
        if (body.posture !== 'sitting') break;
        // On the dock: the robot powers its motors off and the dock powers the robot.
        if (power.motorsOn()) power.cutMotorPower('docked');
        this.docked = true;
        this.dockId = dock.id;
        this.linkConnectedAt = now;
        command.status = FeedbackStatus.STATUS_DOCKED;
        logger.info(`Docked at dock ${dock.id}`);
        break;
      default:
        break;
    }
  }

  /**
   * @returns {dockingPb.DockState}
   */
  stateToProto() {
    const command = this.commands.get(this.active);
    const running = command && command.status === FeedbackStatus.STATUS_IN_PROGRESS;
    let status = this.docked ? DockState.DockedStatus.DOCK_STATUS_DOCKED : DockState.DockedStatus.DOCK_STATUS_UNDOCKED;
    if (running) {
      status = command.undock
        ? DockState.DockedStatus.DOCK_STATUS_UNDOCKING
        : DockState.DockedStatus.DOCK_STATUS_DOCKING;
    }
    const state = new DockState().setStatus(status);
    if (this.docked) {
      state
        .setDockType(DockType.DOCK_TYPE_SPOT_DOCK)
        .setDockId(this.dockId)
        .setPowerStatus(
          this.isCharging() ? DockState.LinkStatus.LINK_STATUS_CONNECTED : DockState.LinkStatus.LINK_STATUS_DETECTING,
        );
    }
    return state;
  }

  /**
   * @returns {dockingPb.ConfigRange[]}
   */
  configToProto() {
    return [
      new dockingPb.ConfigRange()
        .setIdStart(DOCK_ID_RANGE[0])
        .setIdEnd(DOCK_ID_RANGE[1])
        .setType(DockType.DOCK_TYPE_SPOT_DOCK),
    ];
  }

  toJSON() {
    return { docked: this.docked, dockId: this.dockId };
  }

  loadFromJSON(json) {
    if (json?.docked && this.robot.world.dock(json.dockId)) {
      this.docked = true;
      this.dockId = json.dockId;
      this.linkConnectedAt = this.robot.clock.now();
    }
  }

  /** @returns {string} */
  describe() {
    if (!this.docked) return 'not docked';
    return `docked at ${this.dockId}${this.isCharging() ? ', charging' : ''}`;
  }
}

module.exports = { DockingSystem };
