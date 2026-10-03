'use strict';

const { EventEmitter } = require('node:events');
const { clearInterval, clearTimeout, setInterval, setTimeout } = require('node:timers');

const grpc = require('@grpc/grpc-js');

const { BehaviorFault } = require('./bosdyn/api/robot_state_pb');
const { LoggerUtil } = require('./loggerUtil');
const { Arm, ARM_JOINTS } = require('./sim/arm');
const { AuthManager } = require('./sim/auth');
const { AutoReturn } = require('./sim/auto_return');
const { Battery, Thermal } = require('./sim/battery');
const { Body, LEG_JOINTS } = require('./sim/body');
const { Cameras } = require('./sim/cameras');
const { Choreography } = require('./sim/choreography');
const { RobotClock } = require('./sim/clock');
const { CommandManager } = require('./sim/commands');
const { DataBuffer } = require('./sim/data_buffer');
const { Directory } = require('./sim/directory');
const { DockingSystem } = require('./sim/docking');
const { EstopSystem } = require('./sim/estop');
const { FaultManager } = require('./sim/faults');
const { KeepaliveManager } = require('./sim/keepalive');
const { LeaseManager } = require('./sim/lease');
const { License } = require('./sim/license');
const { MissionSystem } = require('./sim/missions');
const { PowerSystem } = require('./sim/power');
const { TimeSyncManager } = require('./sim/time_sync');
const { World } = require('./sim/world');

const logger = LoggerUtil.getLogger('ROBOT');

// Version of the state file.
const STATE_VERSION = 2;
// Longest step of the simulation (s).
const MAX_STEP = 0.02;

/**
 * The simulated robot: its subsystems and the simulation loop.
 *
 * Events: 'power:change', 'posture', 'lease:change', 'moved', 'offline' ({reboot}), 'boot', 'persist'.
 */
class Robot extends EventEmitter {
  /**
   * @param {import('./config').DEFAULT_CONFIG} config
   * @param {{clock?: RobotClock}} [options]
   */
  constructor(config, { clock } = {}) {
    super();
    this.setMaxListeners(100);
    this.config = config;
    /**
     * @type {RobotClock}
     */
    this.clock = clock ?? new RobotClock({ skewSec: config.clockSkewSec });
    this.online = true;
    this.offlineReason = '';
    this.irEmittersEnabled = true;
    this._timers = new Set();
    this._heldCalls = new Set();

    this.auth = new AuthManager(this);
    this.timeSync = new TimeSyncManager(this);
    this.directory = new Directory(this);
    this.license = new License(this);
    this.faults = new FaultManager(this);
    this.leases = new LeaseManager(this);
    this.estop = new EstopSystem(this);
    this.dataBuffer = new DataBuffer(this);
    this.arm = config.robot.hasArm ? new Arm(this) : null;
    this.body = new Body(this);
    this.thermal = new Thermal(this, [...LEG_JOINTS, ...(this.arm ? ARM_JOINTS.map(joint => `arm0.${joint}`) : [])]);
    this.battery = new Battery(this);
    this.power = new PowerSystem(this);
    this.commands = new CommandManager(this);
    this.world = new World(this);
    this.docking = new DockingSystem(this);
    this.keepalive = new KeepaliveManager(this);
    this.autoReturn = new AutoReturn(this);
    this.missions = new MissionSystem(this);
    this.choreography = new Choreography(this);
    this.cameras = new Cameras(this);
    if (config.dock?.startDocked) this.docking.placeOnDock(config.dock.id);
    this.body.boot();
    this.lastUpdate = this.clock.now();
  }

  /**
   * Starts the simulation loop.
   */
  start() {
    if (this._interval) return;
    this.lastUpdate = this.clock.now();
    this._interval = setInterval(() => this.sync(), 1000 / this.config.physics.rate);
    this._interval.unref();
  }

  /** Stops the simulation loop and the pending timers, and fails the calls held while the robot is off. */
  stop() {
    clearInterval(this._interval);
    this._interval = null;
    for (const timer of this._timers) clearTimeout(timer);
    this._timers.clear();
    this._releaseHeldCalls('The simulator stopped.');
  }

  /**
   * Holds a call while the robot is off or rebooting: it never answers, like an unreachable robot, so the client gets
   * its deadline (the SDKs expect a timeout after powering the robot off). The calls without deadline fail when the
   * robot boots (their connection is reset), or they would wait forever.
   * @param {any} call
   * @param {function({code: number, details: string}): void} fail
   */
  holdCall(call, fail) {
    const deadline = call.getDeadline?.();
    const entry = { fail, hasDeadline: deadline !== undefined && deadline !== Infinity };
    this._heldCalls.add(entry);
    call.on?.('cancelled', () => this._heldCalls.delete(entry));
  }

  /**
   * @param {string} details
   * @param {boolean} [all=true] All the calls, or only the calls without deadline.
   */
  _releaseHeldCalls(details, all = true) {
    for (const entry of this._heldCalls) {
      if (!all && entry.hasDeadline) continue;
      entry.fail({ code: grpc.status.UNAVAILABLE, details });
      this._heldCalls.delete(entry);
    }
  }

  _later(delaySec, fn) {
    const timer = setTimeout(() => {
      this._timers.delete(timer);
      fn();
    }, delaySec * 1000);
    timer.unref();
    this._timers.add(timer);
  }

  /**
   * Brings the simulation up to the current time.
   */
  sync() {
    const now = this.clock.now();
    let dt = now - this.lastUpdate;
    if (dt > 0) {
      // A long pause (debugger, sleeping computer) is simulated in steps, at most one minute.
      dt = Math.min(dt, 60);
      const steps = Math.ceil(dt / MAX_STEP);
      for (let i = 1; i <= steps; i++) this._step(this.lastUpdate + (dt * i) / steps, dt / steps);
      this.lastUpdate = now;
    } else {
      // The E-Stop acts at once (e.g. a check-in with CUT).
      this.estop.update(now);
    }
  }

  _step(now, dt) {
    this.estop.update(now);
    this.keepalive.update(now);
    this.leases.update(now);
    this.power.update(now);
    this.body.update(now, dt);
    this.arm?.update(now, dt);
    this.docking.update(now);
    this.autoReturn.update(now);
    this.missions.update(now, dt);
    this.choreography.update(now);
    this.battery.update(dt);
    this.thermal.update(dt);
    this.world.update(now);
    this.directory.checkLiveness(now);
  }

  /**
   * Powers the robot computers off (or reboots them): the robot stops answering.
   * @param {{reboot: boolean, delaySec?: number}} options
   */
  scheduleShutdown({ reboot, delaySec = 0.5 }) {
    this._later(delaySec, () => {
      this.online = false;
      this.offlineReason = reboot ? 'The robot is rebooting.' : 'The robot is powered off.';
      logger.warn(reboot ? 'Rebooting...' : 'The robot is powered off');
      this.emit('offline', { reboot });
      if (reboot) this._later(this.config.durations.reboot, () => this.boot());
    });
  }

  /**
   * Boots the robot: the runtime state is reset (motors off, new lease epoch, E-Stop not configured, new odometry),
   * the physical state is kept (position, battery, dock).
   */
  boot() {
    this.timeSync.reset();
    this.leases.reset();
    this.estop.reset();
    this.power.reset();
    this.commands.reset();
    this.keepalive.reset();
    this.autoReturn.reset();
    this.missions.reset();
    this.world.reset();
    this.docking.reset();
    this.arm?.boot();
    this.body.boot();
    for (const id of [...this.faults.behaviorFaults.keys()]) this.faults.behaviorFaults.delete(id);
    if (this.body.isFallen()) this.faults.addBehaviorFault(BehaviorFault.Cause.CAUSE_FALL, true);
    this.lastUpdate = this.clock.now();
    this.online = true;
    this.offlineReason = '';
    // The connections of before the reboot are reset.
    this._releaseHeldCalls('Connection reset: the robot rebooted.', false);
    logger.info('Robot booted');
    this.emit('boot');
  }

  /**
   * The persistent state: what a real robot keeps when it reboots.
   * @returns {object}
   */
  toJSON() {
    return {
      version: STATE_VERSION,
      serialNumber: this.config.robot.serialNumber,
      auth: this.auth.toJSON(),
      battery: this.battery.toJSON(),
      body: this.body.toJSON(),
      docking: this.docking.toJSON(),
    };
  }

  /**
   * @param {object} json
   * @returns {boolean} Whether the state was loaded.
   */
  loadFromJSON(json) {
    if (json?.version !== STATE_VERSION || json.serialNumber !== this.config.robot.serialNumber) return false;
    this.auth.loadFromJSON(json.auth);
    this.battery.loadFromJSON(json.battery);
    this.body.loadFromJSON(json.body);
    this.docking.loadFromJSON(json.docking);
    if (this.body.isFallen()) this.faults.addBehaviorFault(BehaviorFault.Cause.CAUSE_FALL, true);
    return true;
  }

  /**
   * A summary for the console.
   * @returns {string[]}
   */
  describe() {
    this.sync();
    const lines = [
      `power: ${this.power.describe()}`,
      `body: ${this.body.describe()}`,
      ...(this.arm ? [`arm: ${this.arm.describe()}`] : []),
      `battery: ${this.battery.describe()}`,
      `dock: ${this.docking.describe()}`,
      `E-Stop: ${this.estop.describe().join('; ')}`,
      ...this.leases.describe().map(line => `lease ${line}`),
      `auto return: ${this.autoReturn.describe()}`,
      ...this.faults.describe(),
      ...this.world.describe(),
    ];
    if (!this.online) lines.unshift(this.offlineReason);
    return lines;
  }
}

module.exports = { Robot, STATE_VERSION };
