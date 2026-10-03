'use strict';

const { durationToSec } = require('./clock');
const { leaseFromProto } = require('./lease');
const autoReturnPb = require('../bosdyn/api/auto_return/auto_return_pb');
const { LoggerUtil } = require('../loggerUtil');

const logger = LoggerUtil.getLogger('AUTO_RETURN');

// Distance between the points of the recorded path.
const BREADCRUMB_SPACING = 0.25;
const MAX_BREADCRUMBS = 2000;

/**
 * AutoReturn: once configured, the robot records the path it walks, and walks it back when the client stops
 * controlling the robot (its lease becomes stale), or when Start is called. The return is limited by the maximum
 * displacement (a radius around the position where it starts) and the maximum duration.
 */
class AutoReturn {
  /**
   * @param {import('../robot')} robot
   */
  constructor(robot) {
    this.robot = robot;
    this.reset();
    robot.on('moved', () => this._record());
  }

  /** Disabled (boot of the robot). */
  reset() {
    this.request = null;
    this.breadcrumbs = [];
    this.running = null;
    this._triggeredRequest = null;
  }

  /**
   * @param {autoReturnPb.Params} params
   * @returns {?autoReturnPb.Params} The invalid parameters, null when valid.
   */
  static invalidParams(params) {
    const invalid = new autoReturnPb.Params();
    let bad = false;
    if (!params || !(params.getMaxDisplacement() > 0)) {
      invalid.setMaxDisplacement(params?.getMaxDisplacement() ?? 0);
      bad = true;
    }
    if (params?.hasMaxDuration() && !(durationToSec(params.getMaxDuration()) > 0)) {
      invalid.setMaxDuration(params.getMaxDuration());
      bad = true;
    }
    return bad ? invalid : null;
  }

  /**
   * Configure.
   * @param {autoReturnPb.ConfigureRequest} request
   * @returns {{status: number, invalidParams: ?autoReturnPb.Params}}
   */
  configure(request) {
    const Status = autoReturnPb.ConfigureResponse.Status;
    const invalid = AutoReturn.invalidParams(request.getParams());
    if (invalid || request.getLeasesList().length === 0) {
      return { status: Status.STATUS_INVALID_PARAMS, invalidParams: invalid ?? new autoReturnPb.Params() };
    }
    this.request = request.clone();
    if (request.getClearBuffer()) this.breadcrumbs = [];
    logger.info(`Configured: max displacement ${request.getParams().getMaxDisplacement()} m`);
    return { status: Status.STATUS_OK, invalidParams: null };
  }

  /** @returns {boolean} */
  get enabled() {
    return this.request !== null;
  }

  _record() {
    if (!this.enabled || this.running) return;
    const { footprint } = this.robot.body;
    const last = this.breadcrumbs[this.breadcrumbs.length - 1];
    if (!last || Math.hypot(footprint.x - last.x, footprint.y - last.y) >= BREADCRUMB_SPACING) {
      this.breadcrumbs.push({ x: footprint.x, y: footprint.y, yaw: footprint.yaw });
      if (this.breadcrumbs.length > MAX_BREADCRUMBS) this.breadcrumbs.shift();
    }
  }

  /**
   * Start: walks back now.
   * @param {autoReturnPb.StartRequest} request
   * @returns {{status: number, invalidParams: ?autoReturnPb.Params}}
   */
  start(request) {
    const Status = autoReturnPb.StartResponse.Status;
    if (request.hasParams()) {
      const invalid = AutoReturn.invalidParams(request.getParams());
      if (invalid) return { status: Status.STATUS_INVALID_PARAMS, invalidParams: invalid };
    }
    const leases = request.getLeasesList().map(leaseFromProto);
    const params = request.hasParams() ? request.getParams() : this.request?.getParams();
    if (!params) return { status: Status.STATUS_INVALID_PARAMS, invalidParams: new autoReturnPb.Params() };
    this.trigger(
      leases.length > 0 ? leases : this.request.getLeasesList().map(leaseFromProto),
      'Start request',
      params,
    );
    return { status: Status.STATUS_OK, invalidParams: null };
  }

  /**
   * Walks back along the recorded path.
   * @param {import('./lease').LeaseData[]} leases The leases that AutoReturn uses (they make the older leases of the
   *   client STATUS_OLDER).
   * @param {string} reason
   * @param {?autoReturnPb.Params} [params]
   */
  trigger(leases, reason, params = null) {
    const parameters = params ?? this.request?.getParams();
    if (!parameters) return;
    const { body, leases: leaseManager, power } = this.robot;
    if (!power.motorsOn()) {
      logger.warn(`Not started (${reason}): the motors are off`);
      return;
    }
    for (const lease of leases) leaseManager.use(lease, null, { record: true });
    const start = body.footprint;
    const maxDisplacement = parameters.getMaxDisplacement();
    const points = [];
    for (let i = this.breadcrumbs.length - 1; i >= 0; i--) {
      const crumb = this.breadcrumbs[i];
      if (Math.hypot(crumb.x - start.x, crumb.y - start.y) > maxDisplacement) break;
      points.push(crumb);
    }
    if (points.length === 0) {
      logger.info(`Started (${reason}), but there is no recorded path to walk back`);
      return;
    }
    const maxDuration = parameters.hasMaxDuration() ? durationToSec(parameters.getMaxDuration()) : 600;
    const endTime = this.robot.clock.now() + maxDuration;
    this.running = { endTime };
    logger.info(`Walking back ${points.length} points of the path (${reason})`);
    body.start({ kind: 'trajectory', owner: 'auto_return', points, endTime, maxVel: null, bodyControl: null });
  }

  /** A command took the mobility. */
  onMobilityOverridden() {
    if (this.running && this.robot.body.behavior.owner === 'auto_return') {
      logger.info('Interrupted by a command');
      this.running = null;
    }
  }

  /**
   * Triggers when the client of the configuration stops controlling the robot, stops at the end of the path.
   * @param {number} now
   */
  update(now) {
    const { body } = this.robot;
    if (this.running) {
      const done =
        body.behavior.owner !== 'auto_return' || body.behavior.state.status === 'stopped' || now > this.running.endTime;
      if (done) {
        logger.info('Finished');
        this.running = null;
      }
      return;
    }
    if (!this.enabled || !body.isStanding() || this.robot.docking.isDocked()) return;
    // The comms loss of the client: its lease is stale.
    const lease = leaseFromProto(this.request.getLeasesList()[0]);
    const node = this.robot.leases.nodes.get(lease?.resource);
    const leaf = node ? this.robot.leases.leaves.get(node.leaves[0]) : null;
    const lost = Boolean(leaf?.stale && leaf.active?.sequence[0] === lease.sequence[0]);
    // Once per configuration: after the return, the comms loss behaviors of the robot take over (E-Stop...).
    if (lost && this._triggeredRequest !== this.request) {
      this._triggeredRequest = this.request;
      this.trigger(this.request.getLeasesList().map(leaseFromProto), 'the client stopped controlling the robot');
    }
  }

  /** @returns {string} */
  describe() {
    if (!this.enabled) return 'disabled';
    return `${this.running ? 'returning' : 'enabled'}, ${this.breadcrumbs.length} points recorded`;
  }
}

module.exports = { AutoReturn };
