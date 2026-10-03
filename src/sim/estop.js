'use strict';

const { randomBytes } = require('node:crypto');

const { durationToSec, secToDuration } = require('./clock');
const estopPb = require('../bosdyn/api/estop_pb');
const robotStatePb = require('../bosdyn/api/robot_state_pb');
const { LoggerUtil } = require('../loggerUtil');

const logger = LoggerUtil.getLogger('ESTOP');

const { EstopStopLevel } = estopPb;
const LEVEL_NAMES = {
  [EstopStopLevel.ESTOP_LEVEL_UNKNOWN]: 'UNKNOWN',
  [EstopStopLevel.ESTOP_LEVEL_CUT]: 'CUT',
  [EstopStopLevel.ESTOP_LEVEL_SETTLE_THEN_CUT]: 'SETTLE_THEN_CUT',
  [EstopStopLevel.ESTOP_LEVEL_NONE]: 'NONE',
};

// The role of the endpoint checked by the firmware of the power distribution board.
const PRIMARY_ROLE = 'PDB_rooted';

/**
 * @returns {string} A random identifier.
 */
function newId() {
  return randomBytes(8).toString('hex');
}

/**
 * @returns {string} A random uint64, as a decimal string (the challenges are exact uint64 for the SDK).
 */
function newChallenge() {
  return randomBytes(8).readBigUInt64BE().toString();
}

/**
 * @param {string} challenge
 * @returns {string} The expected response: the one's complement of the challenge.
 */
function expectedResponse(challenge) {
  return BigInt.asUintN(64, ~BigInt(challenge)).toString();
}

/**
 * The most restrictive of two stop levels (CUT < SETTLE_THEN_CUT < NONE).
 * @param {number} a
 * @param {number} b
 * @returns {number}
 */
function mostRestrictive(a, b) {
  return Math.min(a, b);
}

/**
 * The E-Stop service of the robot: a configuration of endpoints (roles and timeouts), the registered endpoints, their
 * check-ins (challenge and response), and the resulting stop level. The hardware E-Stop of the robot can be pressed
 * from the console of the simulator.
 *
 * Like a real robot, the robot is E-Stopped (CUT) until an endpoint is registered for each role of the configuration
 * and checks in with ESTOP_LEVEL_NONE. An endpoint which does not check in for its timeout asserts SETTLE_THEN_CUT
 * (the robot sits down, then the motor power is cut), then CUT after its cut power timeout.
 */
class EstopSystem {
  /**
   * @param {import('../robot')} robot
   */
  constructor(robot) {
    this.robot = robot;
    this.hardwareEstopped = false;
    this.payloadEstopped = false;
    this.reset();
  }

  /** The configuration of a booting robot: one primary endpoint, not registered. */
  reset() {
    this.config = {
      uniqueId: newId(),
      endpoints: [{ role: PRIMARY_ROLE, name: '', uniqueId: '', timeoutSec: 10, cutPowerTimeoutSec: null }],
    };
    /** @type {Map<string, {endpoint: object, challenge: ?string, lastValid: ?number, stopLevel: number}>} */
    this.registrations = new Map();
    this._lastLevel = null;
  }

  /**
   * @param {estopPb.EstopEndpoint} proto
   * @returns {{role: string, name: string, uniqueId: string, timeoutSec: ?number, cutPowerTimeoutSec: ?number}}
   */
  static endpointFromProto(proto) {
    return {
      role: proto?.getRole() ?? '',
      name: proto?.getName() ?? '',
      uniqueId: proto?.getUniqueId() ?? '',
      timeoutSec: proto?.hasTimeout() ? durationToSec(proto.getTimeout()) : null,
      cutPowerTimeoutSec: proto?.hasCutPowerTimeout() ? durationToSec(proto.getCutPowerTimeout()) : null,
    };
  }

  /**
   * @param {object} endpoint
   * @returns {estopPb.EstopEndpoint}
   */
  static endpointToProto(endpoint) {
    const proto = new estopPb.EstopEndpoint()
      .setRole(endpoint.role)
      .setName(endpoint.name)
      .setUniqueId(endpoint.uniqueId);
    if (endpoint.timeoutSec !== null) proto.setTimeout(secToDuration(endpoint.timeoutSec));
    if (endpoint.cutPowerTimeoutSec !== null) proto.setCutPowerTimeout(secToDuration(endpoint.cutPowerTimeoutSec));
    return proto;
  }

  /**
   * @returns {estopPb.EstopConfig}
   */
  configToProto() {
    const endpoints = this.config.endpoints.map(endpoint => {
      const registered = this.registrations.get(endpoint.role)?.endpoint;
      return EstopSystem.endpointToProto(registered ?? endpoint);
    });
    return new estopPb.EstopConfig().setUniqueId(this.config.uniqueId).setEndpointsList(endpoints);
  }

  /**
   * SetEstopConfig.
   * @param {estopPb.EstopConfig} configProto
   * @param {string} targetConfigId
   * @returns {{status: number, error?: string}}
   */
  setConfig(configProto, targetConfigId) {
    const Status = estopPb.SetEstopConfigResponse.Status;
    if (this.robot.power.motorsOnOrPowering()) return { status: Status.STATUS_MOTORS_ON };
    if (targetConfigId !== this.config.uniqueId) return { status: Status.STATUS_INVALID_ID };
    const endpoints = configProto.getEndpointsList().map(proto => EstopSystem.endpointFromProto(proto));
    const roles = new Set();
    for (const endpoint of endpoints) {
      if (!endpoint.role) {
        return { status: Status.STATUS_UNKNOWN, error: 'An endpoint of the configuration has no role.' };
      }
      if (roles.has(endpoint.role)) {
        return {
          status: Status.STATUS_UNKNOWN,
          error: `Two endpoints of the configuration have the role "${endpoint.role}".`,
        };
      }
      roles.add(endpoint.role);
      if (!(endpoint.timeoutSec > 0)) {
        return { status: Status.STATUS_UNKNOWN, error: `The endpoint "${endpoint.role}" has no timeout.` };
      }
      if (endpoint.role === PRIMARY_ROLE && endpoint.timeoutSec > 65530) {
        return {
          status: Status.STATUS_UNKNOWN,
          error: 'The PDB_rooted endpoint must have a timeout of 65530 s or less.',
        };
      }
      endpoint.uniqueId = '';
    }
    this.config = { uniqueId: newId(), endpoints };
    // Setting a configuration forgets the registered endpoints.
    this.registrations.clear();
    logger.info(`New E-Stop configuration ${this.config.uniqueId} with roles: ${[...roles].join(', ') || 'none'}`);
    return { status: Status.STATUS_SUCCESS };
  }

  /**
   * RegisterEstopEndpoint.
   * @param {string} targetConfigId
   * @param {estopPb.EstopEndpoint} targetProto The endpoint to replace (its role, and its unique id if registered).
   * @param {estopPb.EstopEndpoint} newProto
   * @returns {{status: number, endpoint?: object}}
   */
  register(targetConfigId, targetProto, newProto) {
    const Status = estopPb.RegisterEstopEndpointResponse.Status;
    if (targetConfigId !== this.config.uniqueId) return { status: Status.STATUS_CONFIG_MISMATCH };
    const target = EstopSystem.endpointFromProto(targetProto);
    const configured = this.config.endpoints.find(endpoint => endpoint.role === target.role);
    if (!configured) return { status: Status.STATUS_ENDPOINT_MISMATCH };
    const existing = this.registrations.get(target.role);
    if (target.uniqueId ? existing?.endpoint.uniqueId !== target.uniqueId : existing) {
      return { status: Status.STATUS_ENDPOINT_MISMATCH };
    }
    const endpoint = EstopSystem.endpointFromProto(newProto);
    if (endpoint.role && endpoint.role !== target.role) return { status: Status.STATUS_INVALID_ENDPOINT };
    endpoint.role = target.role;
    endpoint.timeoutSec ??= configured.timeoutSec;
    endpoint.cutPowerTimeoutSec ??= configured.cutPowerTimeoutSec;
    if (!(endpoint.timeoutSec > 0)) return { status: Status.STATUS_INVALID_ENDPOINT };
    if (endpoint.cutPowerTimeoutSec !== null && endpoint.cutPowerTimeoutSec < endpoint.timeoutSec) {
      return { status: Status.STATUS_INVALID_ENDPOINT };
    }
    endpoint.uniqueId = newId();
    this.registrations.set(endpoint.role, {
      endpoint,
      challenge: null,
      lastValid: null,
      stopLevel: EstopStopLevel.ESTOP_LEVEL_CUT,
    });
    logger.info(
      `Endpoint "${endpoint.name}" registered for role "${endpoint.role}" (timeout ${endpoint.timeoutSec} s)${
        existing ? `, replacing "${existing.endpoint.name}"` : ''
      }`,
    );
    return { status: Status.STATUS_SUCCESS, endpoint };
  }

  /**
   * DeregisterEstopEndpoint.
   * @param {string} targetConfigId
   * @param {estopPb.EstopEndpoint} targetProto
   * @returns {number} The status.
   */
  deregister(targetConfigId, targetProto) {
    const Status = estopPb.DeregisterEstopEndpointResponse.Status;
    if (targetConfigId !== this.config.uniqueId) return Status.STATUS_CONFIG_MISMATCH;
    const target = EstopSystem.endpointFromProto(targetProto);
    const registration = [...this.registrations.values()].find(
      reg => reg.endpoint.uniqueId === target.uniqueId && (!target.role || reg.endpoint.role === target.role),
    );
    if (!registration) return Status.STATUS_ENDPOINT_MISMATCH;
    if (this.robot.power.motorsOnOrPowering()) return Status.STATUS_MOTORS_ON;
    this.registrations.delete(registration.endpoint.role);
    logger.info(`Endpoint "${registration.endpoint.name}" deregistered`);
    return Status.STATUS_SUCCESS;
  }

  /**
   * EstopCheckIn.
   * @param {estopPb.EstopEndpoint} endpointProto
   * @param {string} challenge
   * @param {string} response
   * @param {number} stopLevel
   * @returns {{status: number, challenge: string}}
   */
  checkIn(endpointProto, challenge, response, stopLevel) {
    const Status = estopPb.EstopCheckInResponse.Status;
    const uniqueId = endpointProto?.getUniqueId() ?? '';
    const registration = [...this.registrations.values()].find(reg => reg.endpoint.uniqueId === uniqueId);
    if (!registration) return { status: Status.STATUS_ENDPOINT_UNKNOWN, challenge: newChallenge() };
    const valid =
      registration.challenge !== null &&
      String(challenge) === registration.challenge &&
      String(response) === expectedResponse(registration.challenge);
    const level = Object.values(EstopStopLevel).includes(stopLevel) && stopLevel ? stopLevel : null;
    let status;
    if (valid) {
      registration.lastValid = this.robot.clock.now();
      if (level !== null) this._setEndpointLevel(registration, level);
      status = Status.STATUS_OK;
    } else {
      // An incorrect check-in does not reset the timeout, but a stop is still honored.
      if (level !== null && level < registration.stopLevel) this._setEndpointLevel(registration, level);
      status = Status.STATUS_INCORRECT_CHALLENGE_RESPONSE;
    }
    registration.challenge = newChallenge();
    this.robot.sync();
    return { status, challenge: registration.challenge };
  }

  _setEndpointLevel(registration, level) {
    if (registration.stopLevel !== level) {
      logger.info(`Endpoint "${registration.endpoint.name}" asserts ${LEVEL_NAMES[level]}`);
      registration.stopLevel = level;
    }
  }

  /**
   * The stop level of each configured endpoint.
   * @param {number} now
   * @returns {{role: string, level: number, details: string}[]}
   */
  _endpointLevels(now) {
    return this.config.endpoints.map(configured => {
      const registration = this.registrations.get(configured.role);
      if (!registration) {
        return {
          role: configured.role,
          level: EstopStopLevel.ESTOP_LEVEL_CUT,
          details: `No endpoint registered for role "${configured.role}".`,
        };
      }
      const { endpoint, lastValid } = registration;
      if (lastValid === null) {
        return {
          role: configured.role,
          level: EstopStopLevel.ESTOP_LEVEL_CUT,
          details: `Endpoint "${endpoint.name}" has not checked in.`,
        };
      }
      const elapsed = now - lastValid;
      const cutTimeout = endpoint.cutPowerTimeoutSec ?? endpoint.timeoutSec + this.robot.config.estop.settleTimeSec;
      if (elapsed > cutTimeout) {
        return {
          role: configured.role,
          level: EstopStopLevel.ESTOP_LEVEL_CUT,
          details: `Endpoint "${endpoint.name}" timed out (no valid check-in for ${elapsed.toFixed(1)} s).`,
        };
      }
      if (elapsed > endpoint.timeoutSec) {
        return {
          role: configured.role,
          level: EstopStopLevel.ESTOP_LEVEL_SETTLE_THEN_CUT,
          details: `Endpoint "${endpoint.name}" timed out (no valid check-in for ${elapsed.toFixed(1)} s).`,
        };
      }
      const details =
        registration.stopLevel === EstopStopLevel.ESTOP_LEVEL_NONE
          ? ''
          : `Endpoint "${endpoint.name}" asserts ${LEVEL_NAMES[registration.stopLevel]}.`;
      return { role: configured.role, level: registration.stopLevel, details };
    });
  }

  /**
   * The stop level of the software E-Stop system (the endpoints).
   * @param {number} [now]
   * @returns {{level: number, details: string}}
   */
  softwareLevel(now = this.robot.clock.now()) {
    let level = EstopStopLevel.ESTOP_LEVEL_NONE;
    const details = [];
    for (const endpoint of this._endpointLevels(now)) {
      level = mostRestrictive(level, endpoint.level);
      if (endpoint.details) details.push(endpoint.details);
    }
    return { level, details: details.join(' ') };
  }

  /**
   * The stop level of the whole system: the endpoints and the hardware E-Stops.
   * @param {number} [now]
   * @returns {{level: number, details: string}}
   */
  systemLevel(now = this.robot.clock.now()) {
    const software = this.softwareLevel(now);
    let { level } = software;
    const details = software.details ? [software.details] : [];
    if (this.hardwareEstopped) {
      level = EstopStopLevel.ESTOP_LEVEL_CUT;
      details.unshift('The hardware E-Stop is pressed.');
    }
    if (this.payloadEstopped) {
      level = EstopStopLevel.ESTOP_LEVEL_CUT;
      details.unshift('A payload E-Stop is pressed.');
    }
    return { level, details: details.join(' ') };
  }

  /** @returns {boolean} */
  isEstopped() {
    return this.systemLevel().level !== EstopStopLevel.ESTOP_LEVEL_NONE;
  }

  /**
   * Applies the stop level to the robot: CUT cuts the motor power, SETTLE_THEN_CUT sits the robot down first.
   * @param {number} now
   */
  update(now) {
    const { level, details } = this.systemLevel(now);
    if (level !== this._lastLevel) {
      if (this._lastLevel !== null) logger.info(`Stop level ${LEVEL_NAMES[level]}${details ? `: ${details}` : ''}`);
      this._lastLevel = level;
    }
    if (level === EstopStopLevel.ESTOP_LEVEL_CUT) {
      this.robot.power.cutMotorPower('E-Stop CUT');
    } else if (level === EstopStopLevel.ESTOP_LEVEL_SETTLE_THEN_CUT) {
      this.robot.power.settleThenCut('E-Stop SETTLE_THEN_CUT');
    }
  }

  /**
   * Presses or releases the hardware E-Stop of the robot (console of the simulator).
   * @param {boolean} pressed
   */
  setHardwareEstop(pressed) {
    this.hardwareEstopped = pressed;
    logger.info(`Hardware E-Stop ${pressed ? 'pressed' : 'released'}`);
    this.robot.sync();
  }

  /**
   * @returns {estopPb.EstopSystemStatus}
   */
  systemStatusToProto() {
    const now = this.robot.clock.now();
    const { level, details } = this.systemLevel(now);
    const endpoints = [...this.registrations.values()].map(registration => {
      const status = new estopPb.EstopEndpointWithStatus()
        .setEndpoint(EstopSystem.endpointToProto(registration.endpoint))
        .setStopLevel(registration.stopLevel);
      if (registration.lastValid !== null) {
        status.setTimeSinceValidResponse(secToDuration(now - registration.lastValid));
      }
      return status;
    });
    return new estopPb.EstopSystemStatus().setEndpointsList(endpoints).setStopLevel(level).setStopLevelDetails(details);
  }

  /**
   * The E-Stop states of the robot state.
   * @param {import('google-protobuf/google/protobuf/timestamp_pb').Timestamp} timestamp
   * @returns {robotStatePb.EStopState[]}
   */
  estopStatesToProto(timestamp) {
    const { EStopState } = robotStatePb;
    const software = this.softwareLevel();
    const state = estopped => (estopped ? EStopState.State.STATE_ESTOPPED : EStopState.State.STATE_NOT_ESTOPPED);
    return [
      new EStopState()
        .setTimestamp(timestamp)
        .setName('hardware_estop')
        .setType(EStopState.Type.TYPE_HARDWARE)
        .setState(state(this.hardwareEstopped))
        .setStateDescription(this.hardwareEstopped ? 'The hardware E-Stop is pressed.' : ''),
      new EStopState()
        .setTimestamp(timestamp)
        .setName('payload_estop')
        .setType(EStopState.Type.TYPE_HARDWARE)
        .setState(state(this.payloadEstopped))
        .setStateDescription(this.payloadEstopped ? 'A payload E-Stop is pressed.' : ''),
      new EStopState()
        .setTimestamp(timestamp)
        .setName('software_estop')
        .setType(EStopState.Type.TYPE_SOFTWARE)
        .setState(state(software.level !== EstopStopLevel.ESTOP_LEVEL_NONE))
        .setStateDescription(software.details),
    ];
  }

  /**
   * A summary for the console.
   * @returns {string[]}
   */
  describe() {
    const { level, details } = this.systemLevel();
    const lines = [`stop level ${LEVEL_NAMES[level]}${details ? ` (${details})` : ''}`];
    for (const registration of this.registrations.values()) {
      const since =
        registration.lastValid === null
          ? 'never'
          : `${(this.robot.clock.now() - registration.lastValid).toFixed(1)} s ago`;
      lines.push(
        `endpoint "${registration.endpoint.name}" (${registration.endpoint.role}): ` +
          `${LEVEL_NAMES[registration.stopLevel]}, last valid check-in ${since}`,
      );
    }
    return lines;
  }
}

module.exports = { EstopSystem, LEVEL_NAMES, PRIMARY_ROLE };
