'use strict';

const { randomUUID } = require('node:crypto');

const { secToDuration, secToTimestamp } = require('./clock');
const robotStatePb = require('../bosdyn/api/robot_state_pb');
const serviceFaultPb = require('../bosdyn/api/service_fault_pb');
const { LoggerUtil } = require('../loggerUtil');

const logger = LoggerUtil.getLogger('FAULTS');

const { BehaviorFault, SystemFault } = robotStatePb;

// Cleared faults kept in the history of the robot state.
const MAX_HISTORY = 20;

/**
 * @param {serviceFaultPb.ServiceFaultId} id
 * @returns {string}
 */
function serviceFaultKey(id) {
  return `${id.getFaultName()}\u0000${id.getServiceName()}\u0000${id.getPayloadGuid()}`;
}

/**
 * The faults of the robot: system faults (hardware, battery...), behavior faults (a fall...) which block the robot
 * commands until they are cleared, and the service faults that payloads and services report with the fault service.
 */
class FaultManager {
  /**
   * @param {import('../robot')} robot
   */
  constructor(robot) {
    /**
     * @type {import('../robot')}
     */
    this.robot = robot;
    /** @type {Map<string, {name: string, onset: number, code: number, uuid: string, message: string,
     *   attributes: string[], severity: number, blocking: boolean}>} */
    this.systemFaults = new Map();
    this.systemHistory = [];
    /** @type {Map<number, {id: number, onset: number, cause: number, status: number}>} */
    this.behaviorFaults = new Map();
    this.nextBehaviorFaultId = 1 + Math.floor(Math.random() * 100);
    /** @type {Map<string, {proto: serviceFaultPb.ServiceFault, onset: number}>} */
    this.serviceFaults = new Map();
    this.serviceHistory = [];
  }

  /**
   * Raises a system fault (once: a fault already active keeps its onset).
   * @param {string} name
   * @param {object} [options]
   * @param {string} [options.message]
   * @param {number} [options.severity] A SystemFault.Severity.
   * @param {string[]} [options.attributes] E.g. ['battery'], ['vision'].
   * @param {number} [options.code]
   * @param {boolean} [options.blocking] The fault prevents the motors from powering on.
   */
  addSystemFault(
    name,
    { message = '', severity = SystemFault.Severity.SEVERITY_WARN, attributes = [], code = 0, blocking = false } = {},
  ) {
    if (this.systemFaults.has(name)) return;
    this.systemFaults.set(name, {
      name,
      onset: this.robot.clock.now(),
      code,
      uuid: randomUUID(),
      message,
      attributes,
      severity,
      blocking,
    });
    logger.warn(`System fault "${name}": ${message}`);
  }

  /**
   * @param {string} name
   * @returns {boolean} Whether the fault was active.
   */
  clearSystemFault(name) {
    const fault = this.systemFaults.get(name);
    if (!fault) return false;
    this.systemFaults.delete(name);
    this.systemHistory.push({ ...fault, cleared: this.robot.clock.now() });
    if (this.systemHistory.length > MAX_HISTORY) this.systemHistory.shift();
    logger.info(`System fault "${name}" cleared`);
    return true;
  }

  /**
   * @returns {SystemFault[]} The active faults which prevent the motors from powering on.
   */
  blockingSystemFaults() {
    const now = this.robot.clock.now();
    return [...this.systemFaults.values()]
      .filter(fault => fault.blocking)
      .map(fault => this._systemFaultToProto(fault, now));
  }

  /**
   * Raises a behavior fault: the robot refuses the commands (STATUS_BEHAVIOR_FAULT) until it is cleared.
   * @param {number} cause A BehaviorFault.Cause.
   * @param {boolean} [clearable=true]
   * @returns {number} The id of the fault.
   */
  addBehaviorFault(cause, clearable = true) {
    const id = this.nextBehaviorFaultId++;
    this.behaviorFaults.set(id, {
      id,
      onset: this.robot.clock.now(),
      cause,
      status: clearable ? BehaviorFault.Status.STATUS_CLEARABLE : BehaviorFault.Status.STATUS_UNCLEARABLE,
    });
    const causeName = Object.keys(BehaviorFault.Cause).find(key => BehaviorFault.Cause[key] === cause);
    logger.warn(`Behavior fault ${id} (${causeName})`);
    return id;
  }

  /** @returns {boolean} */
  hasBehaviorFaults() {
    return this.behaviorFaults.size > 0;
  }

  /**
   * ClearBehaviorFault.
   * @param {number} id
   * @returns {{cleared: boolean, fault: ?BehaviorFault, blockingSystemFaults: SystemFault[]}}
   */
  clearBehaviorFault(id) {
    const fault = this.behaviorFaults.get(id);
    if (!fault) return { cleared: false, fault: null, blockingSystemFaults: [] };
    const proto = this._behaviorFaultToProto(fault);
    if (fault.status !== BehaviorFault.Status.STATUS_CLEARABLE) {
      return { cleared: false, fault: proto, blockingSystemFaults: this.blockingSystemFaults() };
    }
    this.behaviorFaults.delete(id);
    logger.info(`Behavior fault ${id} cleared`);
    return { cleared: true, fault: proto, blockingSystemFaults: [] };
  }

  /**
   * Clears the behavior faults of a cause (e.g. the fall, once the robot has self-righted).
   * @param {number} cause
   */
  clearBehaviorFaultsOfCause(cause) {
    for (const [id, fault] of this.behaviorFaults) {
      if (fault.cause === cause) this.behaviorFaults.delete(id);
    }
  }

  /**
   * TriggerServiceFault.
   * @param {serviceFaultPb.ServiceFault} fault
   * @returns {number} The status.
   */
  triggerServiceFault(fault) {
    const Status = serviceFaultPb.TriggerServiceFaultResponse.Status;
    const key = serviceFaultKey(fault.getFaultId() ?? new serviceFaultPb.ServiceFaultId());
    if (this.serviceFaults.has(key)) return Status.STATUS_FAULT_ALREADY_ACTIVE;
    this.serviceFaults.set(key, { proto: fault.clone(), onset: this.robot.clock.now() });
    logger.warn(`Service fault "${fault.getFaultId()?.getFaultName()}" of "${fault.getFaultId()?.getServiceName()}"`);
    return Status.STATUS_OK;
  }

  /**
   * ClearServiceFault.
   * @param {serviceFaultPb.ServiceFaultId} id
   * @param {boolean} clearAllServiceFaults Clears all the faults of the service of the id.
   * @param {boolean} clearAllPayloadFaults Clears all the faults of the payload of the id.
   * @returns {number} The status.
   */
  clearServiceFault(id, clearAllServiceFaults, clearAllPayloadFaults) {
    const Status = serviceFaultPb.ClearServiceFaultResponse.Status;
    const matches = entry => {
      const entryId = entry.proto.getFaultId();
      if (clearAllServiceFaults && id.getServiceName() && entryId.getServiceName() === id.getServiceName()) return true;
      if (clearAllPayloadFaults && id.getPayloadGuid() && entryId.getPayloadGuid() === id.getPayloadGuid()) return true;
      return serviceFaultKey(entryId) === serviceFaultKey(id);
    };
    let cleared = false;
    for (const [key, entry] of this.serviceFaults) {
      if (matches(entry)) {
        this.serviceFaults.delete(key);
        this.serviceHistory.push({ ...entry, cleared: this.robot.clock.now() });
        if (this.serviceHistory.length > MAX_HISTORY) this.serviceHistory.shift();
        cleared = true;
      }
    }
    return cleared ? Status.STATUS_OK : Status.STATUS_FAULT_NOT_ACTIVE;
  }

  _systemFaultToProto(fault, now) {
    return new SystemFault()
      .setName(fault.name)
      .setOnsetTimestamp(secToTimestamp(fault.onset))
      .setDuration(secToDuration((fault.cleared ?? now) - fault.onset))
      .setCode(fault.code)
      .setUuid(fault.uuid)
      .setErrorMessage(fault.message)
      .setAttributesList(fault.attributes)
      .setSeverity(fault.severity);
  }

  _behaviorFaultToProto(fault) {
    return new BehaviorFault()
      .setBehaviorFaultId(fault.id)
      .setOnsetTimestamp(secToTimestamp(fault.onset))
      .setCause(fault.cause)
      .setStatus(fault.status);
  }

  /**
   * @returns {robotStatePb.SystemFaultState}
   */
  systemFaultStateToProto() {
    const now = this.robot.clock.now();
    const state = new robotStatePb.SystemFaultState()
      .setFaultsList([...this.systemFaults.values()].map(fault => this._systemFaultToProto(fault, now)))
      .setHistoricalFaultsList(this.systemHistory.map(fault => this._systemFaultToProto(fault, now)));
    const aggregated = state.getAggregatedMap();
    for (const fault of this.systemFaults.values()) {
      for (const attribute of fault.attributes) {
        aggregated.set(attribute, Math.max(aggregated.get(attribute) ?? 0, fault.severity));
      }
    }
    return state;
  }

  /**
   * @returns {robotStatePb.BehaviorFaultState}
   */
  behaviorFaultStateToProto() {
    return new robotStatePb.BehaviorFaultState().setFaultsList(
      [...this.behaviorFaults.values()].map(fault => this._behaviorFaultToProto(fault)),
    );
  }

  /**
   * @returns {robotStatePb.ServiceFaultState}
   */
  serviceFaultStateToProto() {
    const now = this.robot.clock.now();
    const toProto = entry =>
      entry.proto
        .clone()
        .setOnsetTimestamp(secToTimestamp(entry.onset))
        .setDuration(secToDuration((entry.cleared ?? now) - entry.onset));
    const state = new robotStatePb.ServiceFaultState()
      .setFaultsList([...this.serviceFaults.values()].map(toProto))
      .setHistoricalFaultsList(this.serviceHistory.map(toProto));
    const aggregated = state.getAggregatedMap();
    for (const entry of this.serviceFaults.values()) {
      for (const attribute of entry.proto.getAttributesList()) {
        aggregated.set(attribute, Math.max(aggregated.get(attribute) ?? 0, entry.proto.getSeverity()));
      }
    }
    return state;
  }

  /**
   * A summary for the console.
   * @returns {string[]}
   */
  describe() {
    const lines = [];
    for (const fault of this.systemFaults.values()) lines.push(`system fault "${fault.name}": ${fault.message}`);
    for (const fault of this.behaviorFaults.values()) {
      const cause = Object.keys(BehaviorFault.Cause).find(key => BehaviorFault.Cause[key] === fault.cause);
      lines.push(`behavior fault ${fault.id} (${cause})`);
    }
    for (const entry of this.serviceFaults.values()) {
      lines.push(
        `service fault "${entry.proto.getFaultId()?.getFaultName()}" ` +
          `of "${entry.proto.getFaultId()?.getServiceName()}"`,
      );
    }
    return lines;
  }
}

module.exports = { FaultManager };
