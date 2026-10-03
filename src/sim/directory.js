'use strict';

const { secToTimestamp } = require('./clock');
const directoryPb = require('../bosdyn/api/directory_pb');
const directoryRegistrationPb = require('../bosdyn/api/directory_registration_pb');
const serviceFaultPb = require('../bosdyn/api/service_fault_pb');
const { LoggerUtil } = require('../loggerUtil');

const logger = LoggerUtil.getLogger('DIRECTORY');

// Name of the service faults of the services which stop their heartbeats (the name of a real robot is not documented).
const LIVENESS_FAULT_NAME = 'Liveness timeout';

/**
 * The directory of the services of the robot: the built-in services of the simulator, and the services that payloads
 * register (they are listed, but the simulator does not route their RPCs).
 */
class Directory {
  /**
   * @param {import('../robot')} robot
   */
  constructor(robot) {
    this.robot = robot;
    /** @type {Map<string, {name: string, type: string, authority: string, userTokenRequired: boolean}>} */
    this.builtin = new Map();
    /** @type {Map<string, {entry: directoryPb.ServiceEntry, endpoint: ?directoryPb.Endpoint, lastUpdate: number}>} */
    this.registered = new Map();
    this.startTime = robot.clock.now();
  }

  /**
   * Adds a built-in service.
   * @param {{name: string, type: string, authority: string, userTokenRequired?: boolean}} entry
   */
  addBuiltin(entry) {
    this.builtin.set(entry.name, { userTokenRequired: true, ...entry });
  }

  /** @returns {directoryPb.ServiceEntry[]} */
  list() {
    const builtin = [...this.builtin.values()].map(entry =>
      new directoryPb.ServiceEntry()
        .setName(entry.name)
        .setType(entry.type)
        .setAuthority(entry.authority)
        .setUserTokenRequired(entry.userTokenRequired)
        .setLastUpdate(secToTimestamp(this.startTime)),
    );
    const registered = [...this.registered.values()].map(({ entry, lastUpdate }) =>
      entry.clone().setLastUpdate(secToTimestamp(lastUpdate)),
    );
    return [...builtin, ...registered].sort((a, b) => a.getName().localeCompare(b.getName()));
  }

  /**
   * @param {string} name
   * @returns {?directoryPb.ServiceEntry}
   */
  get(name) {
    return this.list().find(entry => entry.getName() === name) ?? null;
  }

  /**
   * RegisterService.
   * @param {directoryPb.ServiceEntry} entry
   * @param {?directoryPb.Endpoint} endpoint
   * @returns {number} The status.
   */
  register(entry, endpoint) {
    const Status = directoryRegistrationPb.RegisterServiceResponse.Status;
    const name = entry.getName();
    if (this.builtin.has(name)) return Status.STATUS_ALREADY_EXISTS;
    const existing = this.registered.get(name);
    if (existing) {
      // The keep-alive of the SDK registers again and again: each registration is a heartbeat of the service.
      existing.lastUpdate = this.robot.clock.now();
      return Status.STATUS_ALREADY_EXISTS;
    }
    this.registered.set(name, {
      entry: entry.clone(),
      endpoint: endpoint?.clone() ?? null,
      lastUpdate: this.robot.clock.now(),
    });
    logger.info(
      `Service "${name}" (${entry.getType()}) registered at ${endpoint?.getHostIp()}:${endpoint?.getPort()}` +
        ' (the simulator lists it, but does not route its RPCs)',
    );
    return Status.STATUS_OK;
  }

  /**
   * UpdateService.
   * @param {directoryPb.ServiceEntry} entry
   * @param {?directoryPb.Endpoint} endpoint
   * @returns {number} The status.
   */
  update(entry, endpoint) {
    const Status = directoryRegistrationPb.UpdateServiceResponse.Status;
    const registered = this.registered.get(entry.getName());
    if (!registered) return Status.STATUS_NONEXISTENT_SERVICE;
    registered.entry = entry.clone();
    if (endpoint) registered.endpoint = endpoint.clone();
    registered.lastUpdate = this.robot.clock.now();
    return Status.STATUS_OK;
  }

  /**
   * UnregisterService.
   * @param {string} name
   * @returns {number} The status.
   */
  unregister(name) {
    const Status = directoryRegistrationPb.UnregisterServiceResponse.Status;
    const registered = this.registered.get(name);
    if (!registered) return Status.STATUS_NONEXISTENT_SERVICE;
    this.registered.delete(name);
    // Unregistering clears the liveness fault.
    if (registered.faulted) this.robot.faults.clearServiceFault(this._livenessFaultId(registered.entry), false, false);
    logger.info(`Service "${name}" unregistered`);
    return Status.STATUS_OK;
  }

  /**
   * @param {directoryPb.ServiceEntry} entry
   * @returns {serviceFaultPb.ServiceFaultId}
   */
  _livenessFaultId(entry) {
    return new serviceFaultPb.ServiceFaultId()
      .setFaultName(LIVENESS_FAULT_NAME)
      .setServiceName(entry.getName())
      .setPayloadGuid(entry.getHostPayloadGuid());
  }

  /**
   * Liveness of the registered services: a service with a liveness timeout which stops its heartbeats (the
   * re-registrations of the directory keep-alive of the SDK) gets a service fault, cleared by its next heartbeat.
   * @param {number} now
   */
  checkLiveness(now) {
    for (const registered of this.registered.values()) {
      const timeout = registered.entry.getLivenessTimeoutSecs();
      if (!(timeout > 0)) continue;
      const late = now - registered.lastUpdate > timeout;
      if (late && !registered.faulted) {
        registered.faulted = true;
        const name = registered.entry.getName();
        this.robot.faults.triggerServiceFault(
          new serviceFaultPb.ServiceFault()
            .setFaultId(this._livenessFaultId(registered.entry))
            .setErrorMessage(`Service "${name}" did not send a heartbeat for ${timeout} s.`)
            .setAttributesList(['liveness'])
            .setSeverity(serviceFaultPb.ServiceFault.Severity.SEVERITY_CRITICAL),
        );
      } else if (!late && registered.faulted) {
        registered.faulted = false;
        this.robot.faults.clearServiceFault(this._livenessFaultId(registered.entry), false, false);
      }
    }
  }
}

module.exports = { Directory };
