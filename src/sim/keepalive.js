'use strict';

const { durationToSec, secToTimestamp } = require('./clock');
const { CompareResult, compareLeases, leaseFromProto } = require('./lease');
const keepalivePb = require('../bosdyn/api/keepalive/keepalive_pb');
const { LoggerUtil } = require('../loggerUtil');

const logger = LoggerUtil.getLogger('KEEPALIVE');

const { ActionAfter } = keepalivePb;
const { PolicyControlAction } = keepalivePb.GetStatusResponse;

/**
 * The keepalive service: policies of actions taken when a client stops checking in (record events, auto return,
 * motors off, robot off, stale leases, halt). Removing the policies associated with a lease when its owner changes.
 */
class KeepaliveManager {
  /**
   * @param {import('../robot')} robot
   */
  constructor(robot) {
    this.robot = robot;
    this.nextId = 1n + BigInt(Math.floor(Math.random() * 1000));
    /** @type {Map<string, {id: string, proto: keepalivePb.Policy, actions: {afterSec: number, action:
     *   ActionAfter}[], leases: import('./lease').LeaseData[], lastCheckin: number, clientName: string,
     *   triggered: Set<number>}>} */
    this.policies = new Map();
    robot.on('lease:change', () => this._removeObsoletePolicies());
  }

  /** Forgets the policies (reboot). */
  reset() {
    this.policies.clear();
  }

  /**
   * ModifyPolicy.
   * @param {?keepalivePb.Policy} toAdd
   * @param {string[]} idsToRemove
   * @param {string} clientName
   * @returns {{status: number, added: ?object, removed: object[], error?: string}}
   */
  modify(toAdd, idsToRemove, clientName) {
    const Status = keepalivePb.ModifyPolicyResponse.Status;
    if (idsToRemove.some(id => !this.policies.has(String(id)))) {
      return { status: Status.STATUS_INVALID_POLICY_ID, added: null, removed: [] };
    }
    let added = null;
    if (toAdd) {
      const leases = toAdd.getAssociatedLeasesList().map(lease => leaseFromProto(lease));
      for (const lease of leases) {
        if (!this._isActiveLeaseFamily(lease)) return { status: Status.STATUS_INVALID_LEASE, added: null, removed: [] };
      }
      const actions = [];
      for (const action of toAdd.getActionsList()) {
        if (action.getActionCase() === ActionAfter.ActionCase.ACTION_NOT_SET || !action.hasAfter()) {
          return { status: Status.STATUS_UNKNOWN, added: null, removed: [], error: 'Invalid action in the policy.' };
        }
        actions.push({ afterSec: durationToSec(action.getAfter()), action });
      }
      added = {
        id: String(this.nextId++),
        proto: toAdd.clone(),
        actions,
        leases,
        lastCheckin: this.robot.clock.now(),
        clientName,
        triggered: new Set(),
      };
    }
    const removed = idsToRemove.map(id => this.policies.get(String(id)));
    for (const policy of removed) this.policies.delete(policy.id);
    if (added) {
      this.policies.set(added.id, added);
      logger.info(`Policy ${added.id} "${toAdd.getName()}" added by "${clientName}"`);
    }
    for (const policy of removed) logger.info(`Policy ${policy.id} removed`);
    return { status: Status.STATUS_OK, added, removed };
  }

  /**
   * The lease is the same, a sub lease or a super lease of an active lease.
   * @param {import('./lease').LeaseData} lease
   * @returns {boolean}
   */
  _isActiveLeaseFamily(lease) {
    const { leases } = this.robot;
    const node = leases.nodes.get(lease?.resource);
    if (!node || lease.epoch !== leases.epoch) return false;
    const leaf = leases.leaves.get(node.leaves[0]);
    if (!leaf.owner || !leaf.active) return false;
    const cmp = compareLeases(lease, leaf.active);
    return cmp === CompareResult.SAME || cmp === CompareResult.SUB_LEASE || cmp === CompareResult.SUPER_LEASE;
  }

  _removeObsoletePolicies() {
    for (const [id, policy] of this.policies) {
      if (policy.leases.length > 0 && !policy.leases.every(lease => this.robot.leases.isStillOwner(lease))) {
        this.policies.delete(id);
        logger.info(`Policy ${id} removed: its lease has a new owner`);
      }
    }
  }

  /**
   * CheckIn.
   * @param {string} id
   * @param {string} clientName
   * @returns {?number} The time of the check-in, null for an unknown policy.
   */
  checkIn(id, clientName) {
    const policy = this.policies.get(String(id));
    if (!policy) return null;
    policy.lastCheckin = this.robot.clock.now();
    policy.clientName = clientName;
    policy.triggered.clear();
    return policy.lastCheckin;
  }

  /**
   * @returns {boolean} Whether a policy with a motors off action has triggered (the motors cannot power on).
   */
  motorsOffActionActive() {
    return this.activeControlActions().some(
      action =>
        action === PolicyControlAction.POLICY_CONTROL_ACTION_MOTORS_OFF ||
        action === PolicyControlAction.POLICY_CONTROL_ACTION_IMMEDIATE_MOTORS_OFF ||
        action === PolicyControlAction.POLICY_CONTROL_ACTION_ROBOT_OFF,
    );
  }

  /**
   * @returns {number[]} The PolicyControlAction of the triggered actions which control the robot.
   */
  activeControlActions() {
    const actions = new Set();
    for (const policy of this.policies.values()) {
      for (const index of policy.triggered) {
        const action = policy.actions[index].action;
        if (action.hasAutoReturn()) actions.add(PolicyControlAction.POLICY_CONTROL_ACTION_AUTO_RETURN);
        if (action.hasControlledMotorsOff()) actions.add(PolicyControlAction.POLICY_CONTROL_ACTION_MOTORS_OFF);
        if (action.hasImmediateMotorsOff()) actions.add(PolicyControlAction.POLICY_CONTROL_ACTION_IMMEDIATE_MOTORS_OFF);
        if (action.hasImmediateRobotOff()) actions.add(PolicyControlAction.POLICY_CONTROL_ACTION_ROBOT_OFF);
        if (action.hasHaltRobot()) actions.add(PolicyControlAction.POLICY_CONTROL_ACTION_HALT);
      }
    }
    return [...actions];
  }

  /**
   * Triggers the actions of the policies which were not checked into for long enough.
   * @param {number} now
   */
  update(now) {
    for (const policy of this.policies.values()) {
      policy.actions.forEach(({ afterSec, action }, index) => {
        if (policy.triggered.has(index) || now - policy.lastCheckin < afterSec) return;
        policy.triggered.add(index);
        this._trigger(policy, action);
      });
      // The motors off actions hold while the policy is not checked into.
      for (const index of policy.triggered) {
        const action = policy.actions[index].action;
        if (action.hasControlledMotorsOff()) this.robot.power.settleThenCut(`keepalive policy ${policy.id}`);
        if (action.hasImmediateMotorsOff()) this.robot.power.cutMotorPower(`keepalive policy ${policy.id}`);
      }
    }
  }

  _trigger(policy, action) {
    const name = `policy ${policy.id} "${policy.proto.getName()}"`;
    if (action.hasRecordEvent()) {
      logger.info(`${name}: recording events`);
      this.robot.dataBuffer.recordEvents(action.getRecordEvent().getEventsList());
    } else if (action.hasAutoReturn()) {
      logger.info(`${name}: starting auto return`);
      this.robot.autoReturn.trigger(action.getAutoReturn().getLeasesList().map(leaseFromProto), 'keepalive');
    } else if (action.hasControlledMotorsOff()) {
      logger.info(`${name}: sitting down and powering off the motors`);
      this.robot.power.settleThenCut(`keepalive ${name}`);
    } else if (action.hasImmediateMotorsOff()) {
      logger.info(`${name}: cutting the motor power`);
      this.robot.power.cutMotorPower(`keepalive ${name}`);
    } else if (action.hasImmediateRobotOff()) {
      logger.info(`${name}: powering off the robot`);
      this.robot.power.cutMotorPower(`keepalive ${name}`);
      this.robot.scheduleShutdown({ reboot: false, delaySec: 0 });
    } else if (action.hasLeaseStale()) {
      logger.info(`${name}: marking the leases stale`);
      this.robot.leases.markStale(action.getLeaseStale().getLeasesList().map(leaseFromProto));
    } else if (action.hasHaltRobot()) {
      logger.info(`${name}: halting the robot`);
      this.robot.commands.halt(`keepalive ${name}`);
    }
  }

  /**
   * @param {object} policy
   * @returns {keepalivePb.LivePolicy}
   */
  static livePolicyToProto(policy) {
    return new keepalivePb.LivePolicy()
      .setPolicyId(policy.id)
      .setPolicy(policy.proto)
      .setLastCheckin(secToTimestamp(policy.lastCheckin))
      .setClientName(policy.clientName);
  }

  /**
   * @returns {keepalivePb.GetStatusResponse}
   */
  statusToProto() {
    return new keepalivePb.GetStatusResponse()
      .setStatusList([...this.policies.values()].map(policy => KeepaliveManager.livePolicyToProto(policy)))
      .setActiveControlActionsList(this.activeControlActions());
  }
}

module.exports = { KeepaliveManager };
