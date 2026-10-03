'use strict';

const { randomBytes } = require('node:crypto');

const { secToTimestamp } = require('./clock');
const leasePb = require('../bosdyn/api/lease_pb');
const { LoggerUtil } = require('../loggerUtil');

const logger = LoggerUtil.getLogger('LEASE');

const { Status: UseStatus } = leasePb.LeaseUseResult;

/**
 * A lease, as plain data.
 * @typedef {{resource: string, epoch: string, sequence: number[], clientNames: string[]}} LeaseData
 */

/** The resources of a Spot with an arm: the "body" covers everything. */
const RESOURCE_TREE_ARM = {
  resource: 'body',
  sub: [{ resource: 'mobility' }, { resource: 'full-arm', sub: [{ resource: 'arm' }, { resource: 'gripper' }] }],
};
const RESOURCE_TREE_NO_ARM = { resource: 'body', sub: [{ resource: 'mobility' }] };

const CompareResult = {
  SAME: 1,
  SUPER_LEASE: 2,
  SUB_LEASE: 3,
  OLDER: 4,
  NEWER: 5,
  DIFFERENT_EPOCHS: 7,
};

/**
 * Compares two leases (the resources are ignored), like Lease.compare() of the SDK.
 * @param {LeaseData} a
 * @param {LeaseData} b
 * @returns {number} How a compares to b.
 */
function compareLeases(a, b) {
  if (a.epoch !== b.epoch) return CompareResult.DIFFERENT_EPOCHS;
  const common = Math.min(a.sequence.length, b.sequence.length);
  for (let i = 0; i < common; i++) {
    if (a.sequence[i] < b.sequence[i]) return CompareResult.OLDER;
    if (a.sequence[i] > b.sequence[i]) return CompareResult.NEWER;
  }
  if (a.sequence.length < b.sequence.length) return CompareResult.SUPER_LEASE;
  if (a.sequence.length > b.sequence.length) return CompareResult.SUB_LEASE;
  return CompareResult.SAME;
}

/**
 * @param {?leasePb.Lease} proto
 * @returns {?LeaseData}
 */
function leaseFromProto(proto) {
  if (!proto) return null;
  return {
    resource: proto.getResource(),
    epoch: proto.getEpoch(),
    sequence: [...proto.getSequenceList()],
    clientNames: [...proto.getClientNamesList()],
  };
}

/**
 * @param {?LeaseData} lease
 * @param {string} [resource] Overrides the resource of the lease.
 * @returns {leasePb.Lease}
 */
function leaseToProto(lease, resource) {
  if (!lease) return new leasePb.Lease();
  return new leasePb.Lease()
    .setResource(resource ?? lease.resource)
    .setEpoch(lease.epoch)
    .setSequenceList([...lease.sequence])
    .setClientNamesList([...lease.clientNames]);
}

/**
 * The lease service of the robot: the resource tree, the ownership of the resources, and the validation of the
 * leases sent with the commands (the rules of the "Lease Service" documentation of the SDK):
 * - the lease must have the epoch of the robot, a known resource covering the resources of the command, and a root
 *   sequence number issued by the robot;
 * - it must be the newest lease of these resources: a lease older than the one of another client (TakeLease) or
 *   than a lease already used is STATUS_OLDER;
 * - a returned lease is STATUS_REVOKED.
 * A lease which is neither retained nor used becomes stale: AcquireLease then succeeds for another client.
 */
class LeaseManager {
  /**
   * @param {import('../robot')} robot
   */
  constructor(robot) {
    this.robot = robot;
    this.tree = robot.config.robot.hasArm ? RESOURCE_TREE_ARM : RESOURCE_TREE_NO_ARM;
    /** @type {Map<string, {resource: string, parent: ?string, leaves: string[]}>} */
    this.nodes = new Map();
    this._indexTree(this.tree, null);
    this.reset();
  }

  /** A new epoch, nothing owned (boot of the robot). */
  reset() {
    this.epoch = randomBytes(9).toString('base64url');
    this.counter = 0;
    /** @type {Map<string, {owner: ?{clientName: string, userName: string}, active: ?LeaseData, latest: ?LeaseData,
     *   lastUse: number, stale: boolean}>} */
    this.leaves = new Map();
    for (const [resource, node] of this.nodes) {
      if (node.leaves.length === 1 && node.leaves[0] === resource) {
        this.leaves.set(resource, { owner: null, active: null, latest: null, lastUse: 0, stale: false });
      }
    }
  }

  _indexTree(node, parent) {
    const leaves = [];
    for (const sub of node.sub ?? []) leaves.push(...this._indexTree(sub, node.resource));
    if (leaves.length === 0) leaves.push(node.resource);
    this.nodes.set(node.resource, { resource: node.resource, parent, leaves });
    return leaves;
  }

  /**
   * @param {string} ancestor
   * @param {string} resource
   * @returns {boolean} Whether the ancestor resource covers the resource.
   */
  covers(ancestor, resource) {
    for (let node = this.nodes.get(resource); node; node = this.nodes.get(node.parent)) {
      if (node.resource === ancestor) return true;
    }
    return false;
  }

  /**
   * @param {string} resource
   * @returns {?{clientName: string, userName: string}} The owner of the resource (of its first leaf).
   */
  ownerOf(resource) {
    const node = this.nodes.get(resource);
    if (!node) return null;
    return this.leaves.get(node.leaves[0]).owner;
  }

  /**
   * AcquireLease / TakeLease.
   * @param {string} resource
   * @param {string} clientName
   * @param {string} userName
   * @param {boolean} take Takes the lease from its owner.
   * @returns {{status: string, lease?: LeaseData, owner?: {clientName: string, userName: string}}} status is 'ok',
   *   'invalid_resource' or 'already_claimed'.
   */
  acquire(resource, clientName, userName, take) {
    const node = this.nodes.get(resource);
    if (!node) return { status: 'invalid_resource' };
    if (!take) {
      for (const leafName of node.leaves) {
        const leaf = this.leaves.get(leafName);
        if (leaf.owner && !leaf.stale) return { status: 'already_claimed', owner: leaf.owner };
      }
    }
    const previousOwners = new Set(node.leaves.map(name => this.leaves.get(name).owner?.clientName).filter(Boolean));
    this.counter += 1;
    const lease = { resource, epoch: this.epoch, sequence: [this.counter], clientNames: [clientName] };
    const owner = { clientName, userName };
    const now = this.robot.clock.now();
    for (const leafName of node.leaves) {
      Object.assign(this.leaves.get(leafName), { owner, active: lease, latest: lease, lastUse: now, stale: false });
    }
    const from = [...previousOwners].filter(name => name !== clientName);
    logger.info(
      `${take ? 'Took' : 'Acquired'} lease "${resource}" [${lease.sequence}] for "${clientName}"${
        from.length ? ` (from "${from.join('", "')}")` : ''
      }`,
    );
    this.robot.emit('lease:change', { resource, lease, owner, taken: take && from.length > 0 });
    return { status: 'ok', lease, owner };
  }

  /**
   * ReturnLease.
   * @param {?LeaseData} lease
   * @returns {string} 'ok', 'invalid_resource' or 'not_active'.
   */
  returnLease(lease) {
    const node = lease ? this.nodes.get(lease.resource) : null;
    if (!node) return 'invalid_resource';
    if (lease.epoch !== this.epoch || lease.sequence.length === 0) return 'not_active';
    for (const leafName of node.leaves) {
      const leaf = this.leaves.get(leafName);
      if (!leaf.owner || !leaf.active) return 'not_active';
      const cmp = compareLeases(lease, leaf.active);
      if (cmp !== CompareResult.SAME && cmp !== CompareResult.SUB_LEASE) return 'not_active';
    }
    const owner = this.leaves.get(node.leaves[0]).owner;
    for (const leafName of node.leaves) this.leaves.get(leafName).owner = null;
    logger.info(`Lease "${lease.resource}" [${lease.sequence[0]}] returned by "${owner?.clientName}"`);
    this.robot.emit('lease:change', { resource: lease.resource, lease: null, owner: null, returned: true });
    return 'ok';
  }

  /**
   * Validates a lease sent with a command (or a RetainLease), and records its use.
   * @param {?LeaseData} attempted
   * @param {string[]} [resources] The resources used by the command (default: the resource of the lease).
   * @param {{allowSuperLease?: boolean, record?: boolean}} [options]
   * @returns {{status: number, attempted: ?LeaseData, previous: ?LeaseData, latestKnown: ?LeaseData,
   *   latestResources: LeaseData[], owner: ?{clientName: string, userName: string}}}
   */
  use(attempted, resources = null, { allowSuperLease = false, record = true } = {}) {
    const result = {
      status: UseStatus.STATUS_OK,
      attempted,
      previous: null,
      latestKnown: null,
      latestResources: [],
      owner: null,
    };
    const fail = status => {
      result.status = status;
      return this._fillResult(result);
    };
    if (!attempted || !attempted.resource || attempted.sequence.length === 0) {
      return fail(UseStatus.STATUS_INVALID_LEASE);
    }
    const node = this.nodes.get(attempted.resource);
    if (!node) return fail(UseStatus.STATUS_UNMANAGED);
    if (attempted.epoch !== this.epoch) return fail(UseStatus.STATUS_WRONG_EPOCH);
    if (attempted.sequence[0] > this.counter || attempted.sequence[0] <= 0) return fail(UseStatus.STATUS_INVALID_LEASE);

    // The leaves of the resources of the command (the resources that this robot does not have are ignored, like the
    // arm of a robot without arm).
    const needed = new Set();
    for (const resource of resources ?? [attempted.resource]) {
      const target = this.nodes.get(resource);
      if (!target) continue;
      if (!this.covers(attempted.resource, resource)) return fail(UseStatus.STATUS_INVALID_LEASE);
      for (const leaf of target.leaves) needed.add(leaf);
    }

    for (const leafName of needed) {
      const leaf = this.leaves.get(leafName);
      if (!leaf.active) return fail(UseStatus.STATUS_INVALID_LEASE);
      const rootCmp = attempted.sequence[0] - leaf.active.sequence[0];
      // A newer lease was issued to someone (TakeLease, or AcquireLease of a stale lease).
      if (rootCmp < 0) return fail(UseStatus.STATUS_OLDER);
      // A root that the robot issued for another resource.
      if (rootCmp > 0) return fail(UseStatus.STATUS_INVALID_LEASE);
      if (!leaf.owner) return fail(UseStatus.STATUS_REVOKED);
      const cmp = compareLeases(attempted, leaf.latest);
      if (cmp === CompareResult.OLDER || (cmp === CompareResult.SUPER_LEASE && !allowSuperLease)) {
        return fail(UseStatus.STATUS_OLDER);
      }
    }

    if (needed.size > 0) result.previous = this.leaves.get([...needed][0]).latest;
    if (record) {
      const now = this.robot.clock.now();
      for (const leafName of needed) {
        const leaf = this.leaves.get(leafName);
        const cmp = compareLeases(attempted, leaf.latest);
        if (cmp === CompareResult.NEWER || cmp === CompareResult.SUB_LEASE) leaf.latest = attempted;
        leaf.lastUse = now;
        if (leaf.stale) {
          leaf.stale = false;
          logger.info(`Lease "${leafName}" is fresh again`);
        }
      }
    }
    return this._fillResult(result);
  }

  /**
   * Fills the owner and the latest leases of a lease use result.
   * @param {object} result
   * @returns {object}
   */
  _fillResult(result) {
    const node = result.attempted ? this.nodes.get(result.attempted.resource) : null;
    if (node) {
      const leaves = node.leaves.map(name => this.leaves.get(name));
      result.owner = leaves[0].owner;
      // The newest lease among the leaves, for the resource of the attempted lease.
      let latest = null;
      for (const leaf of leaves) {
        if (leaf.latest && (!latest || compareLeases(leaf.latest, latest) === CompareResult.NEWER)) {
          latest = leaf.latest;
        }
      }
      if (latest) result.latestKnown = { ...latest, resource: node.resource };
    }
    for (const [name, leaf] of this.leaves) {
      if (leaf.latest) result.latestResources.push({ ...leaf.latest, resource: name });
    }
    return result;
  }

  /**
   * @param {ReturnType<LeaseManager['use']>} result
   * @returns {leasePb.LeaseUseResult}
   */
  static resultToProto(result) {
    const proto = new leasePb.LeaseUseResult().setStatus(result.status);
    if (result.owner) {
      proto.setOwner(
        new leasePb.LeaseOwner().setClientName(result.owner.clientName).setUserName(result.owner.userName),
      );
    }
    if (result.attempted) proto.setAttemptedLease(leaseToProto(result.attempted));
    if (result.previous) proto.setPreviousLease(leaseToProto(result.previous));
    if (result.latestKnown) proto.setLatestKnownLease(leaseToProto(result.latestKnown));
    proto.setLatestResourcesList(result.latestResources.map(lease => leaseToProto(lease)));
    return proto;
  }

  /**
   * Marks leases stale (keepalive LeaseStale action), all of them when none is given.
   * @param {?LeaseData[]} leases
   */
  markStale(leases = null) {
    for (const [name, leaf] of this.leaves) {
      if (!leaf.owner || leaf.stale) continue;
      const matches =
        !leases ||
        leases.length === 0 ||
        leases.some(lease => this.covers(lease.resource, name) && lease.sequence[0] === leaf.active.sequence[0]);
      if (matches) {
        leaf.stale = true;
        logger.info(`Lease "${name}" of "${leaf.owner.clientName}" is stale`);
      }
    }
  }

  /**
   * @param {LeaseData} lease
   * @returns {boolean} Whether the owner of the resources of this lease is still the one it was issued to.
   */
  isStillOwner(lease) {
    const node = this.nodes.get(lease.resource);
    if (!node || lease.epoch !== this.epoch) return false;
    return node.leaves.every(name => {
      const leaf = this.leaves.get(name);
      return leaf.owner && leaf.active && leaf.active.sequence[0] === lease.sequence[0];
    });
  }

  /**
   * Staleness of the leases which are not used anymore.
   * @param {number} now
   */
  update(now) {
    const timeout = this.robot.config.lease.staleTimeoutSec;
    for (const [name, leaf] of this.leaves) {
      if (leaf.owner && !leaf.stale && now - leaf.lastUse > timeout) {
        leaf.stale = true;
        logger.info(`Lease "${name}" of "${leaf.owner.clientName}" is stale (not retained for ${timeout} s)`);
      }
    }
  }

  /**
   * ListLeases.
   * @returns {leasePb.ListLeasesResponse}
   */
  listLeasesResponse() {
    const response = new leasePb.ListLeasesResponse();
    const timeout = this.robot.config.lease.staleTimeoutSec;
    for (const [resource, node] of this.nodes) {
      const leaves = node.leaves.map(name => this.leaves.get(name));
      const entry = new leasePb.LeaseResource().setResource(resource);
      const first = leaves[0];
      const sameLease = leaves.every(leaf => leaf.active && first.active && leaf.active === first.active);
      if (first.owner && sameLease) {
        entry
          .setLease(leaseToProto(first.active, resource))
          .setLeaseOwner(
            new leasePb.LeaseOwner().setClientName(first.owner.clientName).setUserName(first.owner.userName),
          )
          .setIsStale(leaves.some(leaf => leaf.stale))
          .setStaleTime(secToTimestamp(Math.min(...leaves.map(leaf => leaf.lastUse)) + timeout));
      }
      response.addResources(entry);
    }
    response.setResourceTree(this._treeToProto(this.tree));
    return response;
  }

  _treeToProto(node) {
    return new leasePb.ResourceTree()
      .setResource(node.resource)
      .setSubResourcesList((node.sub ?? []).map(sub => this._treeToProto(sub)));
  }

  /**
   * A summary for the console.
   * @returns {string[]}
   */
  describe() {
    const lines = [];
    for (const [name, leaf] of this.leaves) {
      const owner = leaf.owner ? `"${leaf.owner.clientName}"${leaf.stale ? ' (stale)' : ''}` : 'nobody';
      const latest = leaf.latest ? `[${leaf.latest.sequence.join(', ')}]` : '-';
      lines.push(`${name}: ${owner}, latest lease ${latest}`);
    }
    return lines;
  }
}

module.exports = {
  CompareResult,
  LeaseManager,
  compareLeases,
  leaseFromProto,
  leaseToProto,
};
