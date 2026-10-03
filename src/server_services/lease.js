'use strict';

const leasePb = require('../bosdyn/api/lease_pb');
const { LeaseServiceService } = require('../bosdyn/api/lease_service_grpc_pb');
const { LeaseManager, leaseFromProto, leaseToProto } = require('../sim/lease');
const { unary } = require('../util');

/**
 * The user of a call, from its token.
 * @param {import('../robot').Robot} robot
 * @param {any} call
 * @returns {string}
 */
function userOf(robot, call) {
  const value = call.metadata?.get('authorization')?.[0];
  const token = /^Bearer (.+)$/.exec(String(value ?? ''))?.[1];
  return (token && robot.auth.verify(token)?.sub) || '';
}

/**
 * @param {?{clientName: string, userName: string}} owner
 * @returns {leasePb.LeaseOwner}
 */
function ownerToProto(owner) {
  return new leasePb.LeaseOwner().setClientName(owner?.clientName ?? '').setUserName(owner?.userName ?? '');
}

/**
 * AcquireLease / TakeLease.
 * @param {boolean} take
 * @returns {function(any, object): any}
 */
function acquireOrTake(take) {
  const Response = take ? leasePb.TakeLeaseResponse : leasePb.AcquireLeaseResponse;
  return (request, { robot, call, clientName }) => {
    const result = robot.leases.acquire(request.getResource(), clientName, userOf(robot, call), take);
    const response = new Response();
    if (result.status === 'invalid_resource') return response.setStatus(Response.Status.STATUS_INVALID_RESOURCE);
    if (result.status === 'already_claimed') {
      return response
        .setStatus(Response.Status.STATUS_RESOURCE_ALREADY_CLAIMED)
        .setLeaseOwner(ownerToProto(result.owner));
    }
    return response
      .setStatus(Response.Status.STATUS_OK)
      .setLease(leaseToProto(result.lease))
      .setLeaseOwner(ownerToProto(result.owner));
  };
}

/**
 * ReturnLease.
 * @param {leasePb.ReturnLeaseRequest} request
 * @param {{robot: import('../robot').Robot}} context
 * @returns {leasePb.ReturnLeaseResponse}
 */
function returnLease(request, { robot }) {
  const { Status } = leasePb.ReturnLeaseResponse;
  const status = robot.leases.returnLease(leaseFromProto(request.getLease()));
  const value =
    { ok: Status.STATUS_OK, invalid_resource: Status.STATUS_INVALID_RESOURCE }[status] ??
    Status.STATUS_NOT_ACTIVE_LEASE;
  return new leasePb.ReturnLeaseResponse().setStatus(value);
}

/**
 * RetainLease: keeps the lease fresh (a super lease of the lease to retain is accepted).
 * @param {leasePb.RetainLeaseRequest} request
 * @param {{robot: import('../robot').Robot}} context
 * @returns {leasePb.RetainLeaseResponse}
 */
function retainLease(request, { robot }) {
  const result = robot.leases.use(leaseFromProto(request.getLease()), null, { allowSuperLease: true });
  return new leasePb.RetainLeaseResponse().setLeaseUseResult(LeaseManager.resultToProto(result));
}

module.exports = {
  service: LeaseServiceService,
  func: {
    acquireLease: unary('AcquireLease', leasePb.AcquireLeaseResponse, acquireOrTake(false)),
    takeLease: unary('TakeLease', leasePb.TakeLeaseResponse, acquireOrTake(true)),
    returnLease: unary('ReturnLease', leasePb.ReturnLeaseResponse, returnLease),
    retainLease: unary('RetainLease', leasePb.RetainLeaseResponse, retainLease),
    listLeases: unary('ListLeases', leasePb.ListLeasesResponse, (request, { robot }) =>
      robot.leases.listLeasesResponse(),
    ),
  },
  directory: [{ name: 'lease', type: 'bosdyn.api.LeaseService', authority: 'api.spot.robot' }],
  userOf,
};
