'use strict';

const worldObjectPb = require('../bosdyn/api/world_object_pb');
const { WorldObjectServiceService } = require('../bosdyn/api/world_object_service_grpc_pb');
const { timestampToSec } = require('../sim/clock');
const { invalidRequest, unary } = require('../util');

/**
 * ListWorldObjects: the fiducials and docks seen in the last 15 seconds, and the objects added by clients.
 * @param {worldObjectPb.ListWorldObjectRequest} request
 * @param {{robot: import('../robot').Robot}} context
 * @returns {worldObjectPb.ListWorldObjectResponse}
 */
function listWorldObjects(request, { robot }) {
  const after = request.hasTimestampFilter() ? timestampToSec(request.getTimestampFilter()) : null;
  return new worldObjectPb.ListWorldObjectResponse().setWorldObjectsList(
    robot.world.list(request.getObjectTypeList(), after),
  );
}

/**
 * MutateWorldObjects: add, change or delete the objects added by clients.
 * @param {worldObjectPb.MutateWorldObjectRequest} request
 * @param {{robot: import('../robot').Robot}} context
 * @returns {worldObjectPb.MutateWorldObjectResponse}
 */
function mutateWorldObjects(request, { robot }) {
  const result = robot.world.mutate(request.getMutation());
  if (result.invalid) throw invalidRequest('The mutation has no action.');
  return new worldObjectPb.MutateWorldObjectResponse().setStatus(result.status).setMutatedObjectId(result.id);
}

module.exports = {
  service: WorldObjectServiceService,
  func: {
    listWorldObjects: unary('ListWorldObjects', worldObjectPb.ListWorldObjectResponse, listWorldObjects),
    mutateWorldObjects: unary('MutateWorldObjects', worldObjectPb.MutateWorldObjectResponse, mutateWorldObjects),
  },
  directory: [{ name: 'world-objects', type: 'bosdyn.api.WorldObjectService', authority: 'world-objects.spot.robot' }],
};
