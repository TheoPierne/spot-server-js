'use strict';

const imagePb = require('../bosdyn/api/image_pb');
const { ImageServiceService } = require('../bosdyn/api/image_service_grpc_pb');
const { unary } = require('../util');

/**
 * ListImageSources.
 * @param {imagePb.ListImageSourcesRequest} request
 * @param {{robot: import('../robot').Robot}} context
 * @returns {imagePb.ListImageSourcesResponse}
 */
function listImageSources(request, { robot }) {
  const { cameras } = robot;
  return new imagePb.ListImageSourcesResponse().setImageSourcesList(
    cameras.sources.map(src => cameras.sourceToProto(src)),
  );
}

/**
 * GetImage: renders the cameras from the current pose of the robot.
 * @param {imagePb.GetImageRequest} request
 * @param {{robot: import('../robot').Robot}} context
 * @returns {Promise<imagePb.GetImageResponse>}
 */
async function getImage(request, { robot }) {
  const responses = await Promise.all(
    request.getImageRequestsList().map(imageRequest => robot.cameras.capture(imageRequest)),
  );
  return new imagePb.GetImageResponse().setImageResponsesList(responses);
}

module.exports = {
  service: ImageServiceService,
  func: {
    listImageSources: unary('ListImageSources', imagePb.ListImageSourcesResponse, listImageSources),
    getImage: unary('GetImage', imagePb.GetImageResponse, getImage),
  },
  directory: [{ name: 'image', type: 'bosdyn.api.ImageService', authority: 'api.spot.robot' }],
};
