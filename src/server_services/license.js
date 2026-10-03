'use strict';

const licensePb = require('../bosdyn/api/license_pb');
const { LicenseServiceService } = require('../bosdyn/api/license_service_grpc_pb');
const { unary } = require('../util');

/**
 * GetFeatureEnabled.
 * @param {licensePb.GetFeatureEnabledRequest} request
 * @param {{robot: import('../robot').Robot}} context
 * @returns {licensePb.GetFeatureEnabledResponse}
 */
function getFeatureEnabled(request, { robot }) {
  const response = new licensePb.GetFeatureEnabledResponse();
  const map = response.getFeatureEnabledMap();
  for (const code of request.getFeatureCodesList()) map.set(code, robot.license.isEnabled(code));
  return response;
}

module.exports = {
  service: LicenseServiceService,
  func: {
    getLicenseInfo: unary('GetLicenseInfo', licensePb.GetLicenseInfoResponse, (request, { robot }) =>
      new licensePb.GetLicenseInfoResponse().setLicense(robot.license.toProto()),
    ),
    getFeatureEnabled: unary('GetFeatureEnabled', licensePb.GetFeatureEnabledResponse, getFeatureEnabled),
  },
  directory: [{ name: 'license', type: 'bosdyn.api.LicenseService', authority: 'api.spot.robot' }],
};
