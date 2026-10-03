'use strict';

const { GetAuthTokenResponse } = require('../bosdyn/api/auth_pb');
const { AuthServiceService } = require('../bosdyn/api/auth_service_grpc_pb');
const { unary } = require('../util');

/**
 * GetAuthToken: with a username and a password, or with a valid token (refresh).
 * @param {import('../bosdyn/api/auth_pb').GetAuthTokenRequest} request
 * @param {{robot: import('../robot').Robot}} context
 * @returns {GetAuthTokenResponse}
 */
function getAuthToken(request, { robot }) {
  const credentials = request.getToken()
    ? { token: request.getToken() }
    : { username: request.getUsername(), password: request.getPassword() };
  const result = robot.auth.authenticate(credentials);
  const response = new GetAuthTokenResponse().setStatus(result.status);
  if (result.token) response.setToken(result.token);
  return response;
}

module.exports = {
  service: AuthServiceService,
  func: {
    getAuthToken: unary('GetAuthToken', GetAuthTokenResponse, getAuthToken, { tokenRequired: false }),
  },
  directory: [{ name: 'auth', type: 'bosdyn.api.AuthService', authority: 'auth.spot.robot', userTokenRequired: false }],
};
