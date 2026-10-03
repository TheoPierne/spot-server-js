'use strict';

const { CommonError } = require('./bosdyn/api/header_pb');

/**
 * An error returned as a gRPC status (e.g. UNAUTHENTICATED), instead of a response.
 */
class GrpcError extends Error {
  /**
   * @param {number} code A grpc.status code.
   * @param {string} message
   */
  constructor(code, message) {
    super(message);
    this.name = 'GrpcError';
    this.code = code;
    this.details = message;
  }
}

/**
 * An error returned in the common header of the response (CommonError), with an otherwise empty response.
 */
class HeaderError extends Error {
  /**
   * @param {number} code A CommonError.Code.
   * @param {string} message
   */
  constructor(code, message) {
    super(message);
    this.name = 'HeaderError';
    this.code = code;
  }
}

/**
 * @param {string} message
 * @returns {HeaderError}
 */
function invalidRequest(message) {
  return new HeaderError(CommonError.Code.CODE_INVALID_REQUEST, message);
}

module.exports = {
  GrpcError,
  HeaderError,
  invalidRequest,
};
