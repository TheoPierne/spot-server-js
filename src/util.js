'use strict';

const grpc = require('@grpc/grpc-js');
const jspb = require('google-protobuf');
const { Any } = require('google-protobuf/google/protobuf/any_pb');

const { CommonError, ResponseHeader } = require('./bosdyn/api/header_pb');
const { getContext } = require('./context');
const { GrpcError, HeaderError, invalidRequest } = require('./errors');
const { LoggerUtil } = require('./loggerUtil');
const { nsecToTimestamp } = require('./sim/clock');

const logger = LoggerUtil.getLogger('RPC');

// The echoed request is left out of the response header beyond this size, like a real robot strips its large fields.
const MAX_ECHOED_REQUEST_BYTES = 64 * 1024;

let _typeIndex = null;

/**
 * Maps the constructors of the generated messages to their full name (e.g. 'bosdyn.api.RobotStateRequest'), from the
 * global namespace where the generated code exports them (goog.exportSymbol).
 * @returns {Map<Function, string>}
 */
function buildTypeIndex() {
  const index = new Map();
  const seen = new Set();
  const walk = (node, path) => {
    if (!node || seen.has(node)) return;
    seen.add(node);
    if (typeof node === 'function' && node.prototype instanceof jspb.Message) index.set(node, path);
    for (const key of Object.keys(node)) {
      const child = node[key];
      if (child && (typeof child === 'object' || typeof child === 'function') && /^[A-Za-z_]/.test(key)) {
        walk(child, `${path}.${key}`);
      }
    }
  };
  const root = globalThis.proto ?? {};
  for (const key of Object.keys(root)) walk(root[key], key);
  return index;
}

/**
 * @param {jspb.Message} message
 * @returns {?string} The full name of the type of the message.
 */
function protoTypeName(message) {
  if (!_typeIndex || !_typeIndex.has(message.constructor)) _typeIndex = buildTypeIndex();
  return _typeIndex.get(message.constructor) ?? null;
}

/**
 * Packs a message in an Any.
 * @param {jspb.Message} message
 * @returns {Any}
 */
function packAny(message) {
  const any = new Any();
  any.pack(message.serializeBinary(), protoTypeName(message));
  return any;
}

/**
 * Unpacks an Any, if it holds the given type.
 * @template T
 * @param {?Any} any
 * @param {{deserializeBinary: function(Uint8Array): T}} type
 * @param {string} typeName
 * @returns {?T}
 */
function unpackAny(any, type, typeName) {
  if (!any || !any.getTypeUrl()) return null;
  return any.unpack(type.deserializeBinary, typeName);
}

/**
 * Fills the common header of a response: the echoed request header, the times of the robot (the time sync of the
 * clients is computed from them), the error, and the echoed request.
 * @param {jspb.Message} response
 * @param {jspb.Message} request
 * @param {bigint} receivedNsec Robot time when the request was received.
 * @param {number} [code=CODE_OK]
 * @param {string} [message='']
 */
function fillResponseHeader(response, request, receivedNsec, code = CommonError.Code.CODE_OK, message = '') {
  if (typeof response?.setHeader !== 'function') return;
  const { robot } = getContext();
  const header = new ResponseHeader()
    .setRequestReceivedTimestamp(nsecToTimestamp(receivedNsec))
    .setError(new CommonError().setCode(code).setMessage(message));
  const requestHeader = typeof request?.getHeader === 'function' ? request.getHeader() : null;
  if (requestHeader) header.setRequestHeader(requestHeader);
  if (request instanceof jspb.Message) {
    const bytes = request.serializeBinary();
    const typeName = protoTypeName(request);
    if (bytes.length <= MAX_ECHOED_REQUEST_BYTES && typeName) {
      const any = new Any();
      any.pack(bytes, typeName);
      header.setRequest(any);
    }
  }
  header.setResponseTimestamp(nsecToTimestamp(robot.clock.nowNsec()));
  response.setHeader(header);
}

/**
 * The client name of the request header.
 * @param {jspb.Message} request
 * @returns {string}
 */
function clientNameOf(request) {
  const header = typeof request?.getHeader === 'function' ? request.getHeader() : null;
  return header?.getClientName() ?? '';
}

/**
 * Converts the errors thrown by a handler to a gRPC error.
 * @param {Error} err
 * @param {string} method
 * @returns {{code: number, details: string}}
 */
function toServiceError(err, method) {
  if (err instanceof GrpcError) return { code: err.code, details: err.details };
  logger.error(`${method} failed: ${err.stack ?? err}`);
  return { code: grpc.status.INTERNAL, details: `Internal error of the simulator: ${err.message}` };
}

/**
 * A robot which is off or rebooting does not answer: the call waits (the client gets its deadline, like with an
 * unreachable robot), and fails when the robot boots (the connections of before the reboot are reset).
 * @param {any} call
 * @param {function({code: number, details: string}): void} fail
 * @returns {boolean} Whether the call is held.
 */
function holdIfOffline(call, fail) {
  const { robot } = getContext();
  if (robot.online) return false;
  robot.holdCall(call, fail);
  return true;
}

/**
 * Checks the user token of the call.
 * @param {any} call
 * @param {boolean} tokenRequired
 * @returns {import('./robot')} The robot.
 */
function checkCall(call, tokenRequired) {
  const { robot } = getContext();
  if (tokenRequired) robot.auth.checkCallMetadata(call.metadata);
  robot.sync();
  return robot;
}

/**
 * Wraps the handler of a unary RPC: it receives the request and a context, and returns the response (or a promise).
 * It throws a GrpcError for a gRPC status, or a HeaderError for an error in the common header.
 *
 * @param {string} method Name of the method, for the logs.
 * @param {function(new: jspb.Message)} ResponseType
 * @param {function(any, {robot: import('./robot'), call: any, clientName: string}): any} handler
 * @param {{tokenRequired?: boolean}} [options]
 * @returns {grpc.handleUnaryCall<any, any>}
 */
function unary(method, ResponseType, handler, { tokenRequired = true } = {}) {
  return (call, done) => {
    if (holdIfOffline(call, err => done(err))) return;
    let receivedNsec;
    try {
      const robot = checkCall(call, tokenRequired);
      receivedNsec = robot.clock.nowNsec();
      const clientName = clientNameOf(call.request);
      logger.debug(`${method} from "${clientName}"`);
      Promise.resolve()
        .then(() => handler(call.request, { robot, call, clientName }))
        .then(
          response => {
            fillResponseHeader(response, call.request, receivedNsec);
            done(null, response);
          },
          err => {
            if (err instanceof HeaderError) {
              const response = new ResponseType();
              fillResponseHeader(response, call.request, receivedNsec, err.code, err.message);
              done(null, response);
            } else {
              done(toServiceError(err, method));
            }
          },
        );
    } catch (err) {
      done(toServiceError(err, method));
    }
  };
}

/**
 * Wraps the handler of a server streaming RPC: it receives the request and a context, writes the responses with
 * context.write(response) (their header is filled), and the call ends when it returns (or its promise resolves).
 *
 * @param {string} method
 * @param {function(any, {robot: import('./robot'), call: any, write: function(jspb.Message): boolean}):
 *   (void|Promise<void>)} handler
 * @param {{tokenRequired?: boolean}} [options]
 * @returns {grpc.handleServerStreamingCall<any, any>}
 */
function serverStreaming(method, handler, { tokenRequired = true } = {}) {
  return call => {
    const fail = err => call.destroy(Object.assign(new Error(err.message), toServiceError(err, method)));
    if (holdIfOffline(call, err => fail(new GrpcError(err.code, err.details)))) return;
    try {
      const robot = checkCall(call, tokenRequired);
      const receivedNsec = robot.clock.nowNsec();
      logger.debug(`${method} (stream) from "${clientNameOf(call.request)}"`);
      const write = response => {
        fillResponseHeader(response, call.request, receivedNsec);
        return call.write(response);
      };
      Promise.resolve()
        .then(() => handler(call.request, { robot, call, write }))
        .then(() => call.end(), fail);
    } catch (err) {
      fail(err);
    }
  };
}

/**
 * Wraps the handler of a client streaming RPC: it receives the list of the requests.
 * @param {string} method
 * @param {function(new: jspb.Message)} ResponseType
 * @param {function(any[], {robot: import('./robot'), call: any}): any} handler
 * @param {{tokenRequired?: boolean}} [options]
 * @returns {grpc.handleClientStreamingCall<any, any>}
 */
function clientStreaming(method, ResponseType, handler, { tokenRequired = true } = {}) {
  return (call, done) => {
    const requests = [];
    call.on('data', request => requests.push(request));
    call.on('error', () => {
      // The client cancelled the call: nothing to answer.
    });
    call.on('end', () => {
      if (holdIfOffline(call, err => done(err))) return;
      let receivedNsec;
      try {
        const robot = checkCall(call, tokenRequired);
        receivedNsec = robot.clock.nowNsec();
        logger.debug(`${method} (${requests.length} messages) from "${clientNameOf(requests[0])}"`);
        Promise.resolve()
          .then(() => handler(requests, { robot, call }))
          .then(
            response => {
              fillResponseHeader(response, requests[0], receivedNsec);
              done(null, response);
            },
            err => {
              if (err instanceof HeaderError) {
                const response = new ResponseType();
                fillResponseHeader(response, requests[0], receivedNsec, err.code, err.message);
                done(null, response);
              } else {
                done(toServiceError(err, method));
              }
            },
          );
      } catch (err) {
        done(toServiceError(err, method));
      }
    });
  };
}

module.exports = {
  GrpcError,
  HeaderError,
  clientNameOf,
  clientStreaming,
  fillResponseHeader,
  invalidRequest,
  packAny,
  protoTypeName,
  serverStreaming,
  unary,
  unpackAny,
};
