'use strict';

const { Buffer } = require('node:buffer');
const { createHmac, randomBytes, randomUUID, timingSafeEqual } = require('node:crypto');

const grpc = require('@grpc/grpc-js');

const { GetAuthTokenResponse } = require('../bosdyn/api/auth_pb');
const { GrpcError } = require('../errors');
const { LoggerUtil } = require('../loggerUtil');

const logger = LoggerUtil.getLogger('AUTH');

/**
 * @param {Buffer|string} data
 * @returns {string}
 */
function base64url(data) {
  return Buffer.from(data).toString('base64url');
}

/**
 * The authentication of the robot: accounts, user tokens (JWTs signed by the robot, valid 12 hours) and the lockout
 * after repeated failed attempts.
 */
class AuthManager {
  /**
   * @param {import('../robot')} robot
   */
  constructor(robot) {
    this.robot = robot;
    this.config = robot.config.auth;
    // The signing key of the tokens. It is persisted, so that the tokens stay valid after a restart of the simulator
    // (the tokens of a real robot stay valid after a reboot).
    this.secret = randomBytes(32).toString('hex');
    this.failedAttempts = 0;
    this.lockedUntil = 0;
  }

  get permissive() {
    return !this.config.users || this.config.users.length === 0;
  }

  /**
   * @param {{username: string, password: string}|{token: string}} credentials
   * @returns {{status: number, token?: string}}
   */
  authenticate(credentials) {
    const now = this.robot.clock.now();
    if ('token' in credentials) {
      const claims = this.verify(credentials.token);
      if (!claims) {
        logger.warn('Authentication with an invalid or expired token');
        return { status: GetAuthTokenResponse.Status.STATUS_INVALID_TOKEN };
      }
      return { status: GetAuthTokenResponse.Status.STATUS_OK, token: this.issue(claims.sub) };
    }

    if (now < this.lockedUntil) {
      return { status: GetAuthTokenResponse.Status.STATUS_TEMPORARILY_LOCKED_OUT };
    }
    const { username, password } = credentials;
    const valid =
      this.permissive || this.config.users.some(user => user.username === username && user.password === password);
    if (!valid) {
      this.failedAttempts += 1;
      logger.warn(`Invalid login for "${username}" (${this.failedAttempts} consecutive failures)`);
      if (this.failedAttempts >= this.config.maxFailedAttempts) {
        this.failedAttempts = 0;
        this.lockedUntil = now + this.config.lockoutSec;
        logger.warn(`Authentication locked out for ${this.config.lockoutSec} s`);
      }
      return { status: GetAuthTokenResponse.Status.STATUS_INVALID_LOGIN };
    }
    this.failedAttempts = 0;
    logger.info(`User "${username}" authenticated`);
    return { status: GetAuthTokenResponse.Status.STATUS_OK, token: this.issue(username) };
  }

  /**
   * Issues a user token (a JWT like the ones of a real robot).
   * @param {string} username
   * @returns {string}
   */
  issue(username) {
    const now = Math.floor(this.robot.clock.now());
    const header = base64url(JSON.stringify({ alg: 'HS256', typ: 'JWT' }));
    const payload = base64url(
      JSON.stringify({
        iss: this.robot.config.robot.serialNumber,
        sub: username,
        iat: now,
        exp: now + this.config.tokenLifetimeSec,
        jti: randomUUID(),
        per: 'user',
      }),
    );
    const signature = createHmac('sha256', this.secret).update(`${header}.${payload}`).digest('base64url');
    return `${header}.${payload}.${signature}`;
  }

  /**
   * @param {string} token
   * @returns {?{sub: string, exp: number}} The claims of a valid token, null otherwise.
   */
  verify(token) {
    const parts = typeof token === 'string' ? token.split('.') : [];
    if (parts.length !== 3) return null;
    const expected = createHmac('sha256', this.secret).update(`${parts[0]}.${parts[1]}`).digest();
    const signature = Buffer.from(parts[2], 'base64url');
    if (signature.length !== expected.length || !timingSafeEqual(signature, expected)) return null;
    let claims;
    try {
      claims = JSON.parse(Buffer.from(parts[1], 'base64url').toString('utf8'));
    } catch {
      return null;
    }
    if (claims.iss !== this.robot.config.robot.serialNumber) return null;
    if (typeof claims.exp !== 'number' || claims.exp <= this.robot.clock.now()) return null;
    return claims;
  }

  /**
   * Checks the user token of a call (metadata "authorization: Bearer <token>"), like the services of a real robot
   * which require a user token.
   * @param {grpc.Metadata} metadata
   * @throws {GrpcError} UNAUTHENTICATED.
   */
  checkCallMetadata(metadata) {
    const value = metadata?.get('authorization')?.[0];
    if (!value) throw new GrpcError(grpc.status.UNAUTHENTICATED, 'Request is missing a user token.');
    const match = /^Bearer (.+)$/.exec(String(value));
    if (!match || !this.verify(match[1])) {
      throw new GrpcError(grpc.status.UNAUTHENTICATED, 'Invalid or expired user token.');
    }
  }

  toJSON() {
    return { secret: this.secret };
  }

  loadFromJSON(json) {
    if (typeof json?.secret === 'string' && json.secret.length >= 32) this.secret = json.secret;
  }
}

module.exports = { AuthManager };
