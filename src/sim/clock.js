'use strict';

const process = require('node:process');

const { Duration } = require('google-protobuf/google/protobuf/duration_pb');
const { Timestamp } = require('google-protobuf/google/protobuf/timestamp_pb');

const NSEC_PER_SEC = 1_000_000_000n;

/**
 * The clock of the simulated robot ("robot time").
 *
 * A real robot has its own clock, which is not the clock of the clients: the time sync service estimates the skew
 * between them. The robot time is the time of the host plus a configurable skew, so that the clients which do not
 * convert their times (e.g. the end times of the commands) fail like on a real robot.
 *
 * In manual mode (the tests), the time only moves with advance().
 */
class RobotClock {
  /**
   * @param {object} [options]
   * @param {number} [options.skewSec=0] Robot time minus host time, in seconds.
   * @param {boolean} [options.manual=false] The time only moves with advance().
   */
  constructor({ skewSec = 0, manual = false } = {}) {
    this.skewNsec = BigInt(Math.round(skewSec * 1e9));
    this.manual = manual;
    this._anchorEpochNsec = BigInt(Date.now()) * 1_000_000n;
    this._anchorHrNsec = process.hrtime.bigint();
    this._manualNsec = this._anchorEpochNsec;
  }

  /**
   * The time of the host (the clock of the clients when they run on the same machine), in nanoseconds.
   * @returns {bigint}
   */
  hostNsec() {
    if (this.manual) return this._manualNsec;
    return this._anchorEpochNsec + (process.hrtime.bigint() - this._anchorHrNsec);
  }

  /**
   * The robot time, in nanoseconds since the epoch.
   * @returns {bigint}
   */
  nowNsec() {
    return this.hostNsec() + this.skewNsec;
  }

  /**
   * The robot time, in seconds since the epoch.
   * @returns {number}
   */
  now() {
    return nsecToSec(this.nowNsec());
  }

  /**
   * Moves the time of a manual clock.
   * @param {number} seconds
   */
  advance(seconds) {
    if (!this.manual) throw new Error('Only a manual clock can be advanced.');
    this._manualNsec += BigInt(Math.round(seconds * 1e9));
  }
}

/**
 * @param {bigint} nsec
 * @returns {number}
 */
function nsecToSec(nsec) {
  const seconds = nsec / NSEC_PER_SEC;
  return Number(seconds) + Number(nsec - seconds * NSEC_PER_SEC) / 1e9;
}

/**
 * @param {number} seconds Seconds since the epoch.
 * @returns {Timestamp}
 */
function secToTimestamp(seconds) {
  let whole = Math.floor(seconds);
  let nanos = Math.round((seconds - whole) * 1e9);
  if (nanos >= 1e9) {
    whole += 1;
    nanos -= 1e9;
  }
  return new Timestamp().setSeconds(whole).setNanos(nanos);
}

/**
 * @param {bigint} nsec Nanoseconds since the epoch.
 * @returns {Timestamp}
 */
function nsecToTimestamp(nsec) {
  const seconds = nsec / NSEC_PER_SEC;
  return new Timestamp().setSeconds(Number(seconds)).setNanos(Number(nsec - seconds * NSEC_PER_SEC));
}

/**
 * @param {?Timestamp} timestamp
 * @returns {?number} Seconds since the epoch, null for an unset timestamp.
 */
function timestampToSec(timestamp) {
  if (!timestamp) return null;
  return timestamp.getSeconds() + timestamp.getNanos() / 1e9;
}

/**
 * @param {?Timestamp} timestamp
 * @returns {?bigint}
 */
function timestampToNsec(timestamp) {
  if (!timestamp) return null;
  return BigInt(timestamp.getSeconds()) * NSEC_PER_SEC + BigInt(timestamp.getNanos());
}

/**
 * @param {number} seconds
 * @returns {Duration}
 */
function secToDuration(seconds) {
  const sign = seconds < 0 ? -1 : 1;
  const abs = Math.abs(seconds);
  let whole = Math.floor(abs);
  let nanos = Math.round((abs - whole) * 1e9);
  if (nanos >= 1e9) {
    whole += 1;
    nanos -= 1e9;
  }
  // The seconds and the nanos of a Duration have the same sign.
  return new Duration().setSeconds(sign * whole).setNanos(sign * nanos);
}

/**
 * @param {bigint} nsec
 * @returns {Duration}
 */
function nsecToDuration(nsec) {
  const seconds = nsec / NSEC_PER_SEC;
  return new Duration().setSeconds(Number(seconds)).setNanos(Number(nsec - seconds * NSEC_PER_SEC));
}

/**
 * @param {?Duration} duration
 * @returns {?number}
 */
function durationToSec(duration) {
  if (!duration) return null;
  return duration.getSeconds() + duration.getNanos() / 1e9;
}

module.exports = {
  NSEC_PER_SEC,
  RobotClock,
  durationToSec,
  nsecToDuration,
  nsecToSec,
  nsecToTimestamp,
  secToDuration,
  secToTimestamp,
  timestampToNsec,
  timestampToSec,
};
