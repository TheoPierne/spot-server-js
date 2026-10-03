'use strict';

const { randomBytes } = require('node:crypto');

const { timestampToNsec } = require('./clock');
const { LoggerUtil } = require('../loggerUtil');

const logger = LoggerUtil.getLogger('TIME_SYNC');

// Samples kept per clock, for the best estimate.
const MAX_SAMPLES = 25;

/**
 * The time sync service of the robot: each client clock gets an identifier, and the skew between the client clock and
 * the robot clock is estimated from the round trips that the client reports (like the TimeSyncUpdate RPC of a real
 * robot, which needs several consistent measurements before STATUS_OK).
 */
class TimeSyncManager {
  /**
   * @param {import('../robot')} robot
   */
  constructor(robot) {
    this.robot = robot;
    /** @type {Map<string, {clientName: string, samples: {rttNsec: bigint, skewNsec: bigint}[], synced: boolean,
     *   lastUpdate: number}>} */
    this.clocks = new Map();
  }

  /**
   * Processes a TimeSyncUpdate.
   * @param {string} clockIdentifier Empty for a new client clock.
   * @param {?import('../bosdyn/api/time_sync_pb').TimeSyncRoundTrip} roundTrip
   * @param {string} clientName
   * @returns {{clockIdentifier: string, previous: ?{rttNsec: bigint, skewNsec: bigint},
   *   best: ?{rttNsec: bigint, skewNsec: bigint}, synced: boolean}}
   */
  update(clockIdentifier, roundTrip, clientName) {
    let id = clockIdentifier;
    let clock = id ? this.clocks.get(id) : undefined;
    if (!clock) {
      // An unknown identifier (e.g. after a reboot of the robot) starts a new estimation, under the same identifier.
      if (!id) id = `${clientName || 'client'}-${randomBytes(4).toString('hex')}`;
      clock = { clientName, samples: [], synced: false, lastUpdate: 0 };
      this.clocks.set(id, clock);
    }
    clock.lastUpdate = this.robot.clock.now();

    let previous = null;
    const sample = roundTrip ? this._sample(roundTrip) : null;
    if (sample) {
      previous = sample;
      clock.samples.push(sample);
      if (clock.samples.length > MAX_SAMPLES) clock.samples.shift();
      if (!clock.synced && this._consistent(clock.samples)) {
        clock.synced = true;
        logger.info(`Time sync established with "${clientName}" (clock ${id})`);
      }
    }
    return { clockIdentifier: id, previous, best: this._best(clock.samples), synced: clock.synced };
  }

  /**
   * @param {string} clockIdentifier
   * @returns {boolean} Whether this client clock has an established time sync.
   */
  isSynced(clockIdentifier) {
    return Boolean(clockIdentifier && this.clocks.get(clockIdentifier)?.synced);
  }

  /**
   * The skew estimated for a client clock (robot time minus client time).
   * @param {string} clockIdentifier
   * @returns {?number} In seconds.
   */
  skewOf(clockIdentifier) {
    const best = this._best(this.clocks.get(clockIdentifier)?.samples ?? []);
    return best ? Number(best.skewNsec) / 1e9 : null;
  }

  /** Forgets the clocks (reboot of the robot). */
  reset() {
    this.clocks.clear();
  }

  /**
   * @param {import('../bosdyn/api/time_sync_pb').TimeSyncRoundTrip} roundTrip
   * @returns {?{rttNsec: bigint, skewNsec: bigint}}
   */
  _sample(roundTrip) {
    const clientTx = timestampToNsec(roundTrip.getClientTx());
    const serverRx = timestampToNsec(roundTrip.getServerRx());
    const serverTx = timestampToNsec(roundTrip.getServerTx());
    const clientRx = timestampToNsec(roundTrip.getClientRx());
    if (!clientTx || !serverRx || !serverTx || !clientRx) return null;
    const rttNsec = clientRx - clientTx - (serverTx - serverRx);
    if (rttNsec < 0n) return null;
    // Robot clock minus client clock: the mean of the two one-way differences.
    const skewNsec = (serverRx - clientTx + (serverTx - clientRx)) / 2n;
    return { rttNsec, skewNsec };
  }

  /**
   * The last measurements agree within their round trip times.
   * @param {{rttNsec: bigint, skewNsec: bigint}[]} samples
   * @returns {boolean}
   */
  _consistent(samples) {
    const needed = this.robot.config.timeSync.samplesNeeded;
    if (samples.length < needed) return false;
    const recent = samples.slice(-needed);
    const skews = recent.map(s => Number(s.skewNsec));
    const spread = Math.max(...skews) - Math.min(...skews);
    const maxRtt = Math.max(...recent.map(s => Number(s.rttNsec)));
    // Agreement within the round trip time, with a floor for the jitter of a loopback connection.
    return spread <= Math.max(maxRtt, 20e6);
  }

  /**
   * The best estimate: the measurement with the smallest round trip time.
   * @param {{rttNsec: bigint, skewNsec: bigint}[]} samples
   * @returns {?{rttNsec: bigint, skewNsec: bigint}}
   */
  _best(samples) {
    let best = null;
    for (const sample of samples) {
      if (!best || sample.rttNsec < best.rttNsec) best = sample;
    }
    return best;
  }
}

module.exports = { TimeSyncManager };
