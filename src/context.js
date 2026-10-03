'use strict';

const path = require('node:path');
const { clearInterval, setInterval } = require('node:timers');

const { LoggerUtil } = require('./loggerUtil');
const { StateStore } = require('./stateStore');

const logger = LoggerUtil.getLogger('CTX');

let _ctx = null;

/**
 * Creates the simulated robot shared by the services.
 * @param {object} opts
 * @param {object} opts.config The configuration (config.js).
 * @param {?string} [opts.stateFile] JSON file of the persistent state (default: data/robot_state.json).
 * @param {boolean} [opts.resetState=false] Ignores the persisted state.
 * @param {boolean} [opts.persist=true] Saves the state.
 * @param {import('./sim/clock').RobotClock} [opts.clock]
 * @returns {Promise<{robot: import('./robot').Robot, store: ?StateStore, stateFile: ?string}>}
 */
async function initContext(opts) {
  if (_ctx) return _ctx;
  // Required here: the modules of the robot require the context (through the utilities of the services).
  const { Robot } = require('./robot');
  const robot = new Robot(opts.config, { clock: opts.clock });
  let store = null;
  let stateFile = null;
  if (opts.persist !== false) {
    stateFile = opts.stateFile ? path.resolve(opts.stateFile) : path.resolve(__dirname, '../data/robot_state.json');
    store = new StateStore(stateFile, { debounceMs: 250 });
    if (opts.resetState) {
      logger.info('State reset requested: the robot starts fresh.');
    } else {
      const loaded = await store.load();
      if (loaded && robot.loadFromJSON(loaded)) {
        logger.info(`Robot state loaded from ${stateFile}`);
      } else if (loaded) {
        logger.info(`The state of ${stateFile} is from another version or robot: the robot starts fresh.`);
      }
    }
    // Saves the physical state when it changes, and the battery regularly.
    const persist = () => store.saveSoon(robot.toJSON());
    for (const event of ['power:change', 'posture', 'boot', 'persist']) robot.on(event, persist);
    const timer = setInterval(persist, 10_000);
    timer.unref();
    robot.once('dispose', () => clearInterval(timer));
    // Saved before serving: the key of the tokens must survive even an abrupt stop.
    persist();
    await store.flush();
  }
  _ctx = { robot, store, stateFile };
  return _ctx;
}

/**
 * @returns {{robot: import('./robot').Robot, store: ?StateStore, stateFile: ?string}}
 */
function getContext() {
  if (!_ctx) throw new Error('Context not initialized. Call initContext() before loading services.');
  return _ctx;
}

/**
 * Forgets the context (tests): stops the robot and saves its state.
 * @returns {Promise<void>}
 */
async function disposeContext() {
  if (!_ctx) return;
  const { robot, store } = _ctx;
  robot.stop();
  robot.emit('dispose');
  if (store) {
    store.saveSoon(robot.toJSON());
    await store.flush();
  }
  _ctx = null;
}

module.exports = {
  disposeContext,
  getContext,
  initContext,
};
