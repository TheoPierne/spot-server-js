#!/usr/bin/env node
'use strict';

const { readdirSync, readFileSync } = require('node:fs');
const path = require('node:path');
const process = require('node:process');

const grpc = require('@grpc/grpc-js');
const { ArgumentParser } = require('argparse');

const { loadConfig } = require('./config');
const { disposeContext, initContext } = require('./context');
const { LoggerUtil } = require('./loggerUtil');

const logger = LoggerUtil.getLogger('SERVER');

const SERVICES_DIR = path.join(__dirname, 'server_services');
const RESOURCES_DIR = path.join(__dirname, 'resources');

/**
 * Starts the simulated robot and its gRPC server.
 * @param {object} [options]
 * @param {string} [options.host='0.0.0.0']
 * @param {number} [options.port=443] 0 for a free port.
 * @param {boolean} [options.insecure=false] Without TLS.
 * @param {object} [options.config] Configuration (merged over the defaults of config.js).
 * @param {?string} [options.configFile] JSON configuration file.
 * @param {?string} [options.stateFile]
 * @param {boolean} [options.resetState=false]
 * @param {boolean} [options.persist=true]
 * @param {import('./sim/clock').RobotClock} [options.clock]
 * @returns {Promise<{server: grpc.Server, port: number, robot: import('./robot').Robot,
 *   stop: function(): Promise<void>}>}
 */
async function startServer(options = {}) {
  const config = loadConfig(options.configFile ?? null, options.config ?? {});
  const { robot } = await initContext({
    config,
    stateFile: options.stateFile ?? null,
    resetState: options.resetState ?? false,
    persist: options.persist ?? true,
    clock: options.clock,
  });

  const server = new grpc.Server();
  for (const file of readdirSync(SERVICES_DIR)
    .filter(name => name.endsWith('.js'))
    .sort()) {
    const module = require(path.join(SERVICES_DIR, file));
    server.addService(module.service, module.func);
    for (const extra of module.extraServices ?? []) server.addService(extra.service, extra.func);
    for (const entry of module.directory ?? []) robot.directory.addBuiltin(entry);
  }

  const credentials = options.insecure
    ? grpc.ServerCredentials.createInsecure()
    : grpc.ServerCredentials.createSsl(
        readFileSync(path.join(RESOURCES_DIR, 'ca.crt')),
        [
          {
            private_key: readFileSync(path.join(RESOURCES_DIR, 'server.key')),
            cert_chain: readFileSync(path.join(RESOURCES_DIR, 'server.crt')),
          },
        ],
        false,
      );
  const host = options.host ?? '0.0.0.0';
  const port = await new Promise((resolve, reject) => {
    server.bindAsync(`${host}:${options.port ?? 443}`, credentials, (err, boundPort) =>
      err ? reject(err) : resolve(boundPort),
    );
  });
  robot.start();
  logger.info(
    `Robot "${config.robot.nickname}" (${config.robot.serialNumber}) listening on ${host}:${port}` +
      ` (${options.insecure ? 'insecure' : 'TLS'}), ${robot.directory.builtin.size} services`,
  );
  if (robot.auth.permissive) logger.info('No account configured: any username and password are accepted.');

  const stop = async () => {
    await new Promise(resolve => server.tryShutdown(() => resolve()));
    await disposeContext();
  };
  return { server, port, robot, stop };
}

/**
 * The command line.
 * @param {string[]} [argv]
 */
async function main(argv = process.argv.slice(2)) {
  const parser = new ArgumentParser({
    description: 'A simulated Spot robot (gRPC services of the Spot API), for testing.',
  });
  parser.add_argument('--host', { default: '0.0.0.0', help: 'Address to listen on (default: all interfaces).' });
  parser.add_argument('--port', { type: 'int', default: 443, help: 'Port (default: 443, the port of a real robot).' });
  parser.add_argument('--dev', '--insecure', {
    action: 'store_true',
    dest: 'insecure',
    help: 'Insecure channel (no TLS). The SDKs connect with TLS by default.',
  });
  parser.add_argument('--config', { help: 'JSON file merged over the default configuration (src/config.js).' });
  parser.add_argument('--state-file', {
    help: 'JSON file of the persistent state (default: ./data/robot_state.json).',
  });
  parser.add_argument('--reset-state', { action: 'store_true', help: 'Ignore the persisted state.' });
  parser.add_argument('--no-persist', { action: 'store_true', help: 'Do not save the state.' });
  parser.add_argument('--username', {
    help: 'Account of the robot (with --password); any credentials are accepted by default.',
  });
  parser.add_argument('--password', { help: 'Password of the account.' });
  parser.add_argument('--no-arm', { action: 'store_true', help: 'A robot without arm.' });
  parser.add_argument('--docked', { action: 'store_true', help: 'The robot starts on its dock.' });
  parser.add_argument('--clock-skew', { type: 'float', help: 'Robot time minus host time, in seconds.' });
  parser.add_argument('--no-console', { action: 'store_true', help: 'No interactive console on the standard input.' });
  parser.add_argument('-v', '--verbose', { action: 'store_true', help: 'Log every RPC.' });
  const args = parser.parse_args(argv);

  const overrides = { robot: {}, auth: {}, dock: {} };
  if (args.username || args.password) {
    overrides.auth.users = [{ username: args.username ?? '', password: args.password ?? '' }];
  }
  if (args.no_arm) overrides.robot.hasArm = false;
  if (args.docked) overrides.dock.startDocked = true;
  if (args.clock_skew !== undefined) overrides.clockSkewSec = args.clock_skew;
  LoggerUtil.getLogger('RPC');
  LoggerUtil.setLevel('RPC', args.verbose ? 'debug' : 'info');

  const { robot, stop } = await startServer({
    host: args.host,
    port: args.port,
    insecure: args.insecure,
    configFile: args.config ?? null,
    config: overrides,
    stateFile: args.state_file ?? null,
    resetState: args.reset_state,
    persist: !args.no_persist,
  });

  let stopping = false;
  const shutdown = async () => {
    if (stopping) return;
    stopping = true;
    logger.info('Stopping...');
    await stop();
    process.exit(0);
  };
  process.once('SIGINT', shutdown);
  process.once('SIGTERM', shutdown);
  robot.on('offline', ({ reboot }) => {
    if (!reboot) logger.info('Type "boot" in the console to power the robot on again.');
  });
  if (!args.no_console && process.stdin.isTTY) {
    const { startConsole } = require('./console');
    startConsole(robot, shutdown);
  }
}

if (require.main === module) {
  main().catch(err => {
    logger.error(`Fatal error: ${err.stack ?? err}`);
    process.exit(1);
  });
}

module.exports = { main, startServer };
