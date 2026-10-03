'use strict';

const estopPb = require('../src/bosdyn/api/estop_pb');
const robotCommandPb = require('../src/bosdyn/api/robot_command_pb');
const { loadConfig } = require('../src/config');
const { Robot } = require('../src/robot');
const { RobotClock, secToDuration } = require('../src/sim/clock');
const { leaseToProto } = require('../src/sim/lease');

/**
 * A simulated robot with a manual clock (the time only moves with advance()).
 * @param {object} [overrides] Merged over the default configuration.
 * @returns {Robot}
 */
function makeRobot(overrides = {}) {
  return new Robot(loadConfig(null, overrides), { clock: new RobotClock({ manual: true }) });
}

/**
 * Moves the time of the robot, in steps of the simulation.
 * @param {Robot} robot
 * @param {number} seconds
 * @param {function(): void} [everyStep] Called at each step (e.g. the check-ins of a client).
 */
function advance(robot, seconds, everyStep = null) {
  const step = 0.02;
  for (let t = 0; t < seconds - 1e-9; t += step) {
    robot.clock.advance(Math.min(step, seconds - t));
    robot.sync();
    everyStep?.();
  }
}

/**
 * Marks a client clock as synchronized (the round trips of the time sync are tested with gRPC).
 * @param {Robot} robot
 * @param {string} [id]
 * @returns {string}
 */
function syncClock(robot, id = 'test-clock') {
  robot.timeSync.clocks.set(id, {
    clientName: 'test',
    samples: [{ rttNsec: 1000000n, skewNsec: 0n }],
    synced: true,
    lastUpdate: 0,
  });
  return id;
}

/**
 * An E-Stop endpoint which checks in like the EstopKeepAlive of the SDK.
 * @param {Robot} robot
 * @param {number} [timeoutSec]
 * @returns {{checkIn: function(number=): number, endpoint: estopPb.EstopEndpoint}}
 */
function estopEndpoint(robot, timeoutSec = 9) {
  const config = new estopPb.EstopConfig().setEndpointsList([
    new estopPb.EstopEndpoint().setRole('PDB_rooted').setName('test').setTimeout(secToDuration(timeoutSec)),
  ]);
  robot.estop.setConfig(config, robot.estop.config.uniqueId);
  const registration = robot.estop.register(
    robot.estop.config.uniqueId,
    new estopPb.EstopEndpoint().setRole('PDB_rooted'),
    new estopPb.EstopEndpoint().setRole('PDB_rooted').setName('test').setTimeout(secToDuration(timeoutSec)),
  );
  const endpoint = new estopPb.EstopEndpoint().setRole('PDB_rooted').setUniqueId(registration.endpoint.uniqueId);
  let challenge = null;
  const checkIn = (level = estopPb.EstopStopLevel.ESTOP_LEVEL_NONE) => {
    const response = challenge === null ? '0' : BigInt.asUintN(64, ~BigInt(challenge)).toString();
    const result = robot.estop.checkIn(endpoint, challenge ?? '0', response, level);
    challenge = result.challenge;
    return result.status;
  };
  // The first check-in only gets a challenge.
  checkIn();
  checkIn();
  return { checkIn, endpoint };
}

/**
 * Acquires the body lease.
 * @param {Robot} robot
 * @param {string} [client]
 * @returns {import('../src/sim/lease').LeaseData} A sublease, like the wallet of the SDK.
 */
function acquireBody(robot, client = 'test-client') {
  const result = robot.leases.acquire('body', client, 'user', false);
  return {
    ...result.lease,
    sequence: [...result.lease.sequence, 0],
    clientNames: [...result.lease.clientNames, client],
  };
}

/**
 * The next lease of a wallet (Lease.createNewer() of the SDK).
 * @param {import('../src/sim/lease').LeaseData} lease
 * @returns {import('../src/sim/lease').LeaseData}
 */
function newer(lease) {
  const sequence = [...lease.sequence];
  sequence[sequence.length - 1] += 1;
  return { ...lease, sequence };
}

/**
 * A RobotCommandRequest.
 * @param {import('../src/bosdyn/api/robot_command_pb').RobotCommand} command
 * @param {import('../src/sim/lease').LeaseData} lease
 * @param {string} [clock]
 * @returns {robotCommandPb.RobotCommandRequest}
 */
function commandRequest(command, lease, clock = '') {
  return new robotCommandPb.RobotCommandRequest()
    .setCommand(command)
    .setLease(leaseToProto(lease))
    .setClockIdentifier(clock);
}

module.exports = { acquireBody, advance, commandRequest, estopEndpoint, makeRobot, newer, syncClock };
