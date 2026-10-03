'use strict';

const assert = require('node:assert/strict');
const { PassThrough } = require('node:stream');
const test = require('node:test');
const { setImmediate } = require('node:timers');

const { makeRobot } = require('./helpers');
const { EstopStopLevel } = require('../src/bosdyn/api/estop_pb');
const { startConsole } = require('../src/console');

test('console: the events of the real world', async () => {
  const robot = makeRobot();
  const input = new PassThrough();
  const output = new PassThrough();
  let text = '';
  output.on('data', data => {
    text += data.toString();
  });
  let stopped = false;
  const rl = startConsole(
    robot,
    () => {
      stopped = true;
    },
    { input, output },
  );
  const type = async line => {
    input.write(`${line}\n`);
    await new Promise(resolve => setImmediate(resolve));
  };

  await type('estop press');
  assert.equal(robot.estop.hardwareEstopped, true);
  assert.equal(robot.estop.systemLevel().details.includes('hardware E-Stop'), true);
  await type('estop release');
  assert.equal(robot.estop.hardwareEstopped, false);
  assert.equal(robot.estop.systemLevel().level, EstopStopLevel.ESTOP_LEVEL_CUT);

  await type('battery 7');
  assert.ok(Math.abs(robot.battery.percent() - 7) < 1e-9);
  assert.ok(robot.faults.systemFaults.has('battery_low'));

  await type('fall right');
  assert.equal(robot.body.posture, 'fallen');
  assert.equal(robot.faults.hasBehaviorFaults(), true);

  await type('fault --critical motor_fault Overheated knee');
  assert.equal(robot.faults.blockingSystemFaults().length, 1);
  await type('clear motor_fault');
  assert.equal(robot.faults.blockingSystemFaults().length, 0);

  await type('grasp');
  assert.equal(robot.arm.holdingItem, true);
  await type('teleport 1 2 0.5');
  assert.deepEqual(robot.body.footprint, { x: 1, y: 2, yaw: 0.5 });
  await type('dock');
  assert.equal(robot.docking.isDocked(), true);

  await type('state');
  assert.match(text, /power: motors OFF/);
  assert.match(text, /docked at 520/);
  await type('nope');
  assert.match(text, /Unknown command "nope"/);

  await type('quit');
  assert.equal(stopped, true);
  rl.close();
});
