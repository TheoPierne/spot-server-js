'use strict';

const assert = require('node:assert/strict');
const test = require('node:test');

const { acquireBody, advance, commandRequest, estopEndpoint, makeRobot, newer, syncClock } = require('./helpers');
const basicCommandPb = require('../src/bosdyn/api/basic_command_pb');
const directoryPb = require('../src/bosdyn/api/directory_pb');
const estopPb = require('../src/bosdyn/api/estop_pb');
const fullBodyCommandPb = require('../src/bosdyn/api/full_body_command_pb');
const geometryPb = require('../src/bosdyn/api/geometry_pb');
const leasePb = require('../src/bosdyn/api/lease_pb');
const mobilityCommandPb = require('../src/bosdyn/api/mobility_command_pb');
const powerPb = require('../src/bosdyn/api/power_pb');
const robotCommandPb = require('../src/bosdyn/api/robot_command_pb');
const { PowerState } = require('../src/bosdyn/api/robot_state_pb');
const synchronizedCommandPb = require('../src/bosdyn/api/synchronized_command_pb');
const trajectoryPb = require('../src/bosdyn/api/trajectory_pb');
const { secToTimestamp } = require('../src/sim/clock');

const UseStatus = leasePb.LeaseUseResult.Status;
const CommandStatus = robotCommandPb.RobotCommandResponse.Status;
const FeedbackStatus = basicCommandPb.RobotCommandFeedbackStatus.Status;
const Motor = PowerState.MotorPowerState;

/**
 * @param {function(mobilityCommandPb.MobilityCommand.Request): void} fill
 * @returns {robotCommandPb.RobotCommand}
 */
function mobilityCommand(fill) {
  const mobility = new mobilityCommandPb.MobilityCommand.Request();
  fill(mobility);
  return new robotCommandPb.RobotCommand().setSynchronizedCommand(
    new synchronizedCommandPb.SynchronizedCommand.Request().setMobilityCommand(mobility),
  );
}

const stand = () => mobilityCommand(m => m.setStandRequest(new basicCommandPb.StandCommand.Request()));
const sit = () => mobilityCommand(m => m.setSitRequest(new basicCommandPb.SitCommand.Request()));
const velocity = (vx, endTime) =>
  mobilityCommand(m =>
    m.setSe2VelocityRequest(
      new basicCommandPb.SE2VelocityCommand.Request()
        .setSe2FrameName('flat_body')
        .setVelocity(new geometryPb.SE2Velocity().setLinear(new geometryPb.Vec2().setX(vx).setY(0)).setAngular(0))
        .setEndTime(secToTimestamp(endTime)),
    ),
  );
const trajectory = (x, y, yaw, frame, endTime) =>
  mobilityCommand(m =>
    m.setSe2TrajectoryRequest(
      new basicCommandPb.SE2TrajectoryCommand.Request()
        .setSe2FrameName(frame)
        .setEndTime(secToTimestamp(endTime))
        .setTrajectory(
          new trajectoryPb.SE2Trajectory().setPointsList([
            new trajectoryPb.SE2TrajectoryPoint().setPose(
              new geometryPb.SE2Pose().setPosition(new geometryPb.Vec2().setX(x).setY(y)).setAngle(yaw),
            ),
          ]),
        ),
    ),
  );
const fullBody = fill => {
  const request = new fullBodyCommandPb.FullBodyCommand.Request();
  fill(request);
  return new robotCommandPb.RobotCommand().setFullBodyCommand(request);
};
const safePowerOff = () => fullBody(r => r.setSafePowerOffRequest(new basicCommandPb.SafePowerOffCommand.Request()));
const selfright = () => fullBody(r => r.setSelfrightRequest(new basicCommandPb.SelfRightCommand.Request()));

/**
 * A robot with an E-Stop endpoint, the body lease and the motors on.
 * @returns {{robot: import('../src/robot').Robot, lease: object, keepAlive: function(): void}}
 */
function poweredRobot() {
  const robot = makeRobot();
  const { checkIn } = estopEndpoint(robot);
  let lease = acquireBody(robot);
  const keepAlive = () => checkIn();
  lease = newer(lease);
  const result = robot.power.command(powerPb.PowerCommandRequest.Request.REQUEST_ON_MOTORS);
  assert.equal(result.status, powerPb.PowerCommandStatus.STATUS_IN_PROGRESS);
  advance(robot, robot.config.durations.powerOn + 0.1, keepAlive);
  assert.equal(robot.power.motorState, Motor.MOTOR_POWER_STATE_ON);
  return { robot, lease, keepAlive };
}

test('leases: acquire, already claimed, stale, take, return', () => {
  const robot = makeRobot();
  const a = robot.leases.acquire('body', 'client-a', 'user', false);
  assert.equal(a.status, 'ok');
  assert.equal(robot.leases.acquire('body', 'client-b', 'user', false).status, 'already_claimed');
  // A sublease of client A, then a newer one: OK; an older one: STATUS_OLDER.
  const sub = { ...a.lease, sequence: [...a.lease.sequence, 0] };
  assert.equal(robot.leases.use(newer(newer(sub))).status, UseStatus.STATUS_OK);
  assert.equal(robot.leases.use(newer(sub)).status, UseStatus.STATUS_OLDER);
  // Not retained: stale, another client can acquire it, and the leases of A are older.
  advance(robot, robot.config.lease.staleTimeoutSec + 0.5);
  const b = robot.leases.acquire('body', 'client-b', 'user', false);
  assert.equal(b.status, 'ok');
  const result = robot.leases.use(newer(newer(newer(sub))), ['mobility']);
  assert.equal(result.status, UseStatus.STATUS_OLDER);
  assert.equal(result.owner.clientName, 'client-b');
  assert.deepEqual(result.latestKnown.sequence, b.lease.sequence);
  // Take, return: a returned lease is revoked; a wrong epoch is detected.
  const c = robot.leases.acquire('body', 'client-c', 'user', true);
  assert.equal(robot.leases.returnLease(c.lease), 'ok');
  assert.equal(robot.leases.use(c.lease).status, UseStatus.STATUS_REVOKED);
  assert.equal(robot.leases.use({ ...c.lease, epoch: 'other' }).status, UseStatus.STATUS_WRONG_EPOCH);
  assert.equal(robot.leases.use({ ...c.lease, sequence: [99] }).status, UseStatus.STATUS_INVALID_LEASE);
  assert.equal(robot.leases.use({ ...c.lease, resource: 'tail' }).status, UseStatus.STATUS_UNMANAGED);
});

test('E-Stop: estopped until an endpoint checks in, challenges, timeout sits the robot down', () => {
  const robot = makeRobot();
  assert.equal(robot.estop.isEstopped(), true);
  const refused = robot.power.command(powerPb.PowerCommandRequest.Request.REQUEST_ON_MOTORS);
  assert.equal(refused.status, powerPb.PowerCommandStatus.STATUS_ESTOPPED);

  const { checkIn } = estopEndpoint(robot, 5);
  assert.equal(robot.estop.isEstopped(), false);
  // An incorrect response does not refresh the timeout.
  const wrong = robot.estop.checkIn(
    new estopPb.EstopEndpoint().setUniqueId([...robot.estop.registrations.values()][0].endpoint.uniqueId),
    '1',
    '2',
    estopPb.EstopStopLevel.ESTOP_LEVEL_NONE,
  );
  assert.equal(wrong.status, estopPb.EstopCheckInResponse.Status.STATUS_INCORRECT_CHALLENGE_RESPONSE);

  // Standing, then the endpoint stops checking in: SETTLE_THEN_CUT sits the robot down and cuts the power.
  let lease = acquireBody(robot);
  robot.power.command(powerPb.PowerCommandRequest.Request.REQUEST_ON_MOTORS);
  advance(robot, 3.2, () => checkIn());
  lease = newer(lease);
  assert.equal(robot.commands.submit(commandRequest(stand(), lease), 'test').status, CommandStatus.STATUS_OK);
  advance(robot, 2, () => checkIn());
  assert.equal(robot.body.posture, 'standing');
  advance(robot, 5.5);
  assert.equal(robot.estop.systemLevel().level, estopPb.EstopStopLevel.ESTOP_LEVEL_SETTLE_THEN_CUT);
  advance(robot, 3);
  assert.equal(robot.body.posture, 'sitting');
  assert.notEqual(robot.power.motorState, Motor.MOTOR_POWER_STATE_ON);
});

test('E-Stop CUT: the standing robot collapses at once', () => {
  const { robot, lease, keepAlive } = poweredRobot();
  assert.equal(robot.commands.submit(commandRequest(stand(), lease), 'test').status, CommandStatus.STATUS_OK);
  advance(robot, 2, keepAlive);
  assert.equal(robot.body.posture, 'standing');
  robot.estop.setHardwareEstop(true);
  assert.equal(robot.body.posture, 'sitting');
  assert.equal(robot.power.motorState, Motor.MOTOR_POWER_STATE_POWERING_OFF);
  advance(robot, 1);
  assert.equal(robot.power.motorState, Motor.MOTOR_POWER_STATE_OFF);
});

test('commands: validation like a real robot', () => {
  const robot = makeRobot();
  const { checkIn } = estopEndpoint(robot);
  let lease = acquireBody(robot);
  lease = newer(lease);
  // Not powered on.
  assert.equal(
    robot.commands.submit(commandRequest(stand(), lease), 'test').status,
    CommandStatus.STATUS_NOT_POWERED_ON,
  );
  robot.power.command(powerPb.PowerCommandRequest.Request.REQUEST_ON_MOTORS);
  advance(robot, 3.2, () => checkIn());
  // An end time needs the time sync, in the future, not too far.
  const now = robot.clock.now();
  assert.equal(
    robot.commands.submit(commandRequest(velocity(0.5, now + 2), newer(lease)), 'test').status,
    CommandStatus.STATUS_NO_TIMESYNC,
  );
  const clock = syncClock(robot);
  assert.equal(
    robot.commands.submit(commandRequest(velocity(0.5, now - 1), newer(lease), clock), 'test').status,
    CommandStatus.STATUS_EXPIRED,
  );
  assert.equal(
    robot.commands.submit(commandRequest(velocity(0.5, now + 1000), newer(lease), clock), 'test').status,
    CommandStatus.STATUS_TOO_DISTANT,
  );
  // Unknown frame.
  const response = robot.commands.submit(
    commandRequest(trajectory(1, 0, 0, 'moon', now + 5), newer(lease), clock),
    'test',
  );
  assert.equal(response.status, CommandStatus.STATUS_UNKNOWN_FRAME);
  // The rejected commands did not use their lease: an older sublease is still accepted.
  const accepted = robot.commands.submit(commandRequest(stand(), lease), 'test');
  assert.equal(accepted.status, CommandStatus.STATUS_OK);
  assert.equal(accepted.leaseUseResult.getStatus(), UseStatus.STATUS_OK);
  // Once a lease is used, an older one is rejected (two clients sharing a lease, or a replayed command).
  const older = robot.commands.submit(commandRequest(stand(), { ...lease, sequence: [lease.sequence[0], 0] }), 'test');
  assert.equal(older.status, CommandStatus.STATUS_UNKNOWN);
  assert.equal(older.leaseUseResult.getStatus(), UseStatus.STATUS_OLDER);
});

test('commands: stand, walk with a velocity, trajectory, sit, safe power off', () => {
  const { robot, lease, keepAlive } = poweredRobot();
  const clock = syncClock(robot);
  let current = lease;
  const submit = command => {
    current = newer(current);
    const result = robot.commands.submit(commandRequest(command, current, clock), 'test');
    assert.equal(result.status, CommandStatus.STATUS_OK, result.message);
    return result.id;
  };
  const feedback = id => robot.commands.feedback(id);

  const standId = submit(stand());
  const standStatus = () =>
    feedback(standId).getSynchronizedFeedback().getMobilityCommandFeedback().getStandFeedback().getStatus();
  assert.equal(standStatus(), basicCommandPb.StandCommand.Feedback.Status.STATUS_IN_PROGRESS);
  advance(robot, 2, keepAlive);
  assert.equal(standStatus(), basicCommandPb.StandCommand.Feedback.Status.STATUS_IS_STANDING);

  // 0.5 m/s for 2 s: about 1 m, then the command times out and the robot stops.
  const start = { ...robot.body.footprint };
  const velocityId = submit(velocity(0.5, robot.clock.now() + 2));
  advance(robot, 3, keepAlive);
  const walked = robot.body.footprint.x - start.x;
  assert.ok(walked > 0.8 && walked < 1.05, `walked ${walked}`);
  assert.equal(
    feedback(standId).getSynchronizedFeedback().getMobilityCommandFeedback().getStatus(),
    FeedbackStatus.STATUS_COMMAND_OVERRIDDEN,
  );
  assert.equal(
    feedback(velocityId).getSynchronizedFeedback().getMobilityCommandFeedback().getStatus(),
    FeedbackStatus.STATUS_COMMAND_TIMED_OUT,
  );
  assert.equal(robot.body.isWalking(), false);

  // A trajectory 1 m to the left, in the frame of the body.
  const trajectoryId = submit(trajectory(0, 1, 0, 'body', robot.clock.now() + 10));
  advance(robot, 6, keepAlive);
  const trajectoryFeedback = feedback(trajectoryId)
    .getSynchronizedFeedback()
    .getMobilityCommandFeedback()
    .getSe2TrajectoryFeedback();
  assert.equal(trajectoryFeedback.getStatus(), basicCommandPb.SE2TrajectoryCommand.Feedback.Status.STATUS_STOPPED);
  assert.equal(
    trajectoryFeedback.getBodyMovementStatus(),
    basicCommandPb.SE2TrajectoryCommand.Feedback.BodyMovementStatus.BODY_STATUS_SETTLED,
  );
  assert.ok(Math.abs(robot.body.footprint.y - start.y - 1) < 0.03);

  const sitId = submit(sit());
  advance(robot, 2, keepAlive);
  assert.equal(
    feedback(sitId).getSynchronizedFeedback().getMobilityCommandFeedback().getSitFeedback().getStatus(),
    basicCommandPb.SitCommand.Feedback.Status.STATUS_IS_SITTING,
  );
  const offId = submit(safePowerOff());
  advance(robot, 1, keepAlive);
  assert.equal(
    feedback(offId).getFullBodyFeedback().getSafePowerOffFeedback().getStatus(),
    basicCommandPb.SafePowerOffCommand.Feedback.Status.STATUS_POWERED_OFF,
  );
  assert.equal(robot.power.motorState, Motor.MOTOR_POWER_STATE_OFF);
});

test('a fall: behavior fault, self-right, clear', () => {
  const { robot, lease, keepAlive } = poweredRobot();
  let current = newer(lease);
  robot.commands.submit(commandRequest(stand(), current), 'test');
  advance(robot, 2, keepAlive);
  robot.body.fall();
  current = newer(current);
  assert.equal(
    robot.commands.submit(commandRequest(stand(), current), 'test').status,
    CommandStatus.STATUS_BEHAVIOR_FAULT,
  );
  current = newer(current);
  const result = robot.commands.submit(commandRequest(selfright(), current), 'test');
  assert.equal(result.status, CommandStatus.STATUS_OK);
  advance(robot, robot.config.durations.selfRight + 0.2, keepAlive);
  assert.equal(
    robot.commands.feedback(result.id).getFullBodyFeedback().getSelfrightFeedback().getStatus(),
    basicCommandPb.SelfRightCommand.Feedback.Status.STATUS_COMPLETED,
  );
  assert.equal(robot.faults.hasBehaviorFaults(), false);
  current = newer(current);
  assert.equal(robot.commands.submit(commandRequest(stand(), current), 'test').status, CommandStatus.STATUS_OK);
});

test('the battery drains with the activity and charges on the dock', () => {
  const robot = makeRobot({ battery: { timeScale: 100 } });
  const start = robot.battery.percent();
  advance(robot, 10);
  const idle = start - robot.battery.percent();
  assert.ok(idle > 0, 'drains while idle');
  robot.docking.placeOnDock(robot.config.dock.id);
  advance(robot, 3);
  const before = robot.battery.percent();
  advance(robot, 10);
  assert.ok(robot.battery.percent() > before, 'charges on the dock');
});

test('reboot: new lease epoch, E-Stop configuration and motors reset, position kept', () => {
  const { robot, keepAlive } = poweredRobot();
  const epoch = robot.leases.epoch;
  robot.body.teleport(1, 2, 0.5);
  robot.boot();
  keepAlive();
  assert.notEqual(robot.leases.epoch, epoch);
  assert.equal(robot.power.motorState, Motor.MOTOR_POWER_STATE_OFF);
  assert.equal(robot.estop.isEstopped(), true);
  assert.deepEqual(robot.body.footprint, { x: 1, y: 2, yaw: 0.5 });
  // The odometry restarts at the body.
  const odom = robot.body.odomInWorld();
  assert.ok(Math.abs(odom.x - 1) < 1e-9 && Math.abs(odom.y - 2) < 1e-9);
});

test('directory: the registered services keep their liveness with heartbeats', () => {
  const robot = makeRobot();
  const entry = new directoryPb.ServiceEntry().setName('my-payload').setType('my.Service').setLivenessTimeoutSecs(2);
  robot.directory.register(entry, null);
  advance(robot, 1.5);
  // The keep-alive of the SDK registers again: a heartbeat (STATUS_ALREADY_EXISTS).
  robot.directory.register(entry, null);
  advance(robot, 1.5);
  assert.equal(robot.faults.serviceFaults.size, 0);
  advance(robot, 1);
  assert.equal(robot.faults.serviceFaults.size, 1);
  robot.directory.register(entry, null);
  advance(robot, 0.1);
  assert.equal(robot.faults.serviceFaults.size, 0);
  advance(robot, 3);
  assert.equal(robot.faults.serviceFaults.size, 1);
  robot.directory.unregister('my-payload');
  assert.equal(robot.faults.serviceFaults.size, 0);
});

test('power cycle: the robot reports the success, then reboots', () => {
  const robot = makeRobot();
  const result = robot.power.command(powerPb.PowerCommandRequest.Request.REQUEST_CYCLE_ROBOT);
  assert.equal(result.status, powerPb.PowerCommandStatus.STATUS_IN_PROGRESS);
  assert.equal(robot.power.feedback(result.id).status, powerPb.PowerCommandStatus.STATUS_IN_PROGRESS);
  advance(robot, 1);
  assert.equal(robot.power.feedback(result.id).status, powerPb.PowerCommandStatus.STATUS_SUCCESS);
  robot.stop();
});
