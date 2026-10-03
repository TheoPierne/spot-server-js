'use strict';

const process = require('node:process');
const readline = require('node:readline');

const { SystemFault } = require('./bosdyn/api/robot_state_pb');

const HELP = `Commands of the simulated robot:
  state                      Summary of the robot (power, posture, battery, leases, E-Stop, faults...)
  estop [press|release]      Press or release the hardware E-Stop of the robot
  fall [left|right]          The robot falls on its side (behavior fault, it must self-right)
  battery <percent>          Set the charge of the battery
  shore [on|off]             Connect the robot to wall power (the motors cannot power on)
  fault <name> [message]     Raise a system fault (fault --critical <name> to block the motor power)
  clear <name>               Clear a system fault
  grasp | release            The gripper holds (or releases) an item
  stale                      Mark all the leases stale (as if their owner lost the connection)
  dock | undock              Put the robot on its dock, or off it
  teleport <x> <y> [yaw]     Move the robot in the room (the odometry does not see it)
  license [expire|valid]     Make the license expire (the motors cannot power on)
  reboot | shutdown | boot   Reboot the robot, power it off, power it back on
  quit                       Stop the simulator`;

/**
 * The interactive console of the simulator, to act on the robot like the real world would.
 * @param {import('./robot').Robot} robot
 * @param {function(): Promise<void>} stop
 * @param {{input?: NodeJS.ReadableStream, output?: NodeJS.WritableStream}} [streams] The standard input and output
 *   by default.
 * @returns {readline.Interface}
 */
function startConsole(robot, stop, { input = process.stdin, output = process.stdout } = {}) {
  const rl = readline.createInterface({ input, output, prompt: 'spot-sim> ' });
  const print = text => output.write(`${text}\n`);
  const commands = {
    help: () => print(HELP),
    state: () => print(robot.describe().join('\n')),
    estop: ([action = 'press']) => robot.estop.setHardwareEstop(action !== 'release'),
    fall: ([side = 'left']) => robot.body.fall(side === 'right' ? -1 : 1),
    battery: ([percent]) => {
      const value = Number(percent);
      if (!Number.isFinite(value)) return print('Usage: battery <percent>');
      robot.battery.setPercent(value);
      return print(`Battery: ${robot.battery.describe()}`);
    },
    shore: ([state = 'on']) => {
      robot.power.shorePower = state !== 'off';
      if (robot.power.shorePower) robot.power.cutMotorPower('shore power connected');
      print(`Shore power ${robot.power.shorePower ? 'connected' : 'disconnected'}`);
    },
    fault: args => {
      const critical = args[0] === '--critical';
      const [name, ...message] = critical ? args.slice(1) : args;
      if (!name) return print('Usage: fault [--critical] <name> [message]');
      return robot.faults.addSystemFault(name, {
        message: message.join(' ') || `Simulated fault ${name}.`,
        severity: critical ? SystemFault.Severity.SEVERITY_CRITICAL : SystemFault.Severity.SEVERITY_WARN,
        blocking: critical,
      });
    },
    clear: ([name]) => print(robot.faults.clearSystemFault(name) ? `Fault "${name}" cleared` : `No fault "${name}"`),
    grasp: () => {
      if (!robot.arm) return print('The robot has no arm.');
      robot.arm.holdingItem = true;
      return print('The gripper holds an item.');
    },
    release: () => {
      if (robot.arm) robot.arm.holdingItem = false;
    },
    stale: () => robot.leases.markStale(),
    dock: () => print(robot.docking.placeOnDock(robot.config.dock.id) ? 'The robot is on its dock.' : 'No dock.'),
    undock: () => {
      const { footprint } = robot.body;
      robot.body.teleport(
        footprint.x + 1.5 * Math.cos(footprint.yaw),
        footprint.y + 1.5 * Math.sin(footprint.yaw),
        footprint.yaw,
      );
      robot.sync();
    },
    teleport: ([x, y, yaw = '0']) => {
      if (!Number.isFinite(Number(x)) || !Number.isFinite(Number(y))) return print('Usage: teleport <x> <y> [yaw]');
      return robot.body.teleport(Number(x), Number(y), Number(yaw));
    },
    license: ([state = 'expire']) => robot.license.setExpired(state !== 'valid'),
    reboot: () => robot.scheduleShutdown({ reboot: true, delaySec: 0 }),
    shutdown: () => robot.scheduleShutdown({ reboot: false, delaySec: 0 }),
    boot: () => robot.boot(),
    quit: () => {
      rl.close();
    },
  };
  commands.exit = commands.quit;
  commands.s = commands.state;

  rl.on('line', line => {
    const [name, ...args] = line.trim().split(/\s+/);
    if (name) {
      const command = commands[name.toLowerCase()];
      if (command) {
        try {
          command(args);
        } catch (err) {
          print(`Error: ${err.message}`);
        }
      } else {
        print(`Unknown command "${name}" (help lists the commands).`);
      }
    }
    if (!closed) rl.prompt();
  });
  let closed = false;
  rl.on('close', () => {
    closed = true;
    stop();
  });
  print('Type "help" for the commands of the simulated robot.');
  rl.prompt();
  return rl;
}

module.exports = { startConsole };
