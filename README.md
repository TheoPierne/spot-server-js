# spot-server-js

A simulated [Spot](https://bostondynamics.com/products/spot/) robot, to test the applications of the Spot API without a
robot: the gRPC services of the API, over TLS, backed by a simulation of the robot. The robot has a state: commands
change it, and the responses follow, with the rules of a real robot (leases, E-Stop, time sync, power, faults...).

It works with [spot-sdk-js](https://github.com/TheoPierne/spot-sdk-js) and with the official Python SDK of Boston
Dynamics: their examples (hello_spot, the arm examples, docking...) run against it.

## What is simulated

- **Authentication**: user tokens (JWTs valid 12 hours, refreshed with a token), the services which need a token answer
  `UNAUTHENTICATED` without one, six failed logins lock the authentication out for a minute. Any username and password
  are accepted unless accounts are configured.
- **Directory**: the services of the robot, and the services registered by payloads, with their liveness: a service
  which stops its heartbeats gets a service fault (the simulator does not route the RPCs of the registered services).
- **Time sync**: a robot clock (optionally skewed from the computer with `--clock-skew`), clock identifiers, several
  measurements before `STATUS_OK`. The commands with an end time need the time sync, and are `STATUS_EXPIRED` or
  `STATUS_TOO_DISTANT` when their end time is wrong.
- **Leases**: the resource tree (`body`, `mobility`, `full-arm`, `arm`, `gripper`), acquire, take, return, retain,
  sub-leases. An older lease is rejected (`STATUS_OLDER`), a returned one is revoked, a lease which is not retained
  becomes stale and another client can acquire it.
- **E-Stop**: configuration, endpoint registration, check-ins with challenges. The robot is E-Stopped until an endpoint
  checks in; an endpoint which stops checking in makes the robot sit down and power off (`SETTLE_THEN_CUT`), then cuts
  the power (`CUT`). The hardware E-Stop can be pressed from the console.
- **Power**: powering the motors on takes a few seconds, and is refused when E-Stopped, on shore power, with a critical
  fault, a keepalive motors-off action or an expired license. Cutting the power while standing makes the robot collapse.
  The robot can be powered off or rebooted (it stops answering, then reboots: new lease epoch, E-Stop and odometry).
- **Robot commands**: stand (with body height and orientation, body trajectories), sit, velocity, SE2 trajectories,
  stance, stop, freeze, self-right, safe power off, battery change pose, payload estimation; the arm (stow, ready,
  carry, joint moves, Cartesian and gaze commands, velocity, drag, stop) and the gripper. Commands are refused like on a
  robot (lease, time sync, motors off, behavior faults, unknown frames...), override each other, and their feedback
  follows the motion (in progress, at goal, overridden, timed out...).
- **Robot state**: power, battery (it drains with the activity and charges on the dock), joints of the legs and of the
  arm, feet, frame tree (`body`, `odom`, `vision`, `flat_body`, `gpe`, `hand`; the odometry drifts slowly), velocities,
  E-Stop states, system, behavior and service faults, motor temperatures, metrics, hardware configuration.
- **Images**: the cameras of a Spot (fisheye, depth, depth in the visual frame, the cameras of the gripper) render the
  room around the robot (checkered floor, walls, fiducials, dock) from its pose, in JPEG or raw, with the requested
  quality, pixel format and resize ratio.
- **World objects**: the AprilTag fiducials and the dock that the robot sees when it is close, and the objects added by
  clients (with their lifetime), like the no-go regions.
- **Docking**: the robot walks to the dock, walks onto it, sits down and powers off; it charges while docked.
  Undocking walks back to the prep pose.
- **Keepalive** policies (events, auto return, motors off, robot off, stale leases, halt), **auto return** (the robot
  walks back along its recorded path when its client stops controlling it), **fault service**, **data buffer** and
  **data service**, **license**, **infrared emitters**, and simplified **missions** and **choreography**.

The other services of a real robot (GraphNav, data acquisition, Spot CAM, network compute bridge...) are not simulated.
The durations, speeds and dimensions are close to a real Spot, but most of them are estimates: Boston Dynamics does not
document them.

## Installation

Node.js 22 or newer.

```sh
git clone https://github.com/TheoPierne/spot-server-js.git
cd spot-server-js
npm install
```

## Usage

```sh
npm start
```

The robot listens on port 443 of all the interfaces, like a real robot. Some options:

| Option | |
| --- | --- |
| `--host 127.0.0.1 --port 8443` | Address and port (`--port 0` for a free port). |
| `--username user --password secret` | An account: only these credentials are accepted. |
| `--no-arm` | A robot without arm. |
| `--docked` | The robot starts on its dock. |
| `--clock-skew 30` | The clock of the robot is 30 s ahead of the computer. |
| `--config my-robot.json` | A configuration file, merged over the defaults of [src/config.js](src/config.js). |
| `--state-file`, `--reset-state`, `--no-persist` | The persistent state (see below). |
| `--dev` | No TLS. |
| `-v` | Log every RPC. |

### Connecting a client

The robot uses the certificates of `src/resources`: clients must trust `src/resources/ca.crt` (the certificate of the
server is for `*.spot.robot`, the authorities of the services, like on a real robot).

With spot-sdk-js, set the `BOSDYN_CA_CERT` environment variable:

```sh
BOSDYN_CA_CERT=/path/to/spot-server-js/src/resources/ca.crt node hello_spot.js 127.0.0.1
```

On another port than 443, call `robot.updateSecureChannelPort(port)` after `sdk.createRobot()`.

With the Python SDK: `sdk.load_robot_cert('/path/to/spot-server-js/src/resources/ca.crt')` after
`create_standard_sdk()`, and `robot.update_secure_channel_port(port)` on another port than 443.

Like a real robot, the robot is E-Stopped until an E-Stop endpoint checks in: run an E-Stop client (the `estop` example
of the SDKs) next to the applications which power the robot on.

### The console

In a terminal, the simulator reads commands that act on the robot like the real world would:

| Command | |
| --- | --- |
| `state` | Summary of the robot (power, posture, battery, leases, E-Stop, faults, fiducials seen...). |
| `estop [press\|release]` | Press or release the hardware E-Stop. |
| `fall [left\|right]` | The robot falls on its side: behavior fault, it must self-right. |
| `battery <percent>` | Set the charge of the battery (below 10 %: fault; below 3 %: the robot sits down and powers off). |
| `shore [on\|off]` | Wall power connected: the motors cannot power on. |
| `fault [--critical] <name> [message]`, `clear <name>` | Raise or clear a system fault (critical: the motors cannot power on). |
| `grasp`, `release` | The gripper holds an item. |
| `stale` | The leases become stale, as if their owners lost the connection. |
| `dock`, `undock` | Put the robot on its dock, or off it. |
| `teleport <x> <y> [yaw]` | Move the robot in the room (the odometry does not see it). |
| `license [expire\|valid]` | Make the license expire. |
| `reboot`, `shutdown`, `boot` | Reboot the robot, power it off, power it back on. |

### Persistent state

The physical state of the robot is saved in `data/robot_state.json`: its position, its posture when lying, its battery,
whether it is on its dock, and the key that signs the tokens (the tokens stay valid). Like a real robot that reboots,
the robot starts with its motors off, a new lease epoch, no E-Stop endpoint, and its odometry at its body.

### Configuration

[src/config.js](src/config.js) documents the configuration: the identity of the robot, the accounts, the durations
(power on, stand up...), the speeds, the battery, the room, the fiducials and the dock. For example:

```json
{
  "robot": { "nickname": "my-spot", "hasArm": false },
  "auth": { "users": [{ "username": "user", "password": "secret" }] },
  "durations": { "powerOn": 1 },
  "battery": { "initialPercent": 40, "timeScale": 20 },
  "dock": { "startDocked": true }
}
```

## Development

```sh
npm test
npm run lint
```

The generated protobuf code of `src/bosdyn/api` comes from spot-sdk-js (`npm run build` there), with the same protos.
