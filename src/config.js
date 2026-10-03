'use strict';

const { readFileSync } = require('node:fs');

/**
 * The default configuration of the simulated robot. A JSON file given with --config is merged over it (objects are
 * merged, arrays and values replaced), e.g. {"robot": {"hasArm": false}, "durations": {"powerOn": 1}}.
 *
 * The durations, speeds and dimensions are close to a real Spot, but they are estimates: Boston Dynamics does not
 * document most of them.
 */
const DEFAULT_CONFIG = {
  robot: {
    serialNumber: 'spot-BD-23110001',
    nickname: 'spot-sim',
    species: 'spot',
    version: 'V3',
    computerSerialNumber: 'cpu-00031415',
    softwareVersion: { major: 5, minor: 2, patch: 0 },
    softwareName: 'spot-sim',
    hasArm: true,
    // The software release that the robot reports (RobotSoftwareRelease.api_version).
    apiVersion: '5.2.0',
  },
  auth: {
    // Accounts of the robot, e.g. [{"username": "user", "password": "password"}]. When there are none, the robot
    // accepts any username and password (the behavior of the previous versions of the simulator).
    users: [],
    // The tokens of a real robot are JWTs valid for 12 hours.
    tokenLifetimeSec: 12 * 3600,
    // After six consecutive failed attempts, a real robot locks the authentication out for one minute.
    maxFailedAttempts: 6,
    lockoutSec: 60,
  },
  // Robot time minus host time, in seconds. Clients which do not convert their times with the time sync service fail
  // with a skew, like with a real robot.
  clockSkewSec: 0,
  timeSync: {
    // Consistent measurements needed before STATUS_OK.
    samplesNeeded: 3,
  },
  lease: {
    // A lease which is neither retained nor used for this long becomes stale: another client can acquire it. The
    // value of a real robot is not documented.
    staleTimeoutSec: 6,
  },
  estop: {
    // Time to sit before the power is cut, after a SETTLE_THEN_CUT, when the endpoint has no cut_power_timeout.
    settleTimeSec: 3,
  },
  durations: {
    powerOn: 3,
    powerOff: 0.5,
    standUp: 1.6,
    sitDown: 1.6,
    selfRight: 6,
    batteryChangePose: 4,
    payloadEstimation: 8,
    reboot: 8,
    armStowUnstow: 1.5,
  },
  mobility: {
    // Maximum speeds (m/s, rad/s) and accelerations.
    maxVelX: 1.6,
    maxVelY: 0.6,
    maxVelYaw: 1.5,
    defaultTrajectoryVel: 1.0,
    maxAccel: 1.5,
    maxYawAccel: 2.5,
    // Nominal height of the body above the ground, standing and sitting.
    standHeight: 0.52,
    sitHeight: 0.17,
    minBodyHeightOffset: -0.2,
    maxBodyHeightOffset: 0.12,
    // Tolerances of the trajectory commands.
    goalPositionTolerance: 0.02,
    goalYawTolerance: 0.03,
    nearGoalDistance: 0.5,
    // Maximum duration of the commands with an end time (STATUS_TOO_DISTANT beyond).
    maxCommandDurationSec: 300,
    // Drift of the odom frame relative to the vision frame, per meter walked.
    odomDriftPerMeter: 0.01,
  },
  battery: {
    capacityWh: 605,
    initialPercent: 92,
    // Power drawn by the robot, in W.
    idlePowerW: 110,
    standingPowerW: 260,
    walkingPowerW: 420,
    chargePowerW: 450,
    // Multiplies the energy drawn and charged, to test the battery behaviors quickly.
    timeScale: 1,
    // Below this charge, the robot sits down and powers off its motors.
    criticalPercent: 3,
    lowPercent: 10,
  },
  // The room of the simulated world (the robot boots at the origin, facing +x). The walls are obstacles.
  world: {
    room: { minX: -3, maxX: 8, minY: -4, maxY: 4, height: 3 },
    // AprilTag fiducials on the walls: id, position (the center of the tag, in the room) and yaw (the direction the
    // tag faces).
    fiducials: [
      { id: 200, x: 7.99, y: 0, z: 0.6, yaw: Math.PI },
      { id: 201, x: 3, y: 3.99, z: 0.6, yaw: -Math.PI / 2 },
      { id: 202, x: 3, y: -3.99, z: 0.6, yaw: Math.PI / 2 },
      { id: 203, x: -2.99, y: 0, z: 0.6, yaw: 0 },
    ],
    fiducialSizeMm: 146,
    // Fiducials are seen from this distance.
    detectionRange: 4.5,
  },
  dock: {
    // A Spot Dock (ids 520 to 549): the center of its base, and the direction its fiducial faces (the robot comes
    // from that side, facing the fiducial, and sits over the base).
    id: 520,
    x: 2.5,
    y: -2,
    yaw: Math.PI / 2,
    // The robot starts on the dock.
    startDocked: false,
  },
  physics: {
    // Simulation rate in Hz.
    rate: 50,
  },
};

/**
 * @param {any} value
 * @returns {boolean}
 */
function isPlainObject(value) {
  return value !== null && typeof value === 'object' && !Array.isArray(value);
}

/**
 * Deep merge of plain objects (arrays and values are replaced).
 * @param {object} base
 * @param {object} override
 * @returns {object}
 */
function mergeConfig(base, override) {
  const result = structuredClone(base);
  for (const [key, value] of Object.entries(override ?? {})) {
    if (isPlainObject(value) && isPlainObject(result[key])) {
      result[key] = mergeConfig(result[key], value);
    } else {
      result[key] = structuredClone(value);
    }
  }
  return result;
}

/**
 * @param {?string} file A JSON file merged over the default configuration.
 * @param {object} [overrides] Merged last (the options of the command line).
 * @returns {typeof DEFAULT_CONFIG}
 */
function loadConfig(file = null, overrides = {}) {
  let config = DEFAULT_CONFIG;
  if (file) config = mergeConfig(config, JSON.parse(readFileSync(file, 'utf8')));
  return mergeConfig(config, overrides);
}

module.exports = {
  DEFAULT_CONFIG,
  loadConfig,
  mergeConfig,
};
