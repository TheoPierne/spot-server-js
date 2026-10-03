'use strict';

const { setTimeout: sleep } = require('node:timers/promises');

const parameterPb = require('../bosdyn/api/parameter_pb');
const robotStatePb = require('../bosdyn/api/robot_state_pb');
const {
  RobotStateServiceService,
  RobotStateStreamingServiceService,
} = require('../bosdyn/api/robot_state_service_grpc_pb');
const { ARM_JOINTS } = require('../sim/arm');
const { LEGS } = require('../sim/body');
const { secToDuration, secToTimestamp } = require('../sim/clock');
const { frameTreeSnapshot, poseInv, poseMul, poseToProto } = require('../sim/math');
const { GrpcError, invalidRequest, serverStreaming, unary } = require('../util');

// Links of the skeleton of the robot, with the size of their box (for the OBJ models).
const LINKS = {
  body: [0.85, 0.24, 0.18],
  ...Object.fromEntries(
    ['front_left', 'front_right', 'rear_left', 'rear_right'].flatMap(leg => [
      [`${leg}_hip`, [0.1, 0.1, 0.1]],
      [`${leg}_upper_leg`, [0.07, 0.07, 0.32]],
      [`${leg}_lower_leg`, [0.04, 0.04, 0.35]],
    ]),
  ),
};
const ARM_LINKS = {
  'arm0.link_sh0': [0.1, 0.1, 0.1],
  'arm0.link_sh1': [0.1, 0.1, 0.1],
  'arm0.link_hr0': [0.34, 0.08, 0.08],
  'arm0.link_el0': [0.1, 0.08, 0.08],
  'arm0.link_el1': [0.4, 0.07, 0.07],
  'arm0.link_wr0': [0.08, 0.08, 0.08],
  'arm0.link_wr1': [0.12, 0.08, 0.08],
  'arm0.link_fngr': [0.1, 0.04, 0.03],
};

/**
 * @param {import('../robot').Robot} robot
 * @returns {Object<string, number[]>}
 */
function linksOf(robot) {
  return robot.arm ? { ...LINKS, ...ARM_LINKS } : LINKS;
}

/**
 * An OBJ model of a box.
 * @param {string} name
 * @param {number[]} size
 * @returns {string}
 */
function boxObj(name, [x, y, z]) {
  const vertices = [];
  for (const sx of [-1, 1]) {
    for (const sy of [-1, 1]) {
      for (const sz of [-1, 1]) vertices.push(`v ${(sx * x) / 2} ${(sy * y) / 2} ${(sz * z) / 2}`);
    }
  }
  const faces = ['f 1 2 4 3', 'f 5 7 8 6', 'f 1 5 6 2', 'f 3 4 8 7', 'f 1 3 7 5', 'f 2 6 8 4'];
  return [`# ${name}`, `o ${name}`, ...vertices, ...faces, ''].join('\n');
}

/**
 * A URDF of the robot (simplified geometry, the joints of Spot).
 * @param {import('../robot').Robot} robot
 * @returns {string}
 */
function urdf(robot) {
  const lines = ['<?xml version="1.0"?>', `<robot name="${robot.config.robot.nickname}">`, '  <link name="body"/>'];
  const joint = (name, parent, child, origin, axis, limits) =>
    [
      `  <joint name="${name}" type="revolute"><parent link="${parent}"/><child link="${child}"/>`,
      origin ? `<origin xyz="${origin}"/>` : '',
      axis ? `<axis xyz="${axis}"/>` : '',
      limits ? `<limit lower="${limits[0]}" upper="${limits[1]}" effort="${limits[2]}" velocity="15"/>` : '',
      '</joint>',
    ].join('');
  const legNames = { fl: 'front_left', fr: 'front_right', hl: 'rear_left', hr: 'rear_right' };
  for (const leg of LEGS) {
    const name = legNames[leg.name];
    lines.push(
      `  <link name="${name}_hip"/>`,
      joint(`${leg.name}.hx`, 'body', `${name}_hip`, `${leg.x} ${leg.side * 0.055} 0`, '1 0 0', [-0.785, 0.785, 45]),
      `  <link name="${name}_upper_leg"/>`,
      joint(
        `${leg.name}.hy`,
        `${name}_hip`,
        `${name}_upper_leg`,
        `0 ${leg.side * 0.110945} 0`,
        '0 1 0',
        [-0.899, 2.295, 45],
      ),
      `  <link name="${name}_lower_leg"/>`,
      joint(
        `${leg.name}.kn`,
        `${name}_upper_leg`,
        `${name}_lower_leg`,
        '0.025 0 -0.32',
        '0 1 0',
        [-2.793, -0.255, 115],
      ),
    );
  }
  if (robot.arm) {
    let parent = 'body';
    for (const armJoint of ARM_JOINTS) {
      lines.push(`  <link name="arm0.link_${armJoint}"/>`, joint(`arm0.${armJoint}`, parent, `arm0.link_${armJoint}`));
      parent = `arm0.link_${armJoint}`;
    }
    lines.push('  <link name="arm0.link_fngr"/>', joint('arm0.f1x', parent, 'arm0.link_fngr'));
  }
  lines.push('</robot>', '');
  return lines.join('\n');
}

/**
 * The full state of the robot.
 * @param {import('../robot').Robot} robot
 * @returns {robotStatePb.RobotState}
 */
function buildRobotState(robot) {
  const now = robot.clock.now();
  const timestamp = secToTimestamp(now);
  const { body } = robot;
  const joints = [...body.legJointStates(now), ...(robot.arm ? robot.arm.jointStates() : [])];
  const kinematic = new robotStatePb.KinematicState()
    .setJointStatesList(joints)
    .setAcquisitionTimestamp(timestamp)
    .setTransformsSnapshot(frameTreeSnapshot(body.frameTreeEdges()))
    .setVelocityOfBodyInVision(body.velocityInFrame('vision'))
    .setVelocityOfBodyInOdom(body.velocityInFrame('odom'));
  const comms = new robotStatePb.CommsState().setTimestamp(timestamp);
  if (robot.power.wifiRadioOn) {
    comms.setWifiState(
      new robotStatePb.WiFiState()
        .setCurrentMode(robotStatePb.WiFiState.Mode.MODE_ACCESS_POINT)
        .setEssid(robot.config.robot.serialNumber),
    );
  }
  const state = new robotStatePb.RobotState()
    .setPowerState(robot.power.powerStateToProto(timestamp))
    .setBatteryStatesList([robot.battery.toProto(timestamp)])
    .setCommsStatesList([comms])
    .setSystemFaultState(robot.faults.systemFaultStateToProto())
    .setEstopStatesList(robot.estop.estopStatesToProto(timestamp))
    .setKinematicState(kinematic)
    .setBehaviorFaultState(robot.faults.behaviorFaultStateToProto())
    .setFootStateList(body.footStatesToProto())
    .setServiceFaultState(robot.faults.serviceFaultStateToProto())
    .setTerrainState(new robotStatePb.TerrainState().setIsUnsafeToSit(false))
    .setSystemState(robot.thermal.toProto())
    .setBehaviorState(new robotStatePb.BehaviorState().setState(body.behaviorStateValue()));
  if (robot.arm) state.setManipulatorState(robot.arm.manipulatorStateToProto(now));
  return state;
}

/**
 * GetRobotMetrics.
 * @param {import('../robot').Robot} robot
 * @returns {robotStatePb.RobotMetrics}
 */
function buildMetrics(robot) {
  const { Parameter } = parameterPb;
  const now = robot.clock.now();
  const uptime = now - robot.directory.startTime;
  return new robotStatePb.RobotMetrics()
    .setTimestamp(secToTimestamp(now))
    .setMetricsList([
      new Parameter().setLabel('distance').setUnits('m').setFloatValue(robot.body.distanceWalked),
      new Parameter().setLabel('gait cycles').setIntValue(Math.round(robot.body.distanceWalked / 0.6)),
      new Parameter().setLabel('time moving').setDuration(secToDuration(robot.body.distanceWalked / 0.8)),
      new Parameter().setLabel('electric power').setUnits('W').setFloatValue(robot.battery.powerW),
      new Parameter().setLabel('uptime').setDuration(secToDuration(uptime)),
    ]);
}

/**
 * GetRobotStateStream: the joint states, the kinematic state and the contacts, at 50 Hz until the client cancels.
 * @param {robotStatePb.RobotStateStreamRequest} request
 * @param {{robot: import('../robot').Robot, call: any, write: function(any): boolean}} context
 * @returns {Promise<void>}
 */
async function getRobotStateStream(request, { robot, call, write }) {
  const state = { cancelled: false };
  call.on('cancelled', () => {
    state.cancelled = true;
  });
  while (!state.cancelled && robot.online) {
    robot.sync();
    const now = robot.clock.now();
    const timestamp = secToTimestamp(now);
    const joints = [...robot.body.legJointStates(now), ...(robot.arm ? robot.arm.jointStates() : [])];
    const worldTbody = robot.body.bodyPoseWorld();
    const response = new robotStatePb.RobotStateStreamResponse()
      .setJointStates(
        new robotStatePb.CombinedJointStates()
          .setAcquisitionTimestamp(timestamp)
          .setPositionList(joints.map(joint => joint.getPosition().getValue()))
          .setVelocityList(joints.map(joint => joint.getVelocity().getValue()))
          .setLoadList(joints.map(joint => joint.getLoad().getValue())),
      )
      .setKinematicState(
        new robotStatePb.RobotStateStreamResponse.KinematicState()
          .setAcquisitionTimestamp(timestamp)
          .setOdomTformBody(poseToProto(poseMul(poseInv(robot.body.odomInWorld()), worldTbody)))
          .setVisionTformBody(poseToProto(poseMul(poseInv(robot.body.visionInWorld), worldTbody)))
          .setVelocityOfBodyInVision(robot.body.velocityInFrame('vision'))
          .setVelocityOfBodyInOdom(robot.body.velocityInFrame('odom')),
      )
      .setContactStatesList(robot.body.footStatesToProto().map(foot => foot.getContact()));
    // The responses follow each other: the loop waits for the client.
    // eslint-disable-next-line no-await-in-loop
    if (!write(response)) await new Promise(resolve => call.once('drain', resolve));
    // eslint-disable-next-line no-await-in-loop
    await sleep(20);
  }
  // The robot powered off or rebooted: the stream stops, and fails when its connection is reset.
  if (!state.cancelled) {
    await new Promise((resolve, reject) => robot.holdCall(call, err => reject(new GrpcError(err.code, err.details))));
  }
}

module.exports = {
  service: RobotStateServiceService,
  func: {
    getRobotState: unary('GetRobotState', robotStatePb.RobotStateResponse, (request, { robot }) =>
      new robotStatePb.RobotStateResponse().setRobotState(buildRobotState(robot)),
    ),
    getRobotMetrics: unary('GetRobotMetrics', robotStatePb.RobotMetricsResponse, (request, { robot }) =>
      new robotStatePb.RobotMetricsResponse().setRobotMetrics(buildMetrics(robot)),
    ),
    getRobotHardwareConfiguration: unary(
      'GetRobotHardwareConfiguration',
      robotStatePb.RobotHardwareConfigurationResponse,
      (request, { robot }) => {
        const skeleton = new robotStatePb.Skeleton()
          .setLinksList(Object.keys(linksOf(robot)).map(name => new robotStatePb.Skeleton.Link().setName(name)))
          .setUrdf(urdf(robot));
        return new robotStatePb.RobotHardwareConfigurationResponse().setHardwareConfiguration(
          new robotStatePb.HardwareConfiguration()
            .setSkeleton(skeleton)
            .setCanPowerCommandRequestOffRobot(true)
            .setCanPowerCommandRequestCycleRobot(true)
            .setCanPowerCommandRequestPayloadPorts(true)
            .setCanPowerCommandRequestWifiRadio(true)
            .setHasAudioVisualSystem(false)
            .setRedundantSafetyStopEnabled(false),
        );
      },
    ),
    getRobotLinkModel: unary('GetRobotLinkModel', robotStatePb.RobotLinkModelResponse, (request, { robot }) => {
      const size = linksOf(robot)[request.getLinkName()];
      if (!size) throw invalidRequest(`Unknown link "${request.getLinkName()}".`);
      return new robotStatePb.RobotLinkModelResponse().setLinkModel(
        new robotStatePb.Skeleton.Link.ObjModel()
          .setFileName(`${request.getLinkName()}.obj`)
          .setFileContents(boxObj(request.getLinkName(), size)),
      );
    }),
  },
  directory: [
    { name: 'robot-state', type: 'bosdyn.api.RobotStateService', authority: 'state.spot.robot' },
    { name: 'robot-state-streaming', type: 'bosdyn.api.RobotStateStreamingService', authority: 'state.spot.robot' },
  ],
  extraServices: [
    {
      service: RobotStateStreamingServiceService,
      func: { getRobotStateStream: serverStreaming('GetRobotStateStream', getRobotStateStream) },
    },
  ],
  buildRobotState,
};
