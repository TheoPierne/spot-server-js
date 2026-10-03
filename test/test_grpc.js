'use strict';

const assert = require('node:assert/strict');
const test = require('node:test');
const { setTimeout } = require('node:timers');

const grpc = require('@grpc/grpc-js');

const authPb = require('../src/bosdyn/api/auth_pb');
const { AuthServiceClient } = require('../src/bosdyn/api/auth_service_grpc_pb');
const directoryPb = require('../src/bosdyn/api/directory_pb');
const { DirectoryServiceClient } = require('../src/bosdyn/api/directory_service_grpc_pb');
const { CommonError, RequestHeader } = require('../src/bosdyn/api/header_pb');
const imagePb = require('../src/bosdyn/api/image_pb');
const { ImageServiceClient } = require('../src/bosdyn/api/image_service_grpc_pb');
const robotIdPb = require('../src/bosdyn/api/robot_id_pb');
const { RobotIdServiceClient } = require('../src/bosdyn/api/robot_id_service_grpc_pb');
const robotStatePb = require('../src/bosdyn/api/robot_state_pb');
const { RobotStateServiceClient } = require('../src/bosdyn/api/robot_state_service_grpc_pb');
const timeSyncPb = require('../src/bosdyn/api/time_sync_pb');
const { TimeSyncServiceClient } = require('../src/bosdyn/api/time_sync_service_grpc_pb');
const { startServer } = require('../src/server');
const { nsecToTimestamp } = require('../src/sim/clock');

/**
 * Starts the simulator (insecure, free port of 127.0.0.1) and creates clients.
 * @param {object} [config]
 * @returns {Promise<{call: function(string, string, any, ?string=): Promise<any>, stop: function(): Promise<void>,
 *   robot: import('../src/robot').Robot}>}
 */
async function simulator(config = {}) {
  const { port, robot, stop } = await startServer({
    host: '127.0.0.1',
    port: 0,
    insecure: true,
    persist: false,
    config,
  });
  const address = `127.0.0.1:${port}`;
  const credentials = grpc.credentials.createInsecure();
  const clients = {
    auth: new AuthServiceClient(address, credentials),
    directory: new DirectoryServiceClient(address, credentials),
    image: new ImageServiceClient(address, credentials),
    robotId: new RobotIdServiceClient(address, credentials),
    robotState: new RobotStateServiceClient(address, credentials),
    timeSync: new TimeSyncServiceClient(address, credentials),
  };
  const call = (client, method, request, token = null, options = {}) =>
    new Promise((resolve, reject) => {
      const metadata = new grpc.Metadata();
      if (token) metadata.set('authorization', `Bearer ${token}`);
      request.setHeader(
        new RequestHeader()
          .setClientName('test-grpc')
          .setRequestTimestamp(nsecToTimestamp(BigInt(Date.now()) * 1000000n)),
      );
      clients[client][method](request, metadata, options, (err, response) => (err ? reject(err) : resolve(response)));
    });
  return {
    call,
    robot,
    stop: async () => {
      for (const client of Object.values(clients)) client.close();
      await stop();
    },
  };
}

test('user tokens: robot-id and auth without a token, the other services need a valid one', async () => {
  const { call, stop } = await simulator();
  try {
    const id = await call('robotId', 'getRobotId', new robotIdPb.RobotIdRequest());
    assert.equal(id.getRobotId().getSpecies(), 'spot');
    assert.equal(id.getHeader().getError().getCode(), CommonError.Code.CODE_OK);
    // The header echoes the request and has the robot times (the time sync of the clients needs them).
    assert.equal(id.getHeader().getRequest().getTypeName(), 'bosdyn.api.RobotIdRequest');
    assert.ok(id.getHeader().hasRequestReceivedTimestamp() && id.getHeader().hasResponseTimestamp());

    await assert.rejects(call('robotState', 'getRobotState', new robotStatePb.RobotStateRequest()), {
      code: grpc.status.UNAUTHENTICATED,
    });
    await assert.rejects(call('robotState', 'getRobotState', new robotStatePb.RobotStateRequest(), 'a.b.c'), {
      code: grpc.status.UNAUTHENTICATED,
    });
    const auth = await call(
      'auth',
      'getAuthToken',
      new authPb.GetAuthTokenRequest().setUsername('user').setPassword('password'),
    );
    assert.equal(auth.getStatus(), authPb.GetAuthTokenResponse.Status.STATUS_OK);
    const state = await call('robotState', 'getRobotState', new robotStatePb.RobotStateRequest(), auth.getToken());
    assert.equal(
      state.getRobotState().getPowerState().getMotorPowerState(),
      robotStatePb.PowerState.MotorPowerState.MOTOR_POWER_STATE_OFF,
    );
    // Refresh with the token.
    const refreshed = await call('auth', 'getAuthToken', new authPb.GetAuthTokenRequest().setToken(auth.getToken()));
    assert.equal(refreshed.getStatus(), authPb.GetAuthTokenResponse.Status.STATUS_OK);
  } finally {
    await stop();
  }
});

test('accounts: invalid login, lockout after six failures', async () => {
  const { call, stop } = await simulator({ auth: { users: [{ username: 'admin', password: 'secret' }] } });
  try {
    const login = password =>
      call('auth', 'getAuthToken', new authPb.GetAuthTokenRequest().setUsername('admin').setPassword(password)).then(
        r => r.getStatus(),
      );
    const { Status } = authPb.GetAuthTokenResponse;
    assert.equal(await login('secret'), Status.STATUS_OK);
    for (let i = 0; i < 6; i++) {
      // eslint-disable-next-line no-await-in-loop
      assert.equal(await login('wrong'), Status.STATUS_INVALID_LOGIN);
    }
    assert.equal(await login('secret'), Status.STATUS_TEMPORARILY_LOCKED_OUT);
  } finally {
    await stop();
  }
});

test('time sync: an identifier, more samples needed, then an estimate', async () => {
  const { call, stop, robot } = await simulator();
  try {
    const token = robot.auth.issue('user');
    let response = await call('timeSync', 'timeSyncUpdate', new timeSyncPb.TimeSyncUpdateRequest(), token);
    const id = response.getClockIdentifier();
    assert.ok(id);
    assert.equal(response.getState().getStatus(), timeSyncPb.TimeSyncState.Status.STATUS_MORE_SAMPLES_NEEDED);
    for (let i = 0; i < 3; i++) {
      const header = response.getHeader();
      const roundTrip = new timeSyncPb.TimeSyncRoundTrip()
        .setClientTx(header.getRequestHeader().getRequestTimestamp())
        .setServerRx(header.getRequestReceivedTimestamp())
        .setServerTx(header.getResponseTimestamp())
        .setClientRx(nsecToTimestamp(BigInt(Date.now()) * 1000000n));
      // eslint-disable-next-line no-await-in-loop
      response = await call(
        'timeSync',
        'timeSyncUpdate',
        new timeSyncPb.TimeSyncUpdateRequest().setClockIdentifier(id).setPreviousRoundTrip(roundTrip),
        token,
      );
    }
    assert.equal(response.getState().getStatus(), timeSyncPb.TimeSyncState.Status.STATUS_OK);
    // Same clock on both sides: the skew is the measurement noise.
    const skew = response.getState().getBestEstimate().getClockSkew();
    assert.ok(Math.abs(skew.getSeconds() + skew.getNanos() / 1e9) < 0.05);
    assert.equal(robot.timeSync.isSynced(id), true);
  } finally {
    await stop();
  }
});

test('directory and images', async () => {
  const { call, stop, robot } = await simulator();
  try {
    const token = robot.auth.issue('user');
    const list = await call('directory', 'listServiceEntries', new directoryPb.ListServiceEntriesRequest(), token);
    const entries = Object.fromEntries(list.getServiceEntriesList().map(entry => [entry.getName(), entry]));
    assert.equal(entries.auth.getUserTokenRequired(), false);
    assert.equal(entries['robot-command'].getType(), 'bosdyn.api.RobotCommandService');
    const missing = await call(
      'directory',
      'getServiceEntry',
      new directoryPb.GetServiceEntryRequest().setServiceName('nope'),
      token,
    );
    assert.equal(missing.getStatus(), directoryPb.GetServiceEntryResponse.Status.STATUS_NONEXISTENT_SERVICE);

    const request = new imagePb.GetImageRequest().setImageRequestsList([
      new imagePb.ImageRequest().setImageSourceName('frontleft_fisheye_image').setQualityPercent(50),
      new imagePb.ImageRequest().setImageSourceName('frontleft_depth'),
      new imagePb.ImageRequest().setImageSourceName('frontleft_depth').setImageFormat(imagePb.Image.Format.FORMAT_JPEG),
      new imagePb.ImageRequest().setImageSourceName('nope'),
    ]);
    const images = (await call('image', 'getImage', request, token)).getImageResponsesList();
    const { Status } = imagePb.ImageResponse;
    assert.deepEqual(
      images.map(image => image.getStatus()),
      [
        Status.STATUS_OK,
        Status.STATUS_OK,
        Status.STATUS_UNSUPPORTED_IMAGE_FORMAT_REQUESTED,
        Status.STATUS_UNKNOWN_CAMERA,
      ],
    );
    const jpeg = images[0].getShot().getImage().getData_asU8();
    assert.deepEqual([jpeg[0], jpeg[1]], [0xff, 0xd8]);
    const depth = images[1].getShot().getImage();
    assert.equal(depth.getData_asU8().length, depth.getCols() * depth.getRows() * 2);
    assert.equal(depth.getPixelFormat(), imagePb.Image.PixelFormat.PIXEL_FORMAT_DEPTH_U16);
  } finally {
    await stop();
  }
});

test('a robot which reboots does not answer: the calls time out, then their connection is reset', async () => {
  const { call, stop, robot } = await simulator({ durations: { reboot: 0.6 } });
  try {
    const token = robot.auth.issue('user');
    robot.scheduleShutdown({ reboot: true, delaySec: 0 });
    await new Promise(resolve => setTimeout(resolve, 50));
    // Like an unreachable robot: no answer before the deadline (the SDKs expect a timeout after a power off).
    await assert.rejects(
      call('robotId', 'getRobotId', new robotIdPb.RobotIdRequest(), null, { deadline: Date.now() + 200 }),
      { code: grpc.status.DEADLINE_EXCEEDED },
    );
    // Without deadline: the connection is reset when the robot boots.
    await assert.rejects(call('robotState', 'getRobotState', new robotStatePb.RobotStateRequest(), token), {
      code: grpc.status.UNAVAILABLE,
    });
    // Booted: it answers, and the tokens of before the reboot are still valid.
    const state = await call('robotState', 'getRobotState', new robotStatePb.RobotStateRequest(), token);
    assert.ok(state.hasRobotState());
  } finally {
    await stop();
  }
});
