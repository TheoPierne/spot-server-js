'use strict';

const { secToTimestamp, timestampToSec } = require('./clock');
const missionPb = require('../bosdyn/api/mission/mission_pb');
const nodesPb = require('../bosdyn/api/mission/nodes_pb');
const { LoggerUtil } = require('../loggerUtil');

const logger = LoggerUtil.getLogger('MISSION');

const { State } = missionPb;

/**
 * The mission service, simplified: a loaded mission runs while it is played (PlayMission must be repeated before its
 * pause time, like on a real robot), and succeeds after a time that grows with its number of nodes. The nodes are
 * not executed.
 */
class MissionSystem {
  /**
   * @param {import('../robot')} robot
   */
  constructor(robot) {
    this.robot = robot;
    this.nextMissionId = 1;
    this.reset();
  }

  /** No mission (boot of the robot). */
  reset() {
    this.mission = null;
    this.status = State.Status.STATUS_NONE;
    this.tickCounter = 0;
    this.runTime = 0;
    this.pauseTime = null;
    this.history = [];
  }

  /**
   * LoadMission.
   * @param {import('../bosdyn/api/mission/nodes_pb').Node} root
   * @returns {missionPb.MissionInfo}
   */
  load(root) {
    let nextNodeId = 1;
    let count = 0;
    const info = node => {
      count += 1;
      const nodeInfo = new missionPb.NodeInfo().setId(nextNodeId++).setName(node.getName());
      if (node.hasUserData()) nodeInfo.setUserData(node.getUserData());
      const impl = node.getImpl();
      // The children of the composite nodes (Sequence, Selector...), from their implementation.
      for (const child of MissionSystem._children(impl)) nodeInfo.addChildren(info(child));
      return nodeInfo;
    };
    const missionInfo = new missionPb.MissionInfo().setId(this.nextMissionId++).setRoot(info(root));
    this.reset();
    this.mission = { root: root.clone(), info: missionInfo, duration: Math.max(3, count) };
    // Loaded, not playing yet.
    this.status = State.Status.STATUS_PAUSED;
    logger.info(`Mission ${missionInfo.getId()} loaded (${count} nodes)`);
    return missionInfo;
  }

  /**
   * The child nodes of the implementation of a node (an Any).
   * @param {?import('google-protobuf/google/protobuf/any_pb').Any} impl
   * @returns {import('../bosdyn/api/mission/nodes_pb').Node[]}
   */
  static _children(impl) {
    if (!impl || !impl.getTypeUrl()) return [];
    const typeName = impl.getTypeName();
    const type = typeName.split('.').pop();
    const message = nodesPb[type];
    if (!message) return [];
    let unpacked;
    try {
      unpacked = impl.unpack(message.deserializeBinary, typeName);
    } catch {
      return [];
    }
    const children = [];
    if (typeof unpacked?.getChildrenList === 'function') children.push(...unpacked.getChildrenList());
    if (typeof unpacked?.getChild === 'function' && unpacked.getChild()) children.push(unpacked.getChild());
    return children;
  }

  /**
   * PlayMission / RestartMission.
   * @param {?import('google-protobuf/google/protobuf/timestamp_pb').Timestamp} pauseTime
   * @param {boolean} restart
   * @returns {boolean} False without mission.
   */
  play(pauseTime, restart = false) {
    if (!this.mission) return false;
    if (restart) {
      this.runTime = 0;
      this.tickCounter = 0;
      this.history = [];
    }
    this.pauseTime = pauseTime ? timestampToSec(pauseTime) : null;
    const finished = [State.Status.STATUS_SUCCESS, State.Status.STATUS_FAILURE, State.Status.STATUS_STOPPED].includes(
      this.status,
    );
    if (!finished || restart) {
      if (this.status !== State.Status.STATUS_RUNNING) logger.info(`Mission ${this.mission.info.getId()} running`);
      this.status = State.Status.STATUS_RUNNING;
    }
    return true;
  }

  /** @returns {boolean} False if no mission is playing. */
  pause() {
    if (this.status !== State.Status.STATUS_RUNNING) return false;
    this.status = State.Status.STATUS_PAUSED;
    return true;
  }

  /** @returns {boolean} False if no mission is playing. */
  stop() {
    if (!this.mission || ![State.Status.STATUS_RUNNING, State.Status.STATUS_PAUSED].includes(this.status)) return false;
    this.status = State.Status.STATUS_STOPPED;
    logger.info('Mission stopped');
    return true;
  }

  /**
   * @param {number} now
   * @param {number} dt
   */
  update(now, dt) {
    if (this.status !== State.Status.STATUS_RUNNING) return;
    if (this.pauseTime !== null && now > this.pauseTime) {
      this.status = State.Status.STATUS_PAUSED;
      logger.info('Mission paused (its pause time passed)');
      return;
    }
    this.runTime += dt;
    const ticks = Math.floor(this.runTime * 10);
    while (this.tickCounter < ticks) {
      this.tickCounter += 1;
      this.history.push({ tick: this.tickCounter, time: now });
      if (this.history.length > 100) this.history.shift();
    }
    if (this.runTime >= this.mission.duration) {
      this.status = State.Status.STATUS_SUCCESS;
      logger.info(`Mission ${this.mission.info.getId()} succeeded`);
    }
  }

  /**
   * @returns {State}
   */
  stateToProto() {
    const state = new State().setStatus(this.status).setTickCounter(this.tickCounter);
    if (this.mission) state.setMissionId(this.mission.info.getId());
    state.setHistoryList(
      this.history
        .slice(-10)
        .map(entry =>
          new State.NodeStatesAtTick().setTickCounter(entry.tick).setTickStartTimestamp(secToTimestamp(entry.time)),
        ),
    );
    return state;
  }
}

module.exports = { MissionSystem };
