'use strict';

const { secToTimestamp, timestampToSec } = require('./clock');
const choreographyPb = require('../bosdyn/api/spot/choreography_sequence_pb');
const { LoggerUtil } = require('../loggerUtil');

const logger = LoggerUtil.getLogger('CHOREOGRAPHY');

// Moves of the choreography service, with their default length in slices.
const MOVES = {
  body_hold: 4,
  bourree: 4,
  butt_circle: 4,
  chicken_head: 8,
  clap: 4,
  crawl: 4,
  frontup: 2,
  goto: 4,
  hop: 2,
  jump: 4,
  kneel_circles: 4,
  kneel_leg_move: 4,
  pace_2step: 4,
  random_rotate: 4,
  rotate_body: 4,
  running_man: 4,
  sit: 4,
  sit_to_stand: 4,
  stand_to_sit: 4,
  step: 2,
  sway: 4,
  turn: 4,
  twerk: 4,
};

/**
 * The choreography service, simplified: the sequences are uploaded and listed, and executing one makes the robot
 * "dance" (body motions) for the length of the sequence, at its slices per minute.
 */
class Choreography {
  /**
   * @param {import('../robot')} robot
   */
  constructor(robot) {
    this.robot = robot;
    /** @type {Map<string, {sequence: choreographyPb.ChoreographySequence, savedState: number}>} */
    this.sequences = new Map();
    this.animations = new Set();
    this.nextExecutionId = 1;
    this.execution = null;
  }

  /** @returns {choreographyPb.MoveInfo[]} */
  movesToProto() {
    return Object.entries(MOVES).map(([name, slices]) =>
      new choreographyPb.MoveInfo()
        .setName(name)
        .setMoveLengthSlices(slices)
        .setMinMoveLengthSlices(1)
        .setMaxMoveLengthSlices(slices * 8)
        .setIsExtendable(true)
        .setControlsLegs(true)
        .setControlsBody(true),
    );
  }

  /**
   * UploadChoreography.
   * @param {choreographyPb.ChoreographySequence} sequence
   * @param {boolean} nonStrict Unknown moves are warnings instead of errors.
   * @returns {{warnings: string[], error: ?string}}
   */
  upload(sequence, nonStrict) {
    const warnings = [];
    if (!sequence?.getName()) return { warnings, error: 'The sequence has no name.' };
    if (!(sequence.getSlicesPerMinute() > 0)) return { warnings, error: 'The slices per minute must be positive.' };
    for (const move of sequence.getMovesList()) {
      const known = move.getType() in MOVES || this.animations.has(move.getType());
      if (!known) {
        const message = `Unknown move type "${move.getType()}".`;
        if (!nonStrict) return { warnings, error: message };
        warnings.push(message);
      }
      if (move.getRequestedSlices() <= 0) warnings.push(`The move "${move.getType()}" has no slices.`);
    }
    this.sequences.set(sequence.getName(), {
      sequence: sequence.clone(),
      savedState: choreographyPb.SequenceInfo.SavedState.SAVED_STATE_TEMPORARY,
    });
    logger.info(`Sequence "${sequence.getName()}" uploaded (${sequence.getMovesList().length} moves)`);
    return { warnings, error: null };
  }

  /** @returns {choreographyPb.SequenceInfo[]} */
  sequencesToProto() {
    return [...this.sequences.entries()].map(([name, entry]) =>
      new choreographyPb.SequenceInfo()
        .setName(name)
        .setLabelsList(entry.sequence.getChoreographyInfo()?.getLabelsList() ?? [])
        .setSavedState(entry.savedState),
    );
  }

  /**
   * @param {choreographyPb.ChoreographySequence} sequence
   * @returns {number} The number of slices of a sequence.
   */
  static slicesOf(sequence) {
    let end = 0;
    for (const move of sequence.getMovesList()) end = Math.max(end, move.getStartSlice() + move.getRequestedSlices());
    return end;
  }

  /**
   * ExecuteChoreography (the lease is checked by the service).
   * @param {string} name
   * @param {?number} startTime Robot time.
   * @param {number} startSlice
   * @returns {{status: number, id: number}}
   */
  execute(name, startTime, startSlice) {
    const Status = choreographyPb.ExecuteChoreographyResponse.Status;
    const entry = this.sequences.get(name);
    if (!entry) return { status: Status.STATUS_UNKNOWN_SEQUENCE, id: 0 };
    if (!this.robot.power.motorsOn() || this.robot.faults.hasBehaviorFaults()) {
      return { status: Status.STATUS_ROBOT_COMMAND_ISSUES, id: 0 };
    }
    const slices = Choreography.slicesOf(entry.sequence);
    const id = this.nextExecutionId++;
    const now = this.robot.clock.now();
    this.execution = {
      id,
      name,
      start: startTime ?? now,
      startSlice,
      slices,
      slicesPerMinute: entry.sequence.getSlicesPerMinute(),
      status: choreographyPb.ChoreographyStatusResponse.Status.STATUS_WAITING_FOR_START_TIME,
    };
    this.robot.docking.onMobilityOverridden();
    this.robot.body.start({ kind: 'stand', owner: 'choreography', bodyControl: null });
    logger.info(`Executing "${name}" (${slices} slices at ${entry.sequence.getSlicesPerMinute()} slices/min)`);
    return { status: Status.STATUS_OK, id };
  }

  /**
   * @param {number} now
   */
  update(now) {
    const execution = this.execution;
    const { Status } = choreographyPb.ChoreographyStatusResponse;
    if (!execution || ![Status.STATUS_WAITING_FOR_START_TIME, Status.STATUS_DANCING].includes(execution.status)) return;
    const { body, power } = this.robot;
    if (body.behavior.owner !== 'choreography') {
      execution.status = Status.STATUS_INTERRUPTED;
      return;
    }
    if (!power.motorsOn()) {
      execution.status = Status.STATUS_POWERED_OFF;
      return;
    }
    if (body.isFallen()) {
      execution.status = Status.STATUS_FALLEN;
      return;
    }
    const slice = this.currentSlice(now);
    if (now < execution.start) return;
    execution.status = Status.STATUS_DANCING;
    // Body motions on the beat.
    const phase = (slice * Math.PI) / 2;
    body.offsetTarget = {
      x: 0.03 * Math.sin(phase),
      y: 0,
      height: -0.05 + 0.05 * Math.sin(phase * 2),
      yaw: 0.15 * Math.sin(phase),
      roll: 0.1 * Math.cos(phase),
      pitch: 0,
    };
    if (slice >= execution.slices) {
      execution.status = Status.STATUS_COMPLETED_SEQUENCE;
      body.offsetTarget = { x: 0, y: 0, height: 0, yaw: 0, roll: 0, pitch: 0 };
      logger.info(`Sequence "${execution.name}" completed`);
    }
  }

  /**
   * @param {number} now
   * @returns {number}
   */
  currentSlice(now) {
    const execution = this.execution;
    if (!execution) return 0;
    return execution.startSlice + Math.max(0, ((now - execution.start) * execution.slicesPerMinute) / 60);
  }

  /**
   * @returns {choreographyPb.ChoreographyStatusResponse} Without header.
   */
  statusToProto() {
    const response = new choreographyPb.ChoreographyStatusResponse();
    const execution = this.execution;
    if (!execution) return response.setStatus(choreographyPb.ChoreographyStatusResponse.Status.STATUS_OTHER);
    return response
      .setStatus(execution.status)
      .setExecutionId(execution.id)
      .setCurrentSlice(Math.min(execution.slices, this.currentSlice(this.robot.clock.now())))
      .setSequenceSlices(execution.slices)
      .setSequenceSlicesPerMinute(execution.slicesPerMinute)
      .setSequenceName(execution.name)
      .setSequenceStartTime(secToTimestamp(execution.start));
  }

  /**
   * @param {?import('google-protobuf/google/protobuf/timestamp_pb').Timestamp} timestamp
   * @returns {?number}
   */
  static timeOf(timestamp) {
    return timestamp && (timestamp.getSeconds() || timestamp.getNanos()) ? timestampToSec(timestamp) : null;
  }
}

module.exports = { Choreography, MOVES };
