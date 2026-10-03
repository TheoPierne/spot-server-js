'use strict';

const { DoubleValue } = require('google-protobuf/google/protobuf/wrappers_pb');

const { secToTimestamp } = require('./clock');
const powerPb = require('../bosdyn/api/power_pb');
const { PowerState } = require('../bosdyn/api/robot_state_pb');
const { LoggerUtil } = require('../loggerUtil');

const logger = LoggerUtil.getLogger('POWER');

const { PowerCommandStatus } = powerPb;
const { Request } = powerPb.PowerCommandRequest;
const Motor = PowerState.MotorPowerState;

const MOTOR_STATE_NAMES = {
  [Motor.MOTOR_POWER_STATE_OFF]: 'OFF',
  [Motor.MOTOR_POWER_STATE_ON]: 'ON',
  [Motor.MOTOR_POWER_STATE_POWERING_ON]: 'POWERING_ON',
  [Motor.MOTOR_POWER_STATE_POWERING_OFF]: 'POWERING_OFF',
  [Motor.MOTOR_POWER_STATE_ERROR]: 'ERROR',
};

// Power commands kept for their feedback.
const MAX_COMMANDS = 100;

/**
 * The power system of the robot: the motor power (powering on takes a few seconds, and is refused while E-Stopped,
 * on shore power, with blocking faults...), the payload ports, the Wi-Fi radio, the fans, and the power off or
 * reboot of the robot computers.
 */
class PowerSystem {
  /**
   * @param {import('../robot')} robot
   */
  constructor(robot) {
    this.robot = robot;
    this.shorePower = false;
    this.payloadPortsOn = true;
    this.wifiRadioOn = true;
    this.nextCommandId = 1 + Math.floor(Math.random() * 1000);
    this.nextFanCommandId = 1 + Math.floor(Math.random() * 1000);
    this.reset();
  }

  /** Motors off (boot of the robot). */
  reset() {
    this.motorState = Motor.MOTOR_POWER_STATE_OFF;
    this.motorErrorMessage = '';
    this.transitionEnd = 0;
    /** @type {Map<number, {id: number, request: number, status: number, motor: boolean}>} */
    this.commands = new Map();
    this.activeMotorCommand = null;
    this.fan = { commands: new Map(), active: null };
  }

  /** @returns {boolean} */
  motorsOn() {
    return this.motorState === Motor.MOTOR_POWER_STATE_ON;
  }

  /** @returns {boolean} */
  motorsOnOrPowering() {
    return this.motorState === Motor.MOTOR_POWER_STATE_ON || this.motorState === Motor.MOTOR_POWER_STATE_POWERING_ON;
  }

  _addCommand(request, status, motor) {
    const id = this.nextCommandId++;
    this.commands.set(id, { id, request, status, motor });
    if (this.commands.size > MAX_COMMANDS) this.commands.delete(this.commands.keys().next().value);
    return this.commands.get(id);
  }

  /**
   * The faults which prevent the motors from powering on.
   * @returns {import('../bosdyn/api/robot_state_pb').SystemFault[]}
   */
  blockingFaults() {
    return this.robot.faults.blockingSystemFaults();
  }

  /**
   * PowerCommand (the lease is checked by the service).
   * @param {number} request
   * @returns {{status: number, id: number, blockingFaults: any[]}}
   */
  command(request) {
    const now = this.robot.clock.now();
    switch (request) {
      case Request.REQUEST_ON_MOTORS:
        return this._powerOnMotors(now);
      case Request.REQUEST_OFF_MOTORS: {
        const command = this._addCommand(request, PowerCommandStatus.STATUS_IN_PROGRESS, true);
        this._overrideMotorCommand();
        if (this.motorState === Motor.MOTOR_POWER_STATE_OFF) {
          command.status = PowerCommandStatus.STATUS_SUCCESS;
        } else {
          this.cutMotorPower('power command REQUEST_OFF_MOTORS');
          this.activeMotorCommand = command.id;
        }
        return { status: command.status, id: command.id, blockingFaults: [] };
      }
      case Request.REQUEST_OFF_ROBOT:
      case Request.REQUEST_CYCLE_ROBOT:
      case Request.REQUEST_SOFT_REBOOT_ROBOT: {
        const command = this._addCommand(request, PowerCommandStatus.STATUS_IN_PROGRESS, false);
        this.cutMotorPower('power off of the robot');
        const reboot = request !== Request.REQUEST_OFF_ROBOT;
        // The robot reports the success, then stops answering while it shuts down (or reboots): the SDKs poll the
        // feedback until the success, or until their RPC times out.
        command.successAt = now + 0.8;
        this.robot.scheduleShutdown({ reboot, delaySec: 3 });
        return { status: command.status, id: command.id, blockingFaults: [] };
      }
      case Request.REQUEST_OFF_PAYLOAD_PORTS:
      case Request.REQUEST_ON_PAYLOAD_PORTS: {
        this.payloadPortsOn = request === Request.REQUEST_ON_PAYLOAD_PORTS;
        logger.info(`Payload ports powered ${this.payloadPortsOn ? 'on' : 'off'}`);
        const command = this._addCommand(request, PowerCommandStatus.STATUS_SUCCESS, false);
        return { status: command.status, id: command.id, blockingFaults: [] };
      }
      case Request.REQUEST_OFF_WIFI_RADIO:
      case Request.REQUEST_ON_WIFI_RADIO: {
        this.wifiRadioOn = request === Request.REQUEST_ON_WIFI_RADIO;
        logger.info(`Wi-Fi radio powered ${this.wifiRadioOn ? 'on' : 'off'}`);
        const command = this._addCommand(request, PowerCommandStatus.STATUS_SUCCESS, false);
        return { status: command.status, id: command.id, blockingFaults: [] };
      }
      default:
        return { status: PowerCommandStatus.STATUS_UNKNOWN, id: 0, blockingFaults: [], invalid: true };
    }
  }

  _powerOnMotors(now) {
    const refuse = (status, blockingFaults = []) => ({ status, id: 0, blockingFaults });
    if (this.robot.estop.isEstopped()) return refuse(PowerCommandStatus.STATUS_ESTOPPED);
    if (this.shorePower) return refuse(PowerCommandStatus.STATUS_SHORE_POWER_CONNECTED);
    const faults = this.blockingFaults();
    if (faults.length > 0) return refuse(PowerCommandStatus.STATUS_FAULTED, faults);
    if (this.robot.keepalive.motorsOffActionActive()) return refuse(PowerCommandStatus.STATUS_KEEPALIVE_MOTORS_OFF);
    if (!this.robot.license.isValid()) return refuse(PowerCommandStatus.STATUS_LICENSE_ERROR);
    if (this.motorState === Motor.MOTOR_POWER_STATE_POWERING_OFF) {
      return refuse(PowerCommandStatus.STATUS_COMMAND_IN_PROGRESS);
    }

    const command = this._addCommand(Request.REQUEST_ON_MOTORS, PowerCommandStatus.STATUS_IN_PROGRESS, true);
    if (this.motorState === Motor.MOTOR_POWER_STATE_ON) {
      command.status = PowerCommandStatus.STATUS_SUCCESS;
      return { status: command.status, id: command.id, blockingFaults: [] };
    }
    this._overrideMotorCommand();
    this.activeMotorCommand = command.id;
    if (this.motorState !== Motor.MOTOR_POWER_STATE_POWERING_ON) {
      this.motorState = Motor.MOTOR_POWER_STATE_POWERING_ON;
      this.transitionEnd = now + this.robot.config.durations.powerOn;
      logger.info('Powering on the motors...');
      this.robot.emit('power:change', this.motorState);
    }
    return { status: command.status, id: command.id, blockingFaults: [] };
  }

  _overrideMotorCommand() {
    const previous = this.commands.get(this.activeMotorCommand);
    if (previous && previous.status === PowerCommandStatus.STATUS_IN_PROGRESS) {
      previous.status = PowerCommandStatus.STATUS_OVERRIDDEN;
    }
    this.activeMotorCommand = null;
  }

  /**
   * PowerCommandFeedback.
   * @param {number} id
   * @returns {?{status: number, blockingFaults: any[]}} Null for an unknown command.
   */
  feedback(id) {
    const command = this.commands.get(id);
    if (!command) return null;
    if (command.successAt !== undefined && this.robot.clock.now() >= command.successAt) {
      command.status = PowerCommandStatus.STATUS_SUCCESS;
    }
    const blockingFaults = command.status === PowerCommandStatus.STATUS_FAULTED ? this.blockingFaults() : [];
    return { status: command.status, blockingFaults };
  }

  /**
   * Cuts the motor power immediately: a standing robot collapses.
   * @param {string} reason
   */
  cutMotorPower(reason) {
    if (this.motorState === Motor.MOTOR_POWER_STATE_OFF || this.motorState === Motor.MOTOR_POWER_STATE_POWERING_OFF) {
      return;
    }
    const wasPoweringOn = this.motorState === Motor.MOTOR_POWER_STATE_POWERING_ON;
    logger.info(`Motor power cut (${reason})`);
    const active = this.commands.get(this.activeMotorCommand);
    if (
      active &&
      active.request === Request.REQUEST_ON_MOTORS &&
      active.status === PowerCommandStatus.STATUS_IN_PROGRESS
    ) {
      active.status = this.robot.estop.isEstopped()
        ? PowerCommandStatus.STATUS_ESTOPPED
        : PowerCommandStatus.STATUS_OVERRIDDEN;
      this.activeMotorCommand = null;
    }
    this.motorState = Motor.MOTOR_POWER_STATE_POWERING_OFF;
    this.transitionEnd = this.robot.clock.now() + this.robot.config.durations.powerOff;
    if (!wasPoweringOn) this.robot.body.onMotorPowerCut();
    this.robot.commands.onMotorPowerLost();
    this.robot.emit('power:change', this.motorState);
  }

  /**
   * Sits the robot down, then cuts the motor power (E-Stop SETTLE_THEN_CUT, low battery, keepalive).
   * @param {string} reason
   */
  settleThenCut(reason) {
    if (!this.motorsOnOrPowering()) return;
    if (this.motorState === Motor.MOTOR_POWER_STATE_POWERING_ON || this.robot.body.isResting()) {
      this.cutMotorPower(reason);
      return;
    }
    this.robot.commands.startInternalSafePowerOff(reason);
  }

  /**
   * @param {number} now
   */
  update(now) {
    if (this.motorState === Motor.MOTOR_POWER_STATE_POWERING_ON && now >= this.transitionEnd) {
      this.motorState = Motor.MOTOR_POWER_STATE_ON;
      this._finishMotorCommand(Request.REQUEST_ON_MOTORS);
      logger.info('Motors powered on');
      this.robot.emit('power:change', this.motorState);
    } else if (this.motorState === Motor.MOTOR_POWER_STATE_POWERING_OFF && now >= this.transitionEnd) {
      this.motorState = Motor.MOTOR_POWER_STATE_OFF;
      this._finishMotorCommand(Request.REQUEST_OFF_MOTORS);
      logger.info('Motors powered off');
      this.robot.emit('power:change', this.motorState);
    }
    this._updateFans(now);
  }

  _finishMotorCommand(request) {
    const active = this.commands.get(this.activeMotorCommand);
    if (active && active.request === request && active.status === PowerCommandStatus.STATUS_IN_PROGRESS) {
      active.status = PowerCommandStatus.STATUS_SUCCESS;
    }
    this.activeMotorCommand = null;
  }

  /**
   * FanPowerCommand.
   * @param {number} percent
   * @param {number} durationSec
   * @returns {{status: number, id: number, endTime: number}}
   */
  fanCommand(percent, durationSec) {
    const now = this.robot.clock.now();
    const Status = powerPb.FanPowerCommandResponse.Status;
    if (this.robot.thermal.maxMotorTemperature() > 75 && percent < 100) {
      return { status: Status.STATUS_TEMPERATURE_TOO_HIGH, id: 0, endTime: now };
    }
    const previous = this.fan.commands.get(this.fan.active);
    if (previous && previous.status === powerPb.FanPowerCommandFeedbackResponse.Status.STATUS_RUNNING) {
      previous.status = powerPb.FanPowerCommandFeedbackResponse.Status.STATUS_OVERRIDDEN_BY_COMMAND;
      previous.earlyStop = now;
    }
    const id = this.nextFanCommandId++;
    const command = {
      id,
      percent: Math.max(0, Math.min(100, percent)),
      endTime: now + durationSec,
      status: powerPb.FanPowerCommandFeedbackResponse.Status.STATUS_RUNNING,
      earlyStop: null,
    };
    this.fan.commands.set(id, command);
    this.fan.active = id;
    return { status: Status.STATUS_OK, id, endTime: command.endTime };
  }

  /**
   * @param {number} id
   * @returns {?powerPb.FanPowerCommandFeedbackResponse} Without header, null for an unknown command.
   */
  fanFeedback(id) {
    const command = this.fan.commands.get(id);
    if (!command) return null;
    const response = new powerPb.FanPowerCommandFeedbackResponse()
      .setStatus(command.status)
      .setDesiredEndTime(secToTimestamp(command.endTime));
    if (command.earlyStop !== null) response.setEarlyStopTime(secToTimestamp(command.earlyStop));
    return response;
  }

  _updateFans(now) {
    const active = this.fan.commands.get(this.fan.active);
    if (
      active &&
      active.status === powerPb.FanPowerCommandFeedbackResponse.Status.STATUS_RUNNING &&
      now >= active.endTime
    ) {
      active.status = powerPb.FanPowerCommandFeedbackResponse.Status.STATUS_COMPLETE;
      this.fan.active = null;
    }
  }

  /**
   * The speed of the fans, in Hz: commanded, or following the temperature of the motors.
   * @returns {Object<string, number>}
   */
  fanFrequencies() {
    const active = this.fan.commands.get(this.fan.active);
    const automatic = Math.max(0, Math.min(1, (this.robot.thermal.maxMotorTemperature() - 30) / 40));
    const fraction = active ? active.percent / 100 : automatic;
    const frequency = 40 + fraction * 160;
    return { body_fan_0: frequency, body_fan_1: frequency * 0.98, hips_fan_0: frequency * 1.02, hips_fan_1: frequency };
  }

  /**
   * @param {import('google-protobuf/google/protobuf/timestamp_pb').Timestamp} timestamp
   * @returns {PowerState}
   */
  powerStateToProto(timestamp) {
    const battery = this.robot.battery;
    return new PowerState()
      .setTimestamp(timestamp)
      .setMotorPowerState(this.motorState)
      .setMotorPowerErrorMessage(this.motorErrorMessage)
      .setShorePowerState(
        this.shorePower
          ? PowerState.ShorePowerState.SHORE_POWER_STATE_ON
          : PowerState.ShorePowerState.SHORE_POWER_STATE_OFF,
      )
      .setRobotPowerState(PowerState.RobotPowerState.ROBOT_POWER_STATE_ON)
      .setPayloadPortsPowerState(
        this.payloadPortsOn
          ? PowerState.PayloadPortsPowerState.PAYLOAD_PORTS_POWER_STATE_ON
          : PowerState.PayloadPortsPowerState.PAYLOAD_PORTS_POWER_STATE_OFF,
      )
      .setWifiRadioPowerState(
        this.wifiRadioOn
          ? PowerState.WifiRadioPowerState.WIFI_RADIO_POWER_STATE_ON
          : PowerState.WifiRadioPowerState.WIFI_RADIO_POWER_STATE_OFF,
      )
      .setLocomotionChargePercentage(new DoubleValue().setValue(battery.percent()))
      .setLocomotionEstimatedRuntime(battery.estimatedRuntimeProto());
  }

  /**
   * A summary for the console.
   * @returns {string}
   */
  describe() {
    const parts = [`motors ${MOTOR_STATE_NAMES[this.motorState]}`];
    if (this.shorePower) parts.push('shore power connected');
    if (!this.payloadPortsOn) parts.push('payload ports off');
    if (!this.wifiRadioOn) parts.push('Wi-Fi radio off');
    return parts.join(', ');
  }
}

module.exports = { MOTOR_STATE_NAMES, PowerSystem };
