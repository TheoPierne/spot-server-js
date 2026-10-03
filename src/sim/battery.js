'use strict';

const { DoubleValue } = require('google-protobuf/google/protobuf/wrappers_pb');

const { secToDuration } = require('./clock');
const robotStatePb = require('../bosdyn/api/robot_state_pb');
const { LoggerUtil } = require('../loggerUtil');

const logger = LoggerUtil.getLogger('BATTERY');

const { BatteryState, SystemFault } = robotStatePb;

// Voltage of the battery, empty and full (V).
const VOLTAGE_EMPTY = 46;
const VOLTAGE_FULL = 58.8;

/**
 * The battery: it discharges with the power drawn by the robot (more standing, more walking) and charges on the dock.
 * Below a low charge the robot reports a fault; below a critical charge, it sits down and powers off its motors, and
 * the motors cannot be powered on.
 */
class Battery {
  /**
   * @param {import('../robot')} robot
   */
  constructor(robot) {
    this.robot = robot;
    this.config = robot.config.battery;
    this.energyWh = (this.config.capacityWh * this.config.initialPercent) / 100;
    this.powerW = this.config.idlePowerW;
    this.charging = false;
    this.temperature = 28;
    this.missing = false;
  }

  /** @returns {number} The charge, in percent. */
  percent() {
    return Math.max(0, Math.min(100, (100 * this.energyWh) / this.config.capacityWh));
  }

  /**
   * Sets the charge (console of the simulator).
   * @param {number} percent
   */
  setPercent(percent) {
    this.energyWh = (this.config.capacityWh * Math.max(0, Math.min(100, percent))) / 100;
    this._updateFaults();
  }

  /** @returns {number} The voltage under the current load (V). */
  voltage() {
    const open = VOLTAGE_EMPTY + ((VOLTAGE_FULL - VOLTAGE_EMPTY) * this.percent()) / 100;
    // Internal resistance of about 0.05 ohm.
    return open + (this.charging ? 1 : -1) * 0.05 * (Math.abs(this._netPowerW()) / open);
  }

  /** @returns {number} Net power into the battery (W): positive when charging. */
  _netPowerW() {
    return this.charging ? this.config.chargePowerW - this.powerW : -this.powerW;
  }

  /** @returns {number} Current into the battery (A): positive when charging, negative when discharging. */
  current() {
    return this._netPowerW() / this.voltage();
  }

  /** @returns {number} Estimated remaining runtime at the current power (s). */
  estimatedRuntime() {
    if (this.charging) return (this.energyWh / this.config.walkingPowerW) * 3600;
    return (this.energyWh / Math.max(this.powerW, 1)) * 3600;
  }

  estimatedRuntimeProto() {
    return secToDuration(Math.round(this.estimatedRuntime()));
  }

  /**
   * @param {number} dt
   */
  update(dt) {
    const body = this.robot.body;
    if (!this.robot.power.motorsOnOrPowering()) {
      this.powerW = this.config.idlePowerW;
    } else if (body.isWalking()) {
      const speed = Math.min(1, Math.hypot(body.velocity.x, body.velocity.y) / this.robot.config.mobility.maxVelX);
      this.powerW =
        this.config.standingPowerW + (this.config.walkingPowerW - this.config.standingPowerW) * (0.4 + 0.6 * speed);
    } else {
      this.powerW = body.isResting() ? this.config.idlePowerW + 60 : this.config.standingPowerW;
    }
    this.charging = this.robot.docking.isCharging() && this.percent() < 100;
    const energy = (this._netPowerW() * dt * this.config.timeScale) / 3600;
    this.energyWh = Math.max(0, Math.min(this.config.capacityWh, this.energyWh + energy));
    // The cells warm up with the current.
    const target = 28 + Math.abs(this._netPowerW()) / 40;
    this.temperature += ((target - this.temperature) * dt) / 600;
    this._updateFaults();
  }

  _updateFaults() {
    const { faults } = this.robot;
    const percent = this.percent();
    if (percent <= this.config.lowPercent) {
      faults.addSystemFault('battery_low', {
        message: `Battery charge is low (${percent.toFixed(0)} %).`,
        severity: SystemFault.Severity.SEVERITY_WARN,
        attributes: ['battery'],
      });
    } else {
      faults.clearSystemFault('battery_low');
    }
    if (percent <= this.config.criticalPercent) {
      if (!faults.systemFaults.has('battery_critical')) {
        logger.warn('Battery critically low: the robot sits down and powers off its motors');
      }
      faults.addSystemFault('battery_critical', {
        message: 'Battery charge is critically low. Charge or replace the battery.',
        severity: SystemFault.Severity.SEVERITY_CRITICAL,
        attributes: ['battery'],
        blocking: true,
      });
      this.robot.power.settleThenCut('critically low battery');
    } else {
      faults.clearSystemFault('battery_critical');
    }
  }

  /**
   * @param {import('google-protobuf/google/protobuf/timestamp_pb').Timestamp} timestamp
   * @returns {BatteryState}
   */
  toProto(timestamp) {
    let status = this.charging ? BatteryState.Status.STATUS_CHARGING : BatteryState.Status.STATUS_DISCHARGING;
    if (this.missing) status = BatteryState.Status.STATUS_MISSING;
    return new BatteryState()
      .setTimestamp(timestamp)
      .setIdentifier('battery-SN-23110042')
      .setChargePercentage(new DoubleValue().setValue(this.percent()))
      .setEstimatedRuntime(this.estimatedRuntimeProto())
      .setCurrent(new DoubleValue().setValue(this.current()))
      .setVoltage(new DoubleValue().setValue(this.voltage()))
      .setTemperaturesList([this.temperature, this.temperature + 0.4, this.temperature - 0.3, this.temperature + 0.2])
      .setStatus(status);
  }

  toJSON() {
    return { energyWh: this.energyWh, temperature: this.temperature };
  }

  loadFromJSON(json) {
    if (Number.isFinite(json?.energyWh)) this.energyWh = Math.max(0, Math.min(this.config.capacityWh, json.energyWh));
    if (Number.isFinite(json?.temperature)) this.temperature = json.temperature;
  }

  /** @returns {string} */
  describe() {
    const flow = this.charging ? 'charging' : `${this.powerW.toFixed(0)} W`;
    return `${this.percent().toFixed(1)} %, ${this.voltage().toFixed(1)} V, ${flow}`;
  }
}

/**
 * The temperatures of the motors: they warm up with the load (standing, walking) and cool down when off.
 */
class Thermal {
  /**
   * @param {import('../robot')} robot
   * @param {string[]} joints
   */
  constructor(robot, joints) {
    this.robot = robot;
    this.temperatures = Object.fromEntries(joints.map(name => [name, 26]));
  }

  /**
   * @param {number} dt
   */
  update(dt) {
    const { body, power } = this.robot;
    let target = 26;
    if (power.motorsOn()) target = body.isWalking() ? 55 : body.isResting() ? 30 : 42;
    for (const name of Object.keys(this.temperatures)) {
      // The knees work the most.
      const jointTarget = name.endsWith('.kn') ? target + (target - 26) * 0.2 : target;
      this.temperatures[name] += ((jointTarget - this.temperatures[name]) * dt) / 300;
    }
  }

  /** @returns {number} */
  maxMotorTemperature() {
    return Math.max(...Object.values(this.temperatures));
  }

  /**
   * @returns {robotStatePb.SystemState}
   */
  toProto() {
    return new robotStatePb.SystemState().setMotorTemperaturesList(
      Object.entries(this.temperatures).map(([name, temperature]) =>
        new robotStatePb.MotorTemperature().setName(name).setTemperature(temperature),
      ),
    );
  }
}

module.exports = { Battery, Thermal };
