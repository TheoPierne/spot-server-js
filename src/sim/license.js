'use strict';

const { secToTimestamp } = require('./clock');
const { LicenseInfo } = require('../bosdyn/api/license_pb');

// The features of a fully licensed robot.
const DEFAULT_FEATURES = [
  'arm',
  'auto_return',
  'autowalk',
  'choreography',
  'data_acquisition',
  'docking',
  'gps',
  'graph_nav',
  'mission',
  'remote_mission',
  'spot_check',
];

/**
 * The license of the robot. The console of the simulator can make it expire (the motors then cannot be powered on).
 */
class License {
  /**
   * @param {import('../robot')} robot
   */
  constructor(robot) {
    this.robot = robot;
    const config = robot.config.license ?? {};
    this.features = config.features ?? DEFAULT_FEATURES;
    this.status = LicenseInfo.Status.STATUS_VALID;
    const now = robot.clock.now();
    this.notValidBefore = now - 180 * 86400;
    this.notValidAfter = now + 185 * 86400;
  }

  /** @returns {boolean} */
  isValid() {
    return this.status === LicenseInfo.Status.STATUS_VALID;
  }

  /**
   * @param {string} feature
   * @returns {boolean}
   */
  isEnabled(feature) {
    return this.isValid() && this.features.includes(feature);
  }

  /**
   * @param {boolean} expired
   */
  setExpired(expired) {
    this.status = expired ? LicenseInfo.Status.STATUS_EXPIRED : LicenseInfo.Status.STATUS_VALID;
  }

  /**
   * @returns {LicenseInfo}
   */
  toProto() {
    return new LicenseInfo()
      .setStatus(this.status)
      .setId(`LIC-${this.robot.config.robot.serialNumber}`)
      .setRobotSerial(this.robot.config.robot.serialNumber)
      .setNotValidBefore(secToTimestamp(this.notValidBefore))
      .setNotValidAfter(secToTimestamp(this.notValidAfter))
      .setLicensedFeaturesList([...this.features]);
  }
}

module.exports = { License };
