'use strict';

const directory_pb = require('../bosdyn/api/directory_pb');
const directory_service_pb_grpc = require('../bosdyn/api/directory_service_grpc_pb');
const { LoggerUtil } = require('../loggerUtil');

const { populate_response_header } = require('../util');

const logger = LoggerUtil.getLogger('DIRECTORY');

function listServiceEntries(call, callback) {
  logger.info('New request /listServiceEntries !');
  let reply = new directory_pb.ListServiceEntriesResponse();
  populate_response_header(reply, call.request);

  const serviceAuth = new directory_pb.ServiceEntry()
  .setName('auth')
  .setType('bosdyn.api.AuthService')
  .setAuthority('auth.spot.robot')
  .setUserTokenRequired(false);

  const serviceId = new directory_pb.ServiceEntry()
  .setName('robot-id')
  .setType('bosdyn.api.RobotIdService')
  .setAuthority('id.spot.robot')
  .setUserTokenRequired(true);

  const serviceDirectory = new directory_pb.ServiceEntry()
  .setName('directory')
  .setType('bosdyn.api.DirectoryService')
  .setAuthority('api.spot.robot')
  .setUserTokenRequired(true);

  const serviceEstop = new directory_pb.ServiceEntry()
  .setName('estop')
  .setType('bosdyn.api.EstopService')
  .setAuthority('estop.spot.robot')
  .setUserTokenRequired(true);

  const serviceLease = new directory_pb.ServiceEntry()
  .setName('lease')
  .setType('bosdyn.api.LeaseService')
  .setAuthority('api.spot.robot')
  .setUserTokenRequired(true);

  const serviceImage = new directory_pb.ServiceEntry()
  .setName('image')
  .setType('bosdyn.api.ImageService')
  .setAuthority('api.spot.robot')
  .setUserTokenRequired(true);

  const serviceState = new directory_pb.ServiceEntry()
  .setName('robot-state')
  .setType('bosdyn.api.RobotStateService')
  .setAuthority('api.spot.robot')
  .setUserTokenRequired(true);

  const serviceCommand = new directory_pb.ServiceEntry()
  .setName('robot-command')
  .setType('bosdyn.api.RobotCommandService')
  .setAuthority('api.spot.robot')
  .setUserTokenRequired(true);

  const serviceTimeSync = new directory_pb.ServiceEntry()
  .setName('time-sync')
  .setType('bosdyn.api.TimeSyncService')
  .setAuthority('timesync.spot.robot')
  .setUserTokenRequired(true);

  const servicePower = new directory_pb.ServiceEntry()
  .setName('power')
  .setType('bosdyn.api.PowerService')
  .setAuthority('api.spot.robot')
  .setUserTokenRequired(true);

  const serviceDock = new directory_pb.ServiceEntry()
  .setName('docking')
  .setType('bosdyn.api.docking.DockingService')
  .setAuthority('api.spot.robot')
  .setUserTokenRequired(true);

  const serviceIR = new directory_pb.ServiceEntry()
  .setName('ir-enable-disable-service')
  .setType('bosdyn.api.IREnableDisableService')
  .setAuthority('api.spot.robot')
  .setUserTokenRequired(true);

  const serviceAutoReturn = new directory_pb.ServiceEntry()
  .setName('auto-return')
  .setType('bosdyn.api.auto_return.AutoReturnService')
  .setAuthority('api.spot.robot')
  .setUserTokenRequired(true);

  const serviceMission = new directory_pb.ServiceEntry()
  .setName('robot-mission')
  .setType('bosdyn.api.mission.MissionService')
  .setAuthority('api.spot.robot')
  .setUserTokenRequired(true);

  const serviceDataBuffer = new directory_pb.ServiceEntry()
  .setName('data-buffer')
  .setType('bosdyn.api.DataBufferService')
  .setAuthority('api.spot.robot')
  .setUserTokenRequired(true);

  const serviceWorldObject = new directory_pb.ServiceEntry()
  .setName('world-objects')
  .setType('bosdyn.api.WorldObjectService')
  .setAuthority('api.spot.robot')
  .setUserTokenRequired(true);

  const serviceDataService = new directory_pb.ServiceEntry()
  .setName('data-service')
  .setType('bosdyn.api.DataService')
  .setAuthority('api.spot.robot')
  .setUserTokenRequired(true);

  reply.setServiceEntriesList([
    serviceAuth,
    serviceId,
    serviceDirectory,
    serviceEstop,
    serviceLease,
    serviceImage,
    serviceState,
    serviceCommand,
    serviceTimeSync,
    servicePower,
    serviceDock,
    serviceIR,
    serviceAutoReturn,
    serviceMission,
    serviceDataBuffer,
    serviceWorldObject,
    serviceDataService,
    ]);

  callback(null, reply);
}

function getServiceEntry(call, callback) {
  logger.info('New request /getServiceEntry !');
  let reply = new directory_pb.GetServiceEntryResponse();
  populate_response_header(reply, call.request);

  const serviceDirectory = new directory_pb.ServiceEntry()
  .setName('directory')
  .setType('bosdyn.api.DirectoryService')
  .setAuthority('api.spot.robot')
  .setUserTokenRequired(true);

  reply.setStatus(1).setServiceEntry(serviceDirectory);

  callback(null, reply);
}

module.exports = {
  service: directory_service_pb_grpc.DirectoryServiceService,
  func: {
    listServiceEntries,
    getServiceEntry,
  },
};
