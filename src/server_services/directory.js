'use strict';

const directoryPb = require('../bosdyn/api/directory_pb');
const directoryRegistrationPb = require('../bosdyn/api/directory_registration_pb');
const { DirectoryRegistrationServiceService } = require('../bosdyn/api/directory_registration_service_grpc_pb');
const { DirectoryServiceService } = require('../bosdyn/api/directory_service_grpc_pb');
const { unary } = require('../util');

/**
 * ListServiceEntries.
 * @param {directoryPb.ListServiceEntriesRequest} request
 * @param {{robot: import('../robot').Robot}} context
 * @returns {directoryPb.ListServiceEntriesResponse}
 */
function listServiceEntries(request, { robot }) {
  return new directoryPb.ListServiceEntriesResponse().setServiceEntriesList(robot.directory.list());
}

/**
 * GetServiceEntry.
 * @param {directoryPb.GetServiceEntryRequest} request
 * @param {{robot: import('../robot').Robot}} context
 * @returns {directoryPb.GetServiceEntryResponse}
 */
function getServiceEntry(request, { robot }) {
  const { Status } = directoryPb.GetServiceEntryResponse;
  const entry = robot.directory.get(request.getServiceName());
  const response = new directoryPb.GetServiceEntryResponse();
  return entry
    ? response.setStatus(Status.STATUS_OK).setServiceEntry(entry)
    : response.setStatus(Status.STATUS_NONEXISTENT_SERVICE);
}

const registration = {
  registerService: unary('RegisterService', directoryRegistrationPb.RegisterServiceResponse, (request, { robot }) =>
    new directoryRegistrationPb.RegisterServiceResponse().setStatus(
      robot.directory.register(request.getServiceEntry() ?? new directoryPb.ServiceEntry(), request.getEndpoint()),
    ),
  ),
  updateService: unary('UpdateService', directoryRegistrationPb.UpdateServiceResponse, (request, { robot }) =>
    new directoryRegistrationPb.UpdateServiceResponse().setStatus(
      robot.directory.update(request.getServiceEntry() ?? new directoryPb.ServiceEntry(), request.getEndpoint()),
    ),
  ),
  unregisterService: unary(
    'UnregisterService',
    directoryRegistrationPb.UnregisterServiceResponse,
    (request, { robot }) =>
      new directoryRegistrationPb.UnregisterServiceResponse().setStatus(
        robot.directory.unregister(request.getServiceName()),
      ),
  ),
};

module.exports = {
  service: DirectoryServiceService,
  func: {
    listServiceEntries: unary('ListServiceEntries', directoryPb.ListServiceEntriesResponse, listServiceEntries),
    getServiceEntry: unary('GetServiceEntry', directoryPb.GetServiceEntryResponse, getServiceEntry),
  },
  directory: [
    { name: 'directory', type: 'bosdyn.api.DirectoryService', authority: 'api.spot.robot' },
    { name: 'directory-registration', type: 'bosdyn.api.DirectoryRegistrationService', authority: 'api.spot.robot' },
  ],
  extraServices: [{ service: DirectoryRegistrationServiceService, func: registration }],
};
