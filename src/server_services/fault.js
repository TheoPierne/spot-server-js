'use strict';

const { FaultServiceService } = require('../bosdyn/api/fault_service_grpc_pb');
const serviceFaultPb = require('../bosdyn/api/service_fault_pb');
const { unary } = require('../util');

module.exports = {
  service: FaultServiceService,
  func: {
    triggerServiceFault: unary(
      'TriggerServiceFault',
      serviceFaultPb.TriggerServiceFaultResponse,
      (request, { robot }) =>
        new serviceFaultPb.TriggerServiceFaultResponse().setStatus(
          robot.faults.triggerServiceFault(request.getFault() ?? new serviceFaultPb.ServiceFault()),
        ),
    ),
    clearServiceFault: unary('ClearServiceFault', serviceFaultPb.ClearServiceFaultResponse, (request, { robot }) =>
      new serviceFaultPb.ClearServiceFaultResponse().setStatus(
        robot.faults.clearServiceFault(
          request.getFaultId() ?? new serviceFaultPb.ServiceFaultId(),
          request.getClearAllServiceFaults(),
          request.getClearAllPayloadFaults(),
        ),
      ),
    ),
  },
  directory: [{ name: 'fault', type: 'bosdyn.api.FaultService', authority: 'fault.spot.robot' }],
};
