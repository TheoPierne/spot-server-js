'use strict';

const dataIndexPb = require('../bosdyn/api/data_index_pb');
const { DataServiceService } = require('../bosdyn/api/data_service_grpc_pb');
const { unary } = require('../util');

module.exports = {
  service: DataServiceService,
  func: {
    getDataIndex: unary('GetDataIndex', dataIndexPb.GetDataIndexResponse, (request, { robot }) =>
      new dataIndexPb.GetDataIndexResponse().setDataIndex(robot.dataBuffer.dataIndex(request.getDataQuery())),
    ),
    getEventsComments: unary('GetEventsComments', dataIndexPb.GetEventsCommentsResponse, (request, { robot }) =>
      new dataIndexPb.GetEventsCommentsResponse().setEventsComments(
        robot.dataBuffer.eventsComments(request.getEventCommentRequest()),
      ),
    ),
    getDataBufferStatus: unary('GetDataBufferStatus', dataIndexPb.GetDataBufferStatusResponse, (request, { robot }) =>
      new dataIndexPb.GetDataBufferStatusResponse().setDataBufferStatus(
        robot.dataBuffer.status(request.getGetBlobSpecs()),
      ),
    ),
    getDataPages: unary('GetDataPages', dataIndexPb.GetDataPagesResponse, (request, { robot }) =>
      new dataIndexPb.GetDataPagesResponse().setPagesList(robot.dataBuffer.pages(request.getTimeRange())),
    ),
    deleteDataPages: unary('DeleteDataPages', dataIndexPb.DeleteDataPagesResponse, (request, { robot }) => {
      const result = robot.dataBuffer.deletePages(request.getTimeRange(), request.getPageIdsList());
      return new dataIndexPb.DeleteDataPagesResponse()
        .setBytesDeleted(result.bytesDeleted)
        .setStatusList(result.statuses);
    }),
  },
  directory: [{ name: 'data', type: 'bosdyn.api.DataService', authority: 'data.spot.robot' }],
};
