'use strict';

const time_pb = require('google-protobuf/google/protobuf/timestamp_pb');
const { cloneDeep } = require('lodash');

const header_pb = require('./bosdyn/api/header_pb');

const NSEC_PER_SEC = 10 ** 9;

function populate_response_header(
  response,
  request,
  error_code = header_pb.CommonError.Code.CODE_OK,
  error_msg = null,
) {
  const header = new header_pb.ResponseHeader();
  header.setRequestHeader(request.getHeader());
  header.setRequestReceivedTimestamp(now_timestamp());
  const error = new header_pb.CommonError();
  error.setCode(error_code);
  if (error_msg !== null || error_msg) error.setMessage(error_msg);
  header.setError(error);
  const copied_request = cloneDeep(request);
  strip_large_bytes_fields(copied_request);
  response.setHeader(header);
}

function strip_large_bytes_fields(proto_message) {
  const message_type = proto_message;
  const whitelist_map = get_bytes_field_whitelist();
  if (message_type in whitelist_map) whitelist_map[message_type](proto_message);
}

function get_bytes_field_whitelist() {
  const whitelist_map = {
    GetImageResponse: strip_get_image_response,
    GetLocalGridsResponse: strip_local_grid_responses,
    StoreDataRequest: strip_store_data_request,
    StoreImageRequest: strip_store_image_request,
    RecordSignalTicksRequest: strip_record_signal_tick,
    RecordDataBlobsRequest: strip_record_data_blob,
    AddLogAnnotationRequest: strip_log_annotation,
  };
  return whitelist_map;
}

function strip_image_response(proto_message) {
  proto_message.clearShot();
}

function strip_get_image_response(proto_message) {
  for (const img_resp in proto_message.getImageResponses()) {
    strip_image_response(img_resp);
  }
}

function strip_local_grid_responses(proto_message) {
  for (const grid_resp in proto_message.getLocalGridResponses()) {
    grid_resp.clearLocalGridResponsesList();
  }
}

function strip_store_image_request(proto_message) {
  proto_message.clearImage();
}

function strip_store_data_request(proto_message) {
  proto_message.setData([]);
}

function strip_record_signal_tick(proto_message) {
  for (const tick_data in proto_message.getTickData()) {
    tick_data.clearTickDataList();
  }
}

function strip_record_data_blob(proto_message) {
  for (const blob in proto_message.getBlobData()) {
    blob.RecordDataBlobsRequest();
  }
}

function strip_log_annotation(proto_message) {
  for (const blob in proto_message.getAnnotations().getBlobData()) {
    blob.clearAnnotations();
  }
}

function sec_to_nsec(secs) {
  return secs * NSEC_PER_SEC;
}

function nsec_to_sec(secs) {
  return parseFloat(secs) / NSEC_PER_SEC;
}

function now_nsec() {
  return sec_to_nsec(Date.now() / 1000);
}

function set_timestamp_from_now(timestamp_proto) {
  set_timestamp_from_nsec(timestamp_proto, now_nsec());
}

function set_timestamp_from_nsec(timestamp_proto, time_nsec) {
  timestamp_proto.setSeconds(parseInt(time_nsec / NSEC_PER_SEC));
  timestamp_proto.setNanos(parseInt(time_nsec % NSEC_PER_SEC));
}

function now_timestamp() {
  const now = now_nsec();
  const timestamp_proto = new time_pb.Timestamp();
  set_timestamp_from_nsec(timestamp_proto, now);
  return timestamp_proto;
}

module.exports = {
  populate_response_header,
  now_timestamp,
  set_timestamp_from_nsec,
  now_nsec,
  nsec_to_sec,
  sec_to_nsec,
};
