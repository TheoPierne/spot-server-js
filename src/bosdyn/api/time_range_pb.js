// source: bosdyn/api/time_range.proto
/**
 * @fileoverview
 * @enhanceable
 * @suppress {missingRequire} reports error on implicit type usages.
 * @suppress {messageConventions} JS Compiler reports an error if a variable or
 *     field starts with 'MSG_' and isn't a translatable message.
 * @public
 */
// GENERATED CODE -- DO NOT EDIT!
/* eslint-disable */
// @ts-nocheck

var jspb = require('google-protobuf');
var goog = jspb;
var global = Function('return this')();

var google_protobuf_timestamp_pb = require('google-protobuf/google/protobuf/timestamp_pb.js');
goog.object.extend(proto, google_protobuf_timestamp_pb);
goog.exportSymbol('proto.bosdyn.api.TimeRange', null, global);
/**
 * Generated by JsPbCodeGenerator.
 * @param {Array=} opt_data Optional initial data array, typically from a
 * server response, or constructed directly in Javascript. The array is used
 * in place and becomes part of the constructed object. It is not cloned.
 * If no data is provided, the constructed object will be empty, but still
 * valid.
 * @extends {jspb.Message}
 * @constructor
 */
proto.bosdyn.api.TimeRange = function(opt_data) {
  jspb.Message.initialize(this, opt_data, 0, -1, null, null);
};
goog.inherits(proto.bosdyn.api.TimeRange, jspb.Message);
if (goog.DEBUG && !COMPILED) {
  /**
   * @public
   * @override
   */
  proto.bosdyn.api.TimeRange.displayName = 'proto.bosdyn.api.TimeRange';
}



if (jspb.Message.GENERATE_TO_OBJECT) {
/**
 * Creates an object representation of this proto.
 * Field names that are reserved in JavaScript and will be renamed to pb_name.
 * Optional fields that are not set will be set to undefined.
 * To access a reserved field use, foo.pb_<name>, eg, foo.pb_default.
 * For the list of reserved names please see:
 *     net/proto2/compiler/js/internal/generator.cc#kKeyword.
 * @param {boolean=} opt_includeInstance Deprecated. whether to include the
 *     JSPB instance for transitional soy proto support:
 *     http://goto/soy-param-migration
 * @return {!Object}
 */
proto.bosdyn.api.TimeRange.prototype.toObject = function(opt_includeInstance) {
  return proto.bosdyn.api.TimeRange.toObject(opt_includeInstance, this);
};


/**
 * Static version of the {@see toObject} method.
 * @param {boolean|undefined} includeInstance Deprecated. Whether to include
 *     the JSPB instance for transitional soy proto support:
 *     http://goto/soy-param-migration
 * @param {!proto.bosdyn.api.TimeRange} msg The msg instance to transform.
 * @return {!Object}
 * @suppress {unusedLocalVariables} f is only used for nested messages
 */
proto.bosdyn.api.TimeRange.toObject = function(includeInstance, msg) {
  var f, obj = {
    start: (f = msg.getStart()) && google_protobuf_timestamp_pb.Timestamp.toObject(includeInstance, f),
    end: (f = msg.getEnd()) && google_protobuf_timestamp_pb.Timestamp.toObject(includeInstance, f)
  };

  if (includeInstance) {
    obj.$jspbMessageInstance = msg;
  }
  return obj;
};
}


/**
 * Deserializes binary data (in protobuf wire format).
 * @param {jspb.ByteSource} bytes The bytes to deserialize.
 * @return {!proto.bosdyn.api.TimeRange}
 */
proto.bosdyn.api.TimeRange.deserializeBinary = function(bytes) {
  var reader = new jspb.BinaryReader(bytes);
  var msg = new proto.bosdyn.api.TimeRange;
  return proto.bosdyn.api.TimeRange.deserializeBinaryFromReader(msg, reader);
};


/**
 * Deserializes binary data (in protobuf wire format) from the
 * given reader into the given message object.
 * @param {!proto.bosdyn.api.TimeRange} msg The message object to deserialize into.
 * @param {!jspb.BinaryReader} reader The BinaryReader to use.
 * @return {!proto.bosdyn.api.TimeRange}
 */
proto.bosdyn.api.TimeRange.deserializeBinaryFromReader = function(msg, reader) {
  while (reader.nextField()) {
    if (reader.isEndGroup()) {
      break;
    }
    var field = reader.getFieldNumber();
    switch (field) {
    case 1:
      var value = new google_protobuf_timestamp_pb.Timestamp;
      reader.readMessage(value,google_protobuf_timestamp_pb.Timestamp.deserializeBinaryFromReader);
      msg.setStart(value);
      break;
    case 2:
      var value = new google_protobuf_timestamp_pb.Timestamp;
      reader.readMessage(value,google_protobuf_timestamp_pb.Timestamp.deserializeBinaryFromReader);
      msg.setEnd(value);
      break;
    default:
      reader.skipField();
      break;
    }
  }
  return msg;
};


/**
 * Serializes the message to binary data (in protobuf wire format).
 * @return {!Uint8Array}
 */
proto.bosdyn.api.TimeRange.prototype.serializeBinary = function() {
  var writer = new jspb.BinaryWriter();
  proto.bosdyn.api.TimeRange.serializeBinaryToWriter(this, writer);
  return writer.getResultBuffer();
};


/**
 * Serializes the given message to binary data (in protobuf wire
 * format), writing to the given BinaryWriter.
 * @param {!proto.bosdyn.api.TimeRange} message
 * @param {!jspb.BinaryWriter} writer
 * @suppress {unusedLocalVariables} f is only used for nested messages
 */
proto.bosdyn.api.TimeRange.serializeBinaryToWriter = function(message, writer) {
  var f = undefined;
  f = message.getStart();
  if (f != null) {
    writer.writeMessage(
      1,
      f,
      google_protobuf_timestamp_pb.Timestamp.serializeBinaryToWriter
    );
  }
  f = message.getEnd();
  if (f != null) {
    writer.writeMessage(
      2,
      f,
      google_protobuf_timestamp_pb.Timestamp.serializeBinaryToWriter
    );
  }
};


/**
 * optional google.protobuf.Timestamp start = 1;
 * @return {?proto.google.protobuf.Timestamp}
 */
proto.bosdyn.api.TimeRange.prototype.getStart = function() {
  return /** @type{?proto.google.protobuf.Timestamp} */ (
    jspb.Message.getWrapperField(this, google_protobuf_timestamp_pb.Timestamp, 1));
};


/**
 * @param {?proto.google.protobuf.Timestamp|undefined} value
 * @return {!proto.bosdyn.api.TimeRange} returns this
*/
proto.bosdyn.api.TimeRange.prototype.setStart = function(value) {
  return jspb.Message.setWrapperField(this, 1, value);
};


/**
 * Clears the message field making it undefined.
 * @return {!proto.bosdyn.api.TimeRange} returns this
 */
proto.bosdyn.api.TimeRange.prototype.clearStart = function() {
  return this.setStart(undefined);
};


/**
 * Returns whether this field is set.
 * @return {boolean}
 */
proto.bosdyn.api.TimeRange.prototype.hasStart = function() {
  return jspb.Message.getField(this, 1) != null;
};


/**
 * optional google.protobuf.Timestamp end = 2;
 * @return {?proto.google.protobuf.Timestamp}
 */
proto.bosdyn.api.TimeRange.prototype.getEnd = function() {
  return /** @type{?proto.google.protobuf.Timestamp} */ (
    jspb.Message.getWrapperField(this, google_protobuf_timestamp_pb.Timestamp, 2));
};


/**
 * @param {?proto.google.protobuf.Timestamp|undefined} value
 * @return {!proto.bosdyn.api.TimeRange} returns this
*/
proto.bosdyn.api.TimeRange.prototype.setEnd = function(value) {
  return jspb.Message.setWrapperField(this, 2, value);
};


/**
 * Clears the message field making it undefined.
 * @return {!proto.bosdyn.api.TimeRange} returns this
 */
proto.bosdyn.api.TimeRange.prototype.clearEnd = function() {
  return this.setEnd(undefined);
};


/**
 * Returns whether this field is set.
 * @return {boolean}
 */
proto.bosdyn.api.TimeRange.prototype.hasEnd = function() {
  return jspb.Message.getField(this, 2) != null;
};


goog.object.extend(exports, proto.bosdyn.api);
