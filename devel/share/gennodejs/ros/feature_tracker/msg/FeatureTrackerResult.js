// Auto-generated. Do not edit!

// (in-package feature_tracker.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class FeatureTrackerResult {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.id = null;
      this.cnt = null;
      this.seq = null;
      this.u0 = null;
      this.v0 = null;
      this.u1 = null;
      this.v1 = null;
      this.x = null;
      this.y = null;
      this.z = null;
    }
    else {
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = 0;
      }
      if (initObj.hasOwnProperty('cnt')) {
        this.cnt = initObj.cnt
      }
      else {
        this.cnt = 0;
      }
      if (initObj.hasOwnProperty('seq')) {
        this.seq = initObj.seq
      }
      else {
        this.seq = 0;
      }
      if (initObj.hasOwnProperty('u0')) {
        this.u0 = initObj.u0
      }
      else {
        this.u0 = 0.0;
      }
      if (initObj.hasOwnProperty('v0')) {
        this.v0 = initObj.v0
      }
      else {
        this.v0 = 0.0;
      }
      if (initObj.hasOwnProperty('u1')) {
        this.u1 = initObj.u1
      }
      else {
        this.u1 = 0.0;
      }
      if (initObj.hasOwnProperty('v1')) {
        this.v1 = initObj.v1
      }
      else {
        this.v1 = 0.0;
      }
      if (initObj.hasOwnProperty('x')) {
        this.x = initObj.x
      }
      else {
        this.x = 0.0;
      }
      if (initObj.hasOwnProperty('y')) {
        this.y = initObj.y
      }
      else {
        this.y = 0.0;
      }
      if (initObj.hasOwnProperty('z')) {
        this.z = initObj.z
      }
      else {
        this.z = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FeatureTrackerResult
    // Serialize message field [id]
    bufferOffset = _serializer.int64(obj.id, buffer, bufferOffset);
    // Serialize message field [cnt]
    bufferOffset = _serializer.int64(obj.cnt, buffer, bufferOffset);
    // Serialize message field [seq]
    bufferOffset = _serializer.int64(obj.seq, buffer, bufferOffset);
    // Serialize message field [u0]
    bufferOffset = _serializer.float64(obj.u0, buffer, bufferOffset);
    // Serialize message field [v0]
    bufferOffset = _serializer.float64(obj.v0, buffer, bufferOffset);
    // Serialize message field [u1]
    bufferOffset = _serializer.float64(obj.u1, buffer, bufferOffset);
    // Serialize message field [v1]
    bufferOffset = _serializer.float64(obj.v1, buffer, bufferOffset);
    // Serialize message field [x]
    bufferOffset = _serializer.float64(obj.x, buffer, bufferOffset);
    // Serialize message field [y]
    bufferOffset = _serializer.float64(obj.y, buffer, bufferOffset);
    // Serialize message field [z]
    bufferOffset = _serializer.float64(obj.z, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FeatureTrackerResult
    let len;
    let data = new FeatureTrackerResult(null);
    // Deserialize message field [id]
    data.id = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [cnt]
    data.cnt = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [seq]
    data.seq = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [u0]
    data.u0 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [v0]
    data.v0 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [u1]
    data.u1 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [v1]
    data.v1 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [x]
    data.x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [y]
    data.y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [z]
    data.z = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 80;
  }

  static datatype() {
    // Returns string type for a message object
    return 'feature_tracker/FeatureTrackerResult';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ba9d4897b1e44abdddcf84f9f72d3eb6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int64 id
    int64 cnt
    int64 seq
    float64 u0
    float64 v0
    float64 u1
    float64 v1
    float64 x
    float64 y
    float64 z
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new FeatureTrackerResult(null);
    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = 0
    }

    if (msg.cnt !== undefined) {
      resolved.cnt = msg.cnt;
    }
    else {
      resolved.cnt = 0
    }

    if (msg.seq !== undefined) {
      resolved.seq = msg.seq;
    }
    else {
      resolved.seq = 0
    }

    if (msg.u0 !== undefined) {
      resolved.u0 = msg.u0;
    }
    else {
      resolved.u0 = 0.0
    }

    if (msg.v0 !== undefined) {
      resolved.v0 = msg.v0;
    }
    else {
      resolved.v0 = 0.0
    }

    if (msg.u1 !== undefined) {
      resolved.u1 = msg.u1;
    }
    else {
      resolved.u1 = 0.0
    }

    if (msg.v1 !== undefined) {
      resolved.v1 = msg.v1;
    }
    else {
      resolved.v1 = 0.0
    }

    if (msg.x !== undefined) {
      resolved.x = msg.x;
    }
    else {
      resolved.x = 0.0
    }

    if (msg.y !== undefined) {
      resolved.y = msg.y;
    }
    else {
      resolved.y = 0.0
    }

    if (msg.z !== undefined) {
      resolved.z = msg.z;
    }
    else {
      resolved.z = 0.0
    }

    return resolved;
    }
};

module.exports = FeatureTrackerResult;
