// Auto-generated. Do not edit!

// (in-package mynteye_wrapper_d.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class GetParamsRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.key = null;
    }
    else {
      if (initObj.hasOwnProperty('key')) {
        this.key = initObj.key
      }
      else {
        this.key = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetParamsRequest
    // Serialize message field [key]
    bufferOffset = _serializer.uint32(obj.key, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetParamsRequest
    let len;
    let data = new GetParamsRequest(null);
    // Deserialize message field [key]
    data.key = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'mynteye_wrapper_d/GetParamsRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e7693783193e81fc44c88804af723942';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint32 IMG_INTRINSICS=0
    uint32 IMG_EXTRINSICS_RTOL=1
    uint32 IMU_INTRINSICS=2
    uint32 IMU_EXTRINSICS=3
    uint32 key
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetParamsRequest(null);
    if (msg.key !== undefined) {
      resolved.key = msg.key;
    }
    else {
      resolved.key = 0
    }

    return resolved;
    }
};

// Constants for message
GetParamsRequest.Constants = {
  IMG_INTRINSICS: 0,
  IMG_EXTRINSICS_RTOL: 1,
  IMU_INTRINSICS: 2,
  IMU_EXTRINSICS: 3,
}

class GetParamsResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.value = null;
    }
    else {
      if (initObj.hasOwnProperty('value')) {
        this.value = initObj.value
      }
      else {
        this.value = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetParamsResponse
    // Serialize message field [value]
    bufferOffset = _serializer.string(obj.value, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetParamsResponse
    let len;
    let data = new GetParamsResponse(null);
    // Deserialize message field [value]
    data.value = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.value.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'mynteye_wrapper_d/GetParamsResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '64e58419496c7248b4ef25731f88b8c3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string value
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetParamsResponse(null);
    if (msg.value !== undefined) {
      resolved.value = msg.value;
    }
    else {
      resolved.value = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: GetParamsRequest,
  Response: GetParamsResponse,
  md5sum() { return '27490e4b5cf4d32a761bad9cafd48f69'; },
  datatype() { return 'mynteye_wrapper_d/GetParams'; }
};
