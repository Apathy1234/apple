// Auto-generated. Do not edit!

// (in-package feature_tracker.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class FeatureTrackerResule {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.Header = null;
      this.id = null;
      this.lifeTime = null;
      this.u0 = null;
      this.v0 = null;
      this.u1 = null;
      this.v1 = null;
    }
    else {
      if (initObj.hasOwnProperty('Header')) {
        this.Header = initObj.Header
      }
      else {
        this.Header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = 0;
      }
      if (initObj.hasOwnProperty('lifeTime')) {
        this.lifeTime = initObj.lifeTime
      }
      else {
        this.lifeTime = 0;
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
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FeatureTrackerResule
    // Serialize message field [Header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.Header, buffer, bufferOffset);
    // Serialize message field [id]
    bufferOffset = _serializer.int64(obj.id, buffer, bufferOffset);
    // Serialize message field [lifeTime]
    bufferOffset = _serializer.int64(obj.lifeTime, buffer, bufferOffset);
    // Serialize message field [u0]
    bufferOffset = _serializer.float64(obj.u0, buffer, bufferOffset);
    // Serialize message field [v0]
    bufferOffset = _serializer.float64(obj.v0, buffer, bufferOffset);
    // Serialize message field [u1]
    bufferOffset = _serializer.float64(obj.u1, buffer, bufferOffset);
    // Serialize message field [v1]
    bufferOffset = _serializer.float64(obj.v1, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FeatureTrackerResule
    let len;
    let data = new FeatureTrackerResule(null);
    // Deserialize message field [Header]
    data.Header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [id]
    data.id = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [lifeTime]
    data.lifeTime = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [u0]
    data.u0 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [v0]
    data.v0 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [u1]
    data.u1 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [v1]
    data.v1 = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.Header);
    return length + 48;
  }

  static datatype() {
    // Returns string type for a message object
    return 'feature_tracker/FeatureTrackerResule';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ab82e194225e796f26c8c2c76b2bd2bb';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header Header
    int64 id
    int64 lifeTime
    float64 u0
    float64 v0
    float64 u1
    float64 v1
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    # 0: no frame
    # 1: global frame
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new FeatureTrackerResule(null);
    if (msg.Header !== undefined) {
      resolved.Header = std_msgs.msg.Header.Resolve(msg.Header)
    }
    else {
      resolved.Header = new std_msgs.msg.Header()
    }

    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = 0
    }

    if (msg.lifeTime !== undefined) {
      resolved.lifeTime = msg.lifeTime;
    }
    else {
      resolved.lifeTime = 0
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

    return resolved;
    }
};

module.exports = FeatureTrackerResule;
