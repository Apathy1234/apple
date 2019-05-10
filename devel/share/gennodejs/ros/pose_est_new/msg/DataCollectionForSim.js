// Auto-generated. Do not edit!

// (in-package pose_est_new.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class DataCollectionForSim {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.updateCameraState = null;
      this.orientation_cam = null;
      this.pos_cam = null;
      this.angular_velocity_imu = null;
      this.linear_acceleration_imu = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('updateCameraState')) {
        this.updateCameraState = initObj.updateCameraState
      }
      else {
        this.updateCameraState = 0;
      }
      if (initObj.hasOwnProperty('orientation_cam')) {
        this.orientation_cam = initObj.orientation_cam
      }
      else {
        this.orientation_cam = new geometry_msgs.msg.Quaternion();
      }
      if (initObj.hasOwnProperty('pos_cam')) {
        this.pos_cam = initObj.pos_cam
      }
      else {
        this.pos_cam = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('angular_velocity_imu')) {
        this.angular_velocity_imu = initObj.angular_velocity_imu
      }
      else {
        this.angular_velocity_imu = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('linear_acceleration_imu')) {
        this.linear_acceleration_imu = initObj.linear_acceleration_imu
      }
      else {
        this.linear_acceleration_imu = new geometry_msgs.msg.Vector3();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DataCollectionForSim
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [updateCameraState]
    bufferOffset = _serializer.int32(obj.updateCameraState, buffer, bufferOffset);
    // Serialize message field [orientation_cam]
    bufferOffset = geometry_msgs.msg.Quaternion.serialize(obj.orientation_cam, buffer, bufferOffset);
    // Serialize message field [pos_cam]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.pos_cam, buffer, bufferOffset);
    // Serialize message field [angular_velocity_imu]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.angular_velocity_imu, buffer, bufferOffset);
    // Serialize message field [linear_acceleration_imu]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.linear_acceleration_imu, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DataCollectionForSim
    let len;
    let data = new DataCollectionForSim(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [updateCameraState]
    data.updateCameraState = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [orientation_cam]
    data.orientation_cam = geometry_msgs.msg.Quaternion.deserialize(buffer, bufferOffset);
    // Deserialize message field [pos_cam]
    data.pos_cam = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [angular_velocity_imu]
    data.angular_velocity_imu = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [linear_acceleration_imu]
    data.linear_acceleration_imu = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 108;
  }

  static datatype() {
    // Returns string type for a message object
    return 'pose_est_new/DataCollectionForSim';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ef7c30dc1047a86e342d27ddf8805527';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    int32 updateCameraState
    geometry_msgs/Quaternion orientation_cam
    geometry_msgs/Vector3 pos_cam
    geometry_msgs/Vector3 angular_velocity_imu
    geometry_msgs/Vector3 linear_acceleration_imu
    
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
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    ================================================================================
    MSG: geometry_msgs/Vector3
    # This represents a vector in free space. 
    # It is only meant to represent a direction. Therefore, it does not
    # make sense to apply a translation to it (e.g., when applying a 
    # generic rigid transformation to a Vector3, tf2 will only apply the
    # rotation). If you want your data to be translatable too, use the
    # geometry_msgs/Point message instead.
    
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
    const resolved = new DataCollectionForSim(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.updateCameraState !== undefined) {
      resolved.updateCameraState = msg.updateCameraState;
    }
    else {
      resolved.updateCameraState = 0
    }

    if (msg.orientation_cam !== undefined) {
      resolved.orientation_cam = geometry_msgs.msg.Quaternion.Resolve(msg.orientation_cam)
    }
    else {
      resolved.orientation_cam = new geometry_msgs.msg.Quaternion()
    }

    if (msg.pos_cam !== undefined) {
      resolved.pos_cam = geometry_msgs.msg.Vector3.Resolve(msg.pos_cam)
    }
    else {
      resolved.pos_cam = new geometry_msgs.msg.Vector3()
    }

    if (msg.angular_velocity_imu !== undefined) {
      resolved.angular_velocity_imu = geometry_msgs.msg.Vector3.Resolve(msg.angular_velocity_imu)
    }
    else {
      resolved.angular_velocity_imu = new geometry_msgs.msg.Vector3()
    }

    if (msg.linear_acceleration_imu !== undefined) {
      resolved.linear_acceleration_imu = geometry_msgs.msg.Vector3.Resolve(msg.linear_acceleration_imu)
    }
    else {
      resolved.linear_acceleration_imu = new geometry_msgs.msg.Vector3()
    }

    return resolved;
    }
};

module.exports = DataCollectionForSim;
