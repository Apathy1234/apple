# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from pose_est_new/DataCollectionForSim.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg
import std_msgs.msg

class DataCollectionForSim(genpy.Message):
  _md5sum = "ef7c30dc1047a86e342d27ddf8805527"
  _type = "pose_est_new/DataCollectionForSim"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """std_msgs/Header header
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
float64 z"""
  __slots__ = ['header','updateCameraState','orientation_cam','pos_cam','angular_velocity_imu','linear_acceleration_imu']
  _slot_types = ['std_msgs/Header','int32','geometry_msgs/Quaternion','geometry_msgs/Vector3','geometry_msgs/Vector3','geometry_msgs/Vector3']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,updateCameraState,orientation_cam,pos_cam,angular_velocity_imu,linear_acceleration_imu

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(DataCollectionForSim, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.updateCameraState is None:
        self.updateCameraState = 0
      if self.orientation_cam is None:
        self.orientation_cam = geometry_msgs.msg.Quaternion()
      if self.pos_cam is None:
        self.pos_cam = geometry_msgs.msg.Vector3()
      if self.angular_velocity_imu is None:
        self.angular_velocity_imu = geometry_msgs.msg.Vector3()
      if self.linear_acceleration_imu is None:
        self.linear_acceleration_imu = geometry_msgs.msg.Vector3()
    else:
      self.header = std_msgs.msg.Header()
      self.updateCameraState = 0
      self.orientation_cam = geometry_msgs.msg.Quaternion()
      self.pos_cam = geometry_msgs.msg.Vector3()
      self.angular_velocity_imu = geometry_msgs.msg.Vector3()
      self.linear_acceleration_imu = geometry_msgs.msg.Vector3()

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_get_struct_i13d().pack(_x.updateCameraState, _x.orientation_cam.x, _x.orientation_cam.y, _x.orientation_cam.z, _x.orientation_cam.w, _x.pos_cam.x, _x.pos_cam.y, _x.pos_cam.z, _x.angular_velocity_imu.x, _x.angular_velocity_imu.y, _x.angular_velocity_imu.z, _x.linear_acceleration_imu.x, _x.linear_acceleration_imu.y, _x.linear_acceleration_imu.z))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.orientation_cam is None:
        self.orientation_cam = geometry_msgs.msg.Quaternion()
      if self.pos_cam is None:
        self.pos_cam = geometry_msgs.msg.Vector3()
      if self.angular_velocity_imu is None:
        self.angular_velocity_imu = geometry_msgs.msg.Vector3()
      if self.linear_acceleration_imu is None:
        self.linear_acceleration_imu = geometry_msgs.msg.Vector3()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 108
      (_x.updateCameraState, _x.orientation_cam.x, _x.orientation_cam.y, _x.orientation_cam.z, _x.orientation_cam.w, _x.pos_cam.x, _x.pos_cam.y, _x.pos_cam.z, _x.angular_velocity_imu.x, _x.angular_velocity_imu.y, _x.angular_velocity_imu.z, _x.linear_acceleration_imu.x, _x.linear_acceleration_imu.y, _x.linear_acceleration_imu.z,) = _get_struct_i13d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_get_struct_i13d().pack(_x.updateCameraState, _x.orientation_cam.x, _x.orientation_cam.y, _x.orientation_cam.z, _x.orientation_cam.w, _x.pos_cam.x, _x.pos_cam.y, _x.pos_cam.z, _x.angular_velocity_imu.x, _x.angular_velocity_imu.y, _x.angular_velocity_imu.z, _x.linear_acceleration_imu.x, _x.linear_acceleration_imu.y, _x.linear_acceleration_imu.z))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.orientation_cam is None:
        self.orientation_cam = geometry_msgs.msg.Quaternion()
      if self.pos_cam is None:
        self.pos_cam = geometry_msgs.msg.Vector3()
      if self.angular_velocity_imu is None:
        self.angular_velocity_imu = geometry_msgs.msg.Vector3()
      if self.linear_acceleration_imu is None:
        self.linear_acceleration_imu = geometry_msgs.msg.Vector3()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 108
      (_x.updateCameraState, _x.orientation_cam.x, _x.orientation_cam.y, _x.orientation_cam.z, _x.orientation_cam.w, _x.pos_cam.x, _x.pos_cam.y, _x.pos_cam.z, _x.angular_velocity_imu.x, _x.angular_velocity_imu.y, _x.angular_velocity_imu.z, _x.linear_acceleration_imu.x, _x.linear_acceleration_imu.y, _x.linear_acceleration_imu.z,) = _get_struct_i13d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_3I = None
def _get_struct_3I():
    global _struct_3I
    if _struct_3I is None:
        _struct_3I = struct.Struct("<3I")
    return _struct_3I
_struct_i13d = None
def _get_struct_i13d():
    global _struct_i13d
    if _struct_i13d is None:
        _struct_i13d = struct.Struct("<i13d")
    return _struct_i13d
