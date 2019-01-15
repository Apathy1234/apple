# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from feature_tracker/CameraTrackerResult.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import feature_tracker.msg
import std_msgs.msg

class CameraTrackerResult(genpy.Message):
  _md5sum = "d039766b47fa8dfad093dc64535488f0"
  _type = "feature_tracker/CameraTrackerResult"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """std_msgs/Header header
int64 num_of_features
FeatureTrackerResult[] features
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
MSG: feature_tracker/FeatureTrackerResult
int64 id
int64 cnt
int64 seq
float64 u0
float64 v0
float64 u1
float64 v1
float64 x
float64 y
float64 z"""
  __slots__ = ['header','num_of_features','features']
  _slot_types = ['std_msgs/Header','int64','feature_tracker/FeatureTrackerResult[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,num_of_features,features

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(CameraTrackerResult, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.num_of_features is None:
        self.num_of_features = 0
      if self.features is None:
        self.features = []
    else:
      self.header = std_msgs.msg.Header()
      self.num_of_features = 0
      self.features = []

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
      buff.write(_get_struct_q().pack(self.num_of_features))
      length = len(self.features)
      buff.write(_struct_I.pack(length))
      for val1 in self.features:
        _x = val1
        buff.write(_get_struct_3q7d().pack(_x.id, _x.cnt, _x.seq, _x.u0, _x.v0, _x.u1, _x.v1, _x.x, _x.y, _x.z))
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
      if self.features is None:
        self.features = None
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
      start = end
      end += 8
      (self.num_of_features,) = _get_struct_q().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.features = []
      for i in range(0, length):
        val1 = feature_tracker.msg.FeatureTrackerResult()
        _x = val1
        start = end
        end += 80
        (_x.id, _x.cnt, _x.seq, _x.u0, _x.v0, _x.u1, _x.v1, _x.x, _x.y, _x.z,) = _get_struct_3q7d().unpack(str[start:end])
        self.features.append(val1)
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
      buff.write(_get_struct_q().pack(self.num_of_features))
      length = len(self.features)
      buff.write(_struct_I.pack(length))
      for val1 in self.features:
        _x = val1
        buff.write(_get_struct_3q7d().pack(_x.id, _x.cnt, _x.seq, _x.u0, _x.v0, _x.u1, _x.v1, _x.x, _x.y, _x.z))
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
      if self.features is None:
        self.features = None
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
      start = end
      end += 8
      (self.num_of_features,) = _get_struct_q().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.features = []
      for i in range(0, length):
        val1 = feature_tracker.msg.FeatureTrackerResult()
        _x = val1
        start = end
        end += 80
        (_x.id, _x.cnt, _x.seq, _x.u0, _x.v0, _x.u1, _x.v1, _x.x, _x.y, _x.z,) = _get_struct_3q7d().unpack(str[start:end])
        self.features.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_q = None
def _get_struct_q():
    global _struct_q
    if _struct_q is None:
        _struct_q = struct.Struct("<q")
    return _struct_q
_struct_3I = None
def _get_struct_3I():
    global _struct_3I
    if _struct_3I is None:
        _struct_3I = struct.Struct("<3I")
    return _struct_3I
_struct_3q7d = None
def _get_struct_3q7d():
    global _struct_3q7d
    if _struct_3q7d is None:
        _struct_3q7d = struct.Struct("<3q7d")
    return _struct_3q7d
