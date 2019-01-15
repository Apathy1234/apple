; Auto-generated. Do not edit!


(cl:in-package feature_tracker-msg)


;//! \htmlinclude CameraTrackerResult.msg.html

(cl:defclass <CameraTrackerResult> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (num_of_features
    :reader num_of_features
    :initarg :num_of_features
    :type cl:integer
    :initform 0)
   (features
    :reader features
    :initarg :features
    :type (cl:vector feature_tracker-msg:FeatureTrackerResult)
   :initform (cl:make-array 0 :element-type 'feature_tracker-msg:FeatureTrackerResult :initial-element (cl:make-instance 'feature_tracker-msg:FeatureTrackerResult))))
)

(cl:defclass CameraTrackerResult (<CameraTrackerResult>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CameraTrackerResult>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CameraTrackerResult)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name feature_tracker-msg:<CameraTrackerResult> is deprecated: use feature_tracker-msg:CameraTrackerResult instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <CameraTrackerResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader feature_tracker-msg:header-val is deprecated.  Use feature_tracker-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'num_of_features-val :lambda-list '(m))
(cl:defmethod num_of_features-val ((m <CameraTrackerResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader feature_tracker-msg:num_of_features-val is deprecated.  Use feature_tracker-msg:num_of_features instead.")
  (num_of_features m))

(cl:ensure-generic-function 'features-val :lambda-list '(m))
(cl:defmethod features-val ((m <CameraTrackerResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader feature_tracker-msg:features-val is deprecated.  Use feature_tracker-msg:features instead.")
  (features m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CameraTrackerResult>) ostream)
  "Serializes a message object of type '<CameraTrackerResult>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'num_of_features)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'features))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'features))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CameraTrackerResult>) istream)
  "Deserializes a message object of type '<CameraTrackerResult>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'num_of_features) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'features) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'features)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'feature_tracker-msg:FeatureTrackerResult))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CameraTrackerResult>)))
  "Returns string type for a message object of type '<CameraTrackerResult>"
  "feature_tracker/CameraTrackerResult")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CameraTrackerResult)))
  "Returns string type for a message object of type 'CameraTrackerResult"
  "feature_tracker/CameraTrackerResult")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CameraTrackerResult>)))
  "Returns md5sum for a message object of type '<CameraTrackerResult>"
  "d039766b47fa8dfad093dc64535488f0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CameraTrackerResult)))
  "Returns md5sum for a message object of type 'CameraTrackerResult"
  "d039766b47fa8dfad093dc64535488f0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CameraTrackerResult>)))
  "Returns full string definition for message of type '<CameraTrackerResult>"
  (cl:format cl:nil "std_msgs/Header header~%int64 num_of_features~%FeatureTrackerResult[] features~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: feature_tracker/FeatureTrackerResult~%int64 id~%int64 cnt~%int64 seq~%float64 u0~%float64 v0~%float64 u1~%float64 v1~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CameraTrackerResult)))
  "Returns full string definition for message of type 'CameraTrackerResult"
  (cl:format cl:nil "std_msgs/Header header~%int64 num_of_features~%FeatureTrackerResult[] features~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: feature_tracker/FeatureTrackerResult~%int64 id~%int64 cnt~%int64 seq~%float64 u0~%float64 v0~%float64 u1~%float64 v1~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CameraTrackerResult>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     8
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'features) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CameraTrackerResult>))
  "Converts a ROS message object to a list"
  (cl:list 'CameraTrackerResult
    (cl:cons ':header (header msg))
    (cl:cons ':num_of_features (num_of_features msg))
    (cl:cons ':features (features msg))
))
