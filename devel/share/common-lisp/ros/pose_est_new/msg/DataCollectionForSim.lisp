; Auto-generated. Do not edit!


(cl:in-package pose_est_new-msg)


;//! \htmlinclude DataCollectionForSim.msg.html

(cl:defclass <DataCollectionForSim> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (updateCameraState
    :reader updateCameraState
    :initarg :updateCameraState
    :type cl:integer
    :initform 0)
   (orientation_cam
    :reader orientation_cam
    :initarg :orientation_cam
    :type geometry_msgs-msg:Quaternion
    :initform (cl:make-instance 'geometry_msgs-msg:Quaternion))
   (pos_cam
    :reader pos_cam
    :initarg :pos_cam
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (angular_velocity_imu
    :reader angular_velocity_imu
    :initarg :angular_velocity_imu
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (linear_acceleration_imu
    :reader linear_acceleration_imu
    :initarg :linear_acceleration_imu
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3)))
)

(cl:defclass DataCollectionForSim (<DataCollectionForSim>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DataCollectionForSim>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DataCollectionForSim)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pose_est_new-msg:<DataCollectionForSim> is deprecated: use pose_est_new-msg:DataCollectionForSim instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <DataCollectionForSim>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pose_est_new-msg:header-val is deprecated.  Use pose_est_new-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'updateCameraState-val :lambda-list '(m))
(cl:defmethod updateCameraState-val ((m <DataCollectionForSim>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pose_est_new-msg:updateCameraState-val is deprecated.  Use pose_est_new-msg:updateCameraState instead.")
  (updateCameraState m))

(cl:ensure-generic-function 'orientation_cam-val :lambda-list '(m))
(cl:defmethod orientation_cam-val ((m <DataCollectionForSim>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pose_est_new-msg:orientation_cam-val is deprecated.  Use pose_est_new-msg:orientation_cam instead.")
  (orientation_cam m))

(cl:ensure-generic-function 'pos_cam-val :lambda-list '(m))
(cl:defmethod pos_cam-val ((m <DataCollectionForSim>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pose_est_new-msg:pos_cam-val is deprecated.  Use pose_est_new-msg:pos_cam instead.")
  (pos_cam m))

(cl:ensure-generic-function 'angular_velocity_imu-val :lambda-list '(m))
(cl:defmethod angular_velocity_imu-val ((m <DataCollectionForSim>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pose_est_new-msg:angular_velocity_imu-val is deprecated.  Use pose_est_new-msg:angular_velocity_imu instead.")
  (angular_velocity_imu m))

(cl:ensure-generic-function 'linear_acceleration_imu-val :lambda-list '(m))
(cl:defmethod linear_acceleration_imu-val ((m <DataCollectionForSim>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pose_est_new-msg:linear_acceleration_imu-val is deprecated.  Use pose_est_new-msg:linear_acceleration_imu instead.")
  (linear_acceleration_imu m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DataCollectionForSim>) ostream)
  "Serializes a message object of type '<DataCollectionForSim>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'updateCameraState)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'orientation_cam) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pos_cam) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'angular_velocity_imu) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'linear_acceleration_imu) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DataCollectionForSim>) istream)
  "Deserializes a message object of type '<DataCollectionForSim>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'updateCameraState) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'orientation_cam) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pos_cam) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'angular_velocity_imu) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'linear_acceleration_imu) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DataCollectionForSim>)))
  "Returns string type for a message object of type '<DataCollectionForSim>"
  "pose_est_new/DataCollectionForSim")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DataCollectionForSim)))
  "Returns string type for a message object of type 'DataCollectionForSim"
  "pose_est_new/DataCollectionForSim")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DataCollectionForSim>)))
  "Returns md5sum for a message object of type '<DataCollectionForSim>"
  "ef7c30dc1047a86e342d27ddf8805527")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DataCollectionForSim)))
  "Returns md5sum for a message object of type 'DataCollectionForSim"
  "ef7c30dc1047a86e342d27ddf8805527")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DataCollectionForSim>)))
  "Returns full string definition for message of type '<DataCollectionForSim>"
  (cl:format cl:nil "std_msgs/Header header~%int32 updateCameraState~%geometry_msgs/Quaternion orientation_cam~%geometry_msgs/Vector3 pos_cam~%geometry_msgs/Vector3 angular_velocity_imu~%geometry_msgs/Vector3 linear_acceleration_imu~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DataCollectionForSim)))
  "Returns full string definition for message of type 'DataCollectionForSim"
  (cl:format cl:nil "std_msgs/Header header~%int32 updateCameraState~%geometry_msgs/Quaternion orientation_cam~%geometry_msgs/Vector3 pos_cam~%geometry_msgs/Vector3 angular_velocity_imu~%geometry_msgs/Vector3 linear_acceleration_imu~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DataCollectionForSim>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'orientation_cam))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pos_cam))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'angular_velocity_imu))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'linear_acceleration_imu))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DataCollectionForSim>))
  "Converts a ROS message object to a list"
  (cl:list 'DataCollectionForSim
    (cl:cons ':header (header msg))
    (cl:cons ':updateCameraState (updateCameraState msg))
    (cl:cons ':orientation_cam (orientation_cam msg))
    (cl:cons ':pos_cam (pos_cam msg))
    (cl:cons ':angular_velocity_imu (angular_velocity_imu msg))
    (cl:cons ':linear_acceleration_imu (linear_acceleration_imu msg))
))
