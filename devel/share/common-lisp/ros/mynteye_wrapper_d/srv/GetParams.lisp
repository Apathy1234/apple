; Auto-generated. Do not edit!


(cl:in-package mynteye_wrapper_d-srv)


;//! \htmlinclude GetParams-request.msg.html

(cl:defclass <GetParams-request> (roslisp-msg-protocol:ros-message)
  ((key
    :reader key
    :initarg :key
    :type cl:integer
    :initform 0))
)

(cl:defclass GetParams-request (<GetParams-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetParams-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetParams-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mynteye_wrapper_d-srv:<GetParams-request> is deprecated: use mynteye_wrapper_d-srv:GetParams-request instead.")))

(cl:ensure-generic-function 'key-val :lambda-list '(m))
(cl:defmethod key-val ((m <GetParams-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mynteye_wrapper_d-srv:key-val is deprecated.  Use mynteye_wrapper_d-srv:key instead.")
  (key m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<GetParams-request>)))
    "Constants for message type '<GetParams-request>"
  '((:IMG_INTRINSICS . 0)
    (:IMG_EXTRINSICS_RTOL . 1)
    (:IMU_INTRINSICS . 2)
    (:IMU_EXTRINSICS . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'GetParams-request)))
    "Constants for message type 'GetParams-request"
  '((:IMG_INTRINSICS . 0)
    (:IMG_EXTRINSICS_RTOL . 1)
    (:IMU_INTRINSICS . 2)
    (:IMU_EXTRINSICS . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetParams-request>) ostream)
  "Serializes a message object of type '<GetParams-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'key)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'key)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'key)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'key)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetParams-request>) istream)
  "Deserializes a message object of type '<GetParams-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'key)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'key)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'key)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'key)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetParams-request>)))
  "Returns string type for a service object of type '<GetParams-request>"
  "mynteye_wrapper_d/GetParamsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetParams-request)))
  "Returns string type for a service object of type 'GetParams-request"
  "mynteye_wrapper_d/GetParamsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetParams-request>)))
  "Returns md5sum for a message object of type '<GetParams-request>"
  "27490e4b5cf4d32a761bad9cafd48f69")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetParams-request)))
  "Returns md5sum for a message object of type 'GetParams-request"
  "27490e4b5cf4d32a761bad9cafd48f69")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetParams-request>)))
  "Returns full string definition for message of type '<GetParams-request>"
  (cl:format cl:nil "uint32 IMG_INTRINSICS=0~%uint32 IMG_EXTRINSICS_RTOL=1~%uint32 IMU_INTRINSICS=2~%uint32 IMU_EXTRINSICS=3~%uint32 key~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetParams-request)))
  "Returns full string definition for message of type 'GetParams-request"
  (cl:format cl:nil "uint32 IMG_INTRINSICS=0~%uint32 IMG_EXTRINSICS_RTOL=1~%uint32 IMU_INTRINSICS=2~%uint32 IMU_EXTRINSICS=3~%uint32 key~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetParams-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetParams-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetParams-request
    (cl:cons ':key (key msg))
))
;//! \htmlinclude GetParams-response.msg.html

(cl:defclass <GetParams-response> (roslisp-msg-protocol:ros-message)
  ((value
    :reader value
    :initarg :value
    :type cl:string
    :initform ""))
)

(cl:defclass GetParams-response (<GetParams-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetParams-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetParams-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mynteye_wrapper_d-srv:<GetParams-response> is deprecated: use mynteye_wrapper_d-srv:GetParams-response instead.")))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <GetParams-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mynteye_wrapper_d-srv:value-val is deprecated.  Use mynteye_wrapper_d-srv:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetParams-response>) ostream)
  "Serializes a message object of type '<GetParams-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'value))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetParams-response>) istream)
  "Deserializes a message object of type '<GetParams-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'value) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'value) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetParams-response>)))
  "Returns string type for a service object of type '<GetParams-response>"
  "mynteye_wrapper_d/GetParamsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetParams-response)))
  "Returns string type for a service object of type 'GetParams-response"
  "mynteye_wrapper_d/GetParamsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetParams-response>)))
  "Returns md5sum for a message object of type '<GetParams-response>"
  "27490e4b5cf4d32a761bad9cafd48f69")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetParams-response)))
  "Returns md5sum for a message object of type 'GetParams-response"
  "27490e4b5cf4d32a761bad9cafd48f69")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetParams-response>)))
  "Returns full string definition for message of type '<GetParams-response>"
  (cl:format cl:nil "string value~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetParams-response)))
  "Returns full string definition for message of type 'GetParams-response"
  (cl:format cl:nil "string value~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetParams-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'value))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetParams-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetParams-response
    (cl:cons ':value (value msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetParams)))
  'GetParams-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetParams)))
  'GetParams-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetParams)))
  "Returns string type for a service object of type '<GetParams>"
  "mynteye_wrapper_d/GetParams")