;; Auto-generated. Do not edit!


(when (boundp 'pose_estimate::PoseEstimateResult)
  (if (not (find-package "POSE_ESTIMATE"))
    (make-package "POSE_ESTIMATE"))
  (shadow 'PoseEstimateResult (find-package "POSE_ESTIMATE")))
(unless (find-package "POSE_ESTIMATE::POSEESTIMATERESULT")
  (make-package "POSE_ESTIMATE::POSEESTIMATERESULT"))

(in-package "ROS")
;;//! \htmlinclude PoseEstimateResult.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass pose_estimate::PoseEstimateResult
  :super ros::object
  :slots (_header _q0 _q1 _q2 _q3 _tx _ty _tz _roll _pitch _yaw ))

(defmethod pose_estimate::PoseEstimateResult
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:q0 __q0) 0.0)
    ((:q1 __q1) 0.0)
    ((:q2 __q2) 0.0)
    ((:q3 __q3) 0.0)
    ((:tx __tx) 0.0)
    ((:ty __ty) 0.0)
    ((:tz __tz) 0.0)
    ((:roll __roll) 0.0)
    ((:pitch __pitch) 0.0)
    ((:yaw __yaw) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _q0 (float __q0))
   (setq _q1 (float __q1))
   (setq _q2 (float __q2))
   (setq _q3 (float __q3))
   (setq _tx (float __tx))
   (setq _ty (float __ty))
   (setq _tz (float __tz))
   (setq _roll (float __roll))
   (setq _pitch (float __pitch))
   (setq _yaw (float __yaw))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:q0
   (&optional __q0)
   (if __q0 (setq _q0 __q0)) _q0)
  (:q1
   (&optional __q1)
   (if __q1 (setq _q1 __q1)) _q1)
  (:q2
   (&optional __q2)
   (if __q2 (setq _q2 __q2)) _q2)
  (:q3
   (&optional __q3)
   (if __q3 (setq _q3 __q3)) _q3)
  (:tx
   (&optional __tx)
   (if __tx (setq _tx __tx)) _tx)
  (:ty
   (&optional __ty)
   (if __ty (setq _ty __ty)) _ty)
  (:tz
   (&optional __tz)
   (if __tz (setq _tz __tz)) _tz)
  (:roll
   (&optional __roll)
   (if __roll (setq _roll __roll)) _roll)
  (:pitch
   (&optional __pitch)
   (if __pitch (setq _pitch __pitch)) _pitch)
  (:yaw
   (&optional __yaw)
   (if __yaw (setq _yaw __yaw)) _yaw)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _q0
    8
    ;; float64 _q1
    8
    ;; float64 _q2
    8
    ;; float64 _q3
    8
    ;; float64 _tx
    8
    ;; float64 _ty
    8
    ;; float64 _tz
    8
    ;; float64 _roll
    8
    ;; float64 _pitch
    8
    ;; float64 _yaw
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _q0
       (sys::poke _q0 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _q1
       (sys::poke _q1 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _q2
       (sys::poke _q2 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _q3
       (sys::poke _q3 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _tx
       (sys::poke _tx (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _ty
       (sys::poke _ty (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _tz
       (sys::poke _tz (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _roll
       (sys::poke _roll (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _pitch
       (sys::poke _pitch (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _yaw
       (sys::poke _yaw (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _q0
     (setq _q0 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _q1
     (setq _q1 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _q2
     (setq _q2 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _q3
     (setq _q3 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _tx
     (setq _tx (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _ty
     (setq _ty (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _tz
     (setq _tz (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _roll
     (setq _roll (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _pitch
     (setq _pitch (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _yaw
     (setq _yaw (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get pose_estimate::PoseEstimateResult :md5sum-) "c29be1721413257519ddd67c8a406938")
(setf (get pose_estimate::PoseEstimateResult :datatype-) "pose_estimate/PoseEstimateResult")
(setf (get pose_estimate::PoseEstimateResult :definition-)
      "std_msgs/Header header
float64 q0
float64 q1
float64 q2
float64 q3
float64 tx
float64 ty
float64 tz
float64 roll
float64 pitch
float64 yaw
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

")



(provide :pose_estimate/PoseEstimateResult "c29be1721413257519ddd67c8a406938")


