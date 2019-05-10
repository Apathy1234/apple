;; Auto-generated. Do not edit!


(when (boundp 'pose_est_new::DataCollectionForSim)
  (if (not (find-package "POSE_EST_NEW"))
    (make-package "POSE_EST_NEW"))
  (shadow 'DataCollectionForSim (find-package "POSE_EST_NEW")))
(unless (find-package "POSE_EST_NEW::DATACOLLECTIONFORSIM")
  (make-package "POSE_EST_NEW::DATACOLLECTIONFORSIM"))

(in-package "ROS")
;;//! \htmlinclude DataCollectionForSim.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass pose_est_new::DataCollectionForSim
  :super ros::object
  :slots (_header _updateCameraState _orientation_cam _pos_cam _angular_velocity_imu _linear_acceleration_imu ))

(defmethod pose_est_new::DataCollectionForSim
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:updateCameraState __updateCameraState) 0)
    ((:orientation_cam __orientation_cam) (instance geometry_msgs::Quaternion :init))
    ((:pos_cam __pos_cam) (instance geometry_msgs::Vector3 :init))
    ((:angular_velocity_imu __angular_velocity_imu) (instance geometry_msgs::Vector3 :init))
    ((:linear_acceleration_imu __linear_acceleration_imu) (instance geometry_msgs::Vector3 :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _updateCameraState (round __updateCameraState))
   (setq _orientation_cam __orientation_cam)
   (setq _pos_cam __pos_cam)
   (setq _angular_velocity_imu __angular_velocity_imu)
   (setq _linear_acceleration_imu __linear_acceleration_imu)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:updateCameraState
   (&optional __updateCameraState)
   (if __updateCameraState (setq _updateCameraState __updateCameraState)) _updateCameraState)
  (:orientation_cam
   (&rest __orientation_cam)
   (if (keywordp (car __orientation_cam))
       (send* _orientation_cam __orientation_cam)
     (progn
       (if __orientation_cam (setq _orientation_cam (car __orientation_cam)))
       _orientation_cam)))
  (:pos_cam
   (&rest __pos_cam)
   (if (keywordp (car __pos_cam))
       (send* _pos_cam __pos_cam)
     (progn
       (if __pos_cam (setq _pos_cam (car __pos_cam)))
       _pos_cam)))
  (:angular_velocity_imu
   (&rest __angular_velocity_imu)
   (if (keywordp (car __angular_velocity_imu))
       (send* _angular_velocity_imu __angular_velocity_imu)
     (progn
       (if __angular_velocity_imu (setq _angular_velocity_imu (car __angular_velocity_imu)))
       _angular_velocity_imu)))
  (:linear_acceleration_imu
   (&rest __linear_acceleration_imu)
   (if (keywordp (car __linear_acceleration_imu))
       (send* _linear_acceleration_imu __linear_acceleration_imu)
     (progn
       (if __linear_acceleration_imu (setq _linear_acceleration_imu (car __linear_acceleration_imu)))
       _linear_acceleration_imu)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int32 _updateCameraState
    4
    ;; geometry_msgs/Quaternion _orientation_cam
    (send _orientation_cam :serialization-length)
    ;; geometry_msgs/Vector3 _pos_cam
    (send _pos_cam :serialization-length)
    ;; geometry_msgs/Vector3 _angular_velocity_imu
    (send _angular_velocity_imu :serialization-length)
    ;; geometry_msgs/Vector3 _linear_acceleration_imu
    (send _linear_acceleration_imu :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int32 _updateCameraState
       (write-long _updateCameraState s)
     ;; geometry_msgs/Quaternion _orientation_cam
       (send _orientation_cam :serialize s)
     ;; geometry_msgs/Vector3 _pos_cam
       (send _pos_cam :serialize s)
     ;; geometry_msgs/Vector3 _angular_velocity_imu
       (send _angular_velocity_imu :serialize s)
     ;; geometry_msgs/Vector3 _linear_acceleration_imu
       (send _linear_acceleration_imu :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int32 _updateCameraState
     (setq _updateCameraState (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; geometry_msgs/Quaternion _orientation_cam
     (send _orientation_cam :deserialize buf ptr-) (incf ptr- (send _orientation_cam :serialization-length))
   ;; geometry_msgs/Vector3 _pos_cam
     (send _pos_cam :deserialize buf ptr-) (incf ptr- (send _pos_cam :serialization-length))
   ;; geometry_msgs/Vector3 _angular_velocity_imu
     (send _angular_velocity_imu :deserialize buf ptr-) (incf ptr- (send _angular_velocity_imu :serialization-length))
   ;; geometry_msgs/Vector3 _linear_acceleration_imu
     (send _linear_acceleration_imu :deserialize buf ptr-) (incf ptr- (send _linear_acceleration_imu :serialization-length))
   ;;
   self)
  )

(setf (get pose_est_new::DataCollectionForSim :md5sum-) "ef7c30dc1047a86e342d27ddf8805527")
(setf (get pose_est_new::DataCollectionForSim :datatype-) "pose_est_new/DataCollectionForSim")
(setf (get pose_est_new::DataCollectionForSim :definition-)
      "std_msgs/Header header
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
")



(provide :pose_est_new/DataCollectionForSim "ef7c30dc1047a86e342d27ddf8805527")


