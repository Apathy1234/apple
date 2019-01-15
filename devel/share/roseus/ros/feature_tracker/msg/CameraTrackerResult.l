;; Auto-generated. Do not edit!


(when (boundp 'feature_tracker::CameraTrackerResult)
  (if (not (find-package "FEATURE_TRACKER"))
    (make-package "FEATURE_TRACKER"))
  (shadow 'CameraTrackerResult (find-package "FEATURE_TRACKER")))
(unless (find-package "FEATURE_TRACKER::CAMERATRACKERRESULT")
  (make-package "FEATURE_TRACKER::CAMERATRACKERRESULT"))

(in-package "ROS")
;;//! \htmlinclude CameraTrackerResult.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass feature_tracker::CameraTrackerResult
  :super ros::object
  :slots (_header _num_of_features _features ))

(defmethod feature_tracker::CameraTrackerResult
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:num_of_features __num_of_features) 0)
    ((:features __features) (let (r) (dotimes (i 0) (push (instance feature_tracker::FeatureTrackerResult :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _num_of_features (round __num_of_features))
   (setq _features __features)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:num_of_features
   (&optional __num_of_features)
   (if __num_of_features (setq _num_of_features __num_of_features)) _num_of_features)
  (:features
   (&rest __features)
   (if (keywordp (car __features))
       (send* _features __features)
     (progn
       (if __features (setq _features (car __features)))
       _features)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int64 _num_of_features
    8
    ;; feature_tracker/FeatureTrackerResult[] _features
    (apply #'+ (send-all _features :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int64 _num_of_features
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _num_of_features (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _num_of_features) (= (length (_num_of_features . bv)) 2)) ;; bignum
              (write-long (ash (elt (_num_of_features . bv) 0) 0) s)
              (write-long (ash (elt (_num_of_features . bv) 1) -1) s))
             ((and (class _num_of_features) (= (length (_num_of_features . bv)) 1)) ;; big1
              (write-long (elt (_num_of_features . bv) 0) s)
              (write-long (if (>= _num_of_features 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _num_of_features s)(write-long (if (>= _num_of_features 0) 0 #xffffffff) s)))
     ;; feature_tracker/FeatureTrackerResult[] _features
     (write-long (length _features) s)
     (dolist (elem _features)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int64 _num_of_features
#+(or :alpha :irix6 :x86_64)
      (setf _num_of_features (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _num_of_features (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; feature_tracker/FeatureTrackerResult[] _features
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _features (let (r) (dotimes (i n) (push (instance feature_tracker::FeatureTrackerResult :init) r)) r))
     (dolist (elem- _features)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get feature_tracker::CameraTrackerResult :md5sum-) "d039766b47fa8dfad093dc64535488f0")
(setf (get feature_tracker::CameraTrackerResult :datatype-) "feature_tracker/CameraTrackerResult")
(setf (get feature_tracker::CameraTrackerResult :definition-)
      "std_msgs/Header header
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
float64 z
")



(provide :feature_tracker/CameraTrackerResult "d039766b47fa8dfad093dc64535488f0")


