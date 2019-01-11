;; Auto-generated. Do not edit!


(when (boundp 'feature_tracker::FeatureTrackerResule)
  (if (not (find-package "FEATURE_TRACKER"))
    (make-package "FEATURE_TRACKER"))
  (shadow 'FeatureTrackerResule (find-package "FEATURE_TRACKER")))
(unless (find-package "FEATURE_TRACKER::FEATURETRACKERRESULE")
  (make-package "FEATURE_TRACKER::FEATURETRACKERRESULE"))

(in-package "ROS")
;;//! \htmlinclude FeatureTrackerResule.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass feature_tracker::FeatureTrackerResule
  :super ros::object
  :slots (_Header _id _lifeTime _u0 _v0 _u1 _v1 ))

(defmethod feature_tracker::FeatureTrackerResule
  (:init
   (&key
    ((:Header __Header) (instance std_msgs::Header :init))
    ((:id __id) 0)
    ((:lifeTime __lifeTime) 0)
    ((:u0 __u0) 0.0)
    ((:v0 __v0) 0.0)
    ((:u1 __u1) 0.0)
    ((:v1 __v1) 0.0)
    )
   (send-super :init)
   (setq _Header __Header)
   (setq _id (round __id))
   (setq _lifeTime (round __lifeTime))
   (setq _u0 (float __u0))
   (setq _v0 (float __v0))
   (setq _u1 (float __u1))
   (setq _v1 (float __v1))
   self)
  (:Header
   (&rest __Header)
   (if (keywordp (car __Header))
       (send* _Header __Header)
     (progn
       (if __Header (setq _Header (car __Header)))
       _Header)))
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:lifeTime
   (&optional __lifeTime)
   (if __lifeTime (setq _lifeTime __lifeTime)) _lifeTime)
  (:u0
   (&optional __u0)
   (if __u0 (setq _u0 __u0)) _u0)
  (:v0
   (&optional __v0)
   (if __v0 (setq _v0 __v0)) _v0)
  (:u1
   (&optional __u1)
   (if __u1 (setq _u1 __u1)) _u1)
  (:v1
   (&optional __v1)
   (if __v1 (setq _v1 __v1)) _v1)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _Header
    (send _Header :serialization-length)
    ;; int64 _id
    8
    ;; int64 _lifeTime
    8
    ;; float64 _u0
    8
    ;; float64 _v0
    8
    ;; float64 _u1
    8
    ;; float64 _v1
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _Header
       (send _Header :serialize s)
     ;; int64 _id
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _id (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _id) (= (length (_id . bv)) 2)) ;; bignum
              (write-long (ash (elt (_id . bv) 0) 0) s)
              (write-long (ash (elt (_id . bv) 1) -1) s))
             ((and (class _id) (= (length (_id . bv)) 1)) ;; big1
              (write-long (elt (_id . bv) 0) s)
              (write-long (if (>= _id 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _id s)(write-long (if (>= _id 0) 0 #xffffffff) s)))
     ;; int64 _lifeTime
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _lifeTime (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _lifeTime) (= (length (_lifeTime . bv)) 2)) ;; bignum
              (write-long (ash (elt (_lifeTime . bv) 0) 0) s)
              (write-long (ash (elt (_lifeTime . bv) 1) -1) s))
             ((and (class _lifeTime) (= (length (_lifeTime . bv)) 1)) ;; big1
              (write-long (elt (_lifeTime . bv) 0) s)
              (write-long (if (>= _lifeTime 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _lifeTime s)(write-long (if (>= _lifeTime 0) 0 #xffffffff) s)))
     ;; float64 _u0
       (sys::poke _u0 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _v0
       (sys::poke _v0 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _u1
       (sys::poke _u1 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _v1
       (sys::poke _v1 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _Header
     (send _Header :deserialize buf ptr-) (incf ptr- (send _Header :serialization-length))
   ;; int64 _id
#+(or :alpha :irix6 :x86_64)
      (setf _id (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _id (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _lifeTime
#+(or :alpha :irix6 :x86_64)
      (setf _lifeTime (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _lifeTime (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; float64 _u0
     (setq _u0 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _v0
     (setq _v0 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _u1
     (setq _u1 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _v1
     (setq _v1 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get feature_tracker::FeatureTrackerResule :md5sum-) "ab82e194225e796f26c8c2c76b2bd2bb")
(setf (get feature_tracker::FeatureTrackerResule :datatype-) "feature_tracker/FeatureTrackerResule")
(setf (get feature_tracker::FeatureTrackerResule :definition-)
      "std_msgs/Header Header
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

")



(provide :feature_tracker/FeatureTrackerResule "ab82e194225e796f26c8c2c76b2bd2bb")


