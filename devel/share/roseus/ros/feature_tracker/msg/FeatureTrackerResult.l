;; Auto-generated. Do not edit!


(when (boundp 'feature_tracker::FeatureTrackerResult)
  (if (not (find-package "FEATURE_TRACKER"))
    (make-package "FEATURE_TRACKER"))
  (shadow 'FeatureTrackerResult (find-package "FEATURE_TRACKER")))
(unless (find-package "FEATURE_TRACKER::FEATURETRACKERRESULT")
  (make-package "FEATURE_TRACKER::FEATURETRACKERRESULT"))

(in-package "ROS")
;;//! \htmlinclude FeatureTrackerResult.msg.html


(defclass feature_tracker::FeatureTrackerResult
  :super ros::object
  :slots (_id _cnt _seq _u0 _v0 _u1 _v1 _x _y _z ))

(defmethod feature_tracker::FeatureTrackerResult
  (:init
   (&key
    ((:id __id) 0)
    ((:cnt __cnt) 0)
    ((:seq __seq) 0)
    ((:u0 __u0) 0.0)
    ((:v0 __v0) 0.0)
    ((:u1 __u1) 0.0)
    ((:v1 __v1) 0.0)
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:z __z) 0.0)
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _cnt (round __cnt))
   (setq _seq (round __seq))
   (setq _u0 (float __u0))
   (setq _v0 (float __v0))
   (setq _u1 (float __u1))
   (setq _v1 (float __v1))
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _z (float __z))
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:cnt
   (&optional __cnt)
   (if __cnt (setq _cnt __cnt)) _cnt)
  (:seq
   (&optional __seq)
   (if __seq (setq _seq __seq)) _seq)
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
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:z
   (&optional __z)
   (if __z (setq _z __z)) _z)
  (:serialization-length
   ()
   (+
    ;; int64 _id
    8
    ;; int64 _cnt
    8
    ;; int64 _seq
    8
    ;; float64 _u0
    8
    ;; float64 _v0
    8
    ;; float64 _u1
    8
    ;; float64 _v1
    8
    ;; float64 _x
    8
    ;; float64 _y
    8
    ;; float64 _z
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
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
     ;; int64 _cnt
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _cnt (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _cnt) (= (length (_cnt . bv)) 2)) ;; bignum
              (write-long (ash (elt (_cnt . bv) 0) 0) s)
              (write-long (ash (elt (_cnt . bv) 1) -1) s))
             ((and (class _cnt) (= (length (_cnt . bv)) 1)) ;; big1
              (write-long (elt (_cnt . bv) 0) s)
              (write-long (if (>= _cnt 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _cnt s)(write-long (if (>= _cnt 0) 0 #xffffffff) s)))
     ;; int64 _seq
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _seq (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _seq) (= (length (_seq . bv)) 2)) ;; bignum
              (write-long (ash (elt (_seq . bv) 0) 0) s)
              (write-long (ash (elt (_seq . bv) 1) -1) s))
             ((and (class _seq) (= (length (_seq . bv)) 1)) ;; big1
              (write-long (elt (_seq . bv) 0) s)
              (write-long (if (>= _seq 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _seq s)(write-long (if (>= _seq 0) 0 #xffffffff) s)))
     ;; float64 _u0
       (sys::poke _u0 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _v0
       (sys::poke _v0 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _u1
       (sys::poke _u1 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _v1
       (sys::poke _v1 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _x
       (sys::poke _x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y
       (sys::poke _y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _z
       (sys::poke _z (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
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
   ;; int64 _cnt
#+(or :alpha :irix6 :x86_64)
      (setf _cnt (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _cnt (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _seq
#+(or :alpha :irix6 :x86_64)
      (setf _seq (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _seq (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
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
   ;; float64 _x
     (setq _x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y
     (setq _y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _z
     (setq _z (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get feature_tracker::FeatureTrackerResult :md5sum-) "ba9d4897b1e44abdddcf84f9f72d3eb6")
(setf (get feature_tracker::FeatureTrackerResult :datatype-) "feature_tracker/FeatureTrackerResult")
(setf (get feature_tracker::FeatureTrackerResult :definition-)
      "int64 id
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



(provide :feature_tracker/FeatureTrackerResult "ba9d4897b1e44abdddcf84f9f72d3eb6")


