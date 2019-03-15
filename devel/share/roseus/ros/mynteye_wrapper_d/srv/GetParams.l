;; Auto-generated. Do not edit!


(when (boundp 'mynteye_wrapper_d::GetParams)
  (if (not (find-package "MYNTEYE_WRAPPER_D"))
    (make-package "MYNTEYE_WRAPPER_D"))
  (shadow 'GetParams (find-package "MYNTEYE_WRAPPER_D")))
(unless (find-package "MYNTEYE_WRAPPER_D::GETPARAMS")
  (make-package "MYNTEYE_WRAPPER_D::GETPARAMS"))
(unless (find-package "MYNTEYE_WRAPPER_D::GETPARAMSREQUEST")
  (make-package "MYNTEYE_WRAPPER_D::GETPARAMSREQUEST"))
(unless (find-package "MYNTEYE_WRAPPER_D::GETPARAMSRESPONSE")
  (make-package "MYNTEYE_WRAPPER_D::GETPARAMSRESPONSE"))

(in-package "ROS")





(intern "*IMG_INTRINSICS*" (find-package "MYNTEYE_WRAPPER_D::GETPARAMSREQUEST"))
(shadow '*IMG_INTRINSICS* (find-package "MYNTEYE_WRAPPER_D::GETPARAMSREQUEST"))
(defconstant mynteye_wrapper_d::GetParamsRequest::*IMG_INTRINSICS* 0)
(intern "*IMG_EXTRINSICS_RTOL*" (find-package "MYNTEYE_WRAPPER_D::GETPARAMSREQUEST"))
(shadow '*IMG_EXTRINSICS_RTOL* (find-package "MYNTEYE_WRAPPER_D::GETPARAMSREQUEST"))
(defconstant mynteye_wrapper_d::GetParamsRequest::*IMG_EXTRINSICS_RTOL* 1)
(intern "*IMU_INTRINSICS*" (find-package "MYNTEYE_WRAPPER_D::GETPARAMSREQUEST"))
(shadow '*IMU_INTRINSICS* (find-package "MYNTEYE_WRAPPER_D::GETPARAMSREQUEST"))
(defconstant mynteye_wrapper_d::GetParamsRequest::*IMU_INTRINSICS* 2)
(intern "*IMU_EXTRINSICS*" (find-package "MYNTEYE_WRAPPER_D::GETPARAMSREQUEST"))
(shadow '*IMU_EXTRINSICS* (find-package "MYNTEYE_WRAPPER_D::GETPARAMSREQUEST"))
(defconstant mynteye_wrapper_d::GetParamsRequest::*IMU_EXTRINSICS* 3)
(defclass mynteye_wrapper_d::GetParamsRequest
  :super ros::object
  :slots (_key ))

(defmethod mynteye_wrapper_d::GetParamsRequest
  (:init
   (&key
    ((:key __key) 0)
    )
   (send-super :init)
   (setq _key (round __key))
   self)
  (:key
   (&optional __key)
   (if __key (setq _key __key)) _key)
  (:serialization-length
   ()
   (+
    ;; uint32 _key
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _key
       (write-long _key s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _key
     (setq _key (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass mynteye_wrapper_d::GetParamsResponse
  :super ros::object
  :slots (_value ))

(defmethod mynteye_wrapper_d::GetParamsResponse
  (:init
   (&key
    ((:value __value) "")
    )
   (send-super :init)
   (setq _value (string __value))
   self)
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
  (:serialization-length
   ()
   (+
    ;; string _value
    4 (length _value)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _value
       (write-long (length _value) s) (princ _value s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _value
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _value (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass mynteye_wrapper_d::GetParams
  :super ros::object
  :slots ())

(setf (get mynteye_wrapper_d::GetParams :md5sum-) "27490e4b5cf4d32a761bad9cafd48f69")
(setf (get mynteye_wrapper_d::GetParams :datatype-) "mynteye_wrapper_d/GetParams")
(setf (get mynteye_wrapper_d::GetParams :request) mynteye_wrapper_d::GetParamsRequest)
(setf (get mynteye_wrapper_d::GetParams :response) mynteye_wrapper_d::GetParamsResponse)

(defmethod mynteye_wrapper_d::GetParamsRequest
  (:response () (instance mynteye_wrapper_d::GetParamsResponse :init)))

(setf (get mynteye_wrapper_d::GetParamsRequest :md5sum-) "27490e4b5cf4d32a761bad9cafd48f69")
(setf (get mynteye_wrapper_d::GetParamsRequest :datatype-) "mynteye_wrapper_d/GetParamsRequest")
(setf (get mynteye_wrapper_d::GetParamsRequest :definition-)
      "uint32 IMG_INTRINSICS=0
uint32 IMG_EXTRINSICS_RTOL=1
uint32 IMU_INTRINSICS=2
uint32 IMU_EXTRINSICS=3
uint32 key
---
string value

")

(setf (get mynteye_wrapper_d::GetParamsResponse :md5sum-) "27490e4b5cf4d32a761bad9cafd48f69")
(setf (get mynteye_wrapper_d::GetParamsResponse :datatype-) "mynteye_wrapper_d/GetParamsResponse")
(setf (get mynteye_wrapper_d::GetParamsResponse :definition-)
      "uint32 IMG_INTRINSICS=0
uint32 IMG_EXTRINSICS_RTOL=1
uint32 IMU_INTRINSICS=2
uint32 IMU_EXTRINSICS=3
uint32 key
---
string value

")



(provide :mynteye_wrapper_d/GetParams "27490e4b5cf4d32a761bad9cafd48f69")


