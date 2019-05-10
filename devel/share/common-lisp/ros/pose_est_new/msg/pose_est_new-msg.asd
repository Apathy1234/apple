
(cl:in-package :asdf)

(defsystem "pose_est_new-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "CameraState" :depends-on ("_package_CameraState"))
    (:file "_package_CameraState" :depends-on ("_package"))
    (:file "DataCollectionForSim" :depends-on ("_package_DataCollectionForSim"))
    (:file "_package_DataCollectionForSim" :depends-on ("_package"))
  ))