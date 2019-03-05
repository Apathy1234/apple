
(cl:in-package :asdf)

(defsystem "pose_estimate-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "PoseEstimateResult" :depends-on ("_package_PoseEstimateResult"))
    (:file "_package_PoseEstimateResult" :depends-on ("_package"))
  ))