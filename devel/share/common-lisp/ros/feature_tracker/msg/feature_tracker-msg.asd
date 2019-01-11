
(cl:in-package :asdf)

(defsystem "feature_tracker-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "CameraTrackerResult" :depends-on ("_package_CameraTrackerResult"))
    (:file "_package_CameraTrackerResult" :depends-on ("_package"))
    (:file "FeatureTrackerResult" :depends-on ("_package_FeatureTrackerResult"))
    (:file "_package_FeatureTrackerResult" :depends-on ("_package"))
  ))