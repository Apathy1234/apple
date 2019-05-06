
(cl:in-package :asdf)

(defsystem "pose_est_new-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "CamreaState" :depends-on ("_package_CamreaState"))
    (:file "_package_CamreaState" :depends-on ("_package"))
  ))