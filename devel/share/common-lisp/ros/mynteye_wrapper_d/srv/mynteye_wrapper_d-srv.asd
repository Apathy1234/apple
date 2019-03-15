
(cl:in-package :asdf)

(defsystem "mynteye_wrapper_d-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "GetParams" :depends-on ("_package_GetParams"))
    (:file "_package_GetParams" :depends-on ("_package"))
  ))