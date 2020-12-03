
(cl:in-package :asdf)

(defsystem "controller_main-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "currentPose" :depends-on ("_package_currentPose"))
    (:file "_package_currentPose" :depends-on ("_package"))
  ))