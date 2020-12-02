
(cl:in-package :asdf)

(defsystem "depth_controller-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ActuatorCommands" :depends-on ("_package_ActuatorCommands"))
    (:file "_package_ActuatorCommands" :depends-on ("_package"))
  ))