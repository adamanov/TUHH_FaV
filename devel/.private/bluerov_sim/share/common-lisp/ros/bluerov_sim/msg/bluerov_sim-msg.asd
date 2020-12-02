
(cl:in-package :asdf)

(defsystem "bluerov_sim-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ActuatorCommands" :depends-on ("_package_ActuatorCommands"))
    (:file "_package_ActuatorCommands" :depends-on ("_package"))
  ))