
(cl:in-package :asdf)

(defsystem "range_sensor-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "RangeMeasurement" :depends-on ("_package_RangeMeasurement"))
    (:file "_package_RangeMeasurement" :depends-on ("_package"))
    (:file "RangeMeasurementArray" :depends-on ("_package_RangeMeasurementArray"))
    (:file "_package_RangeMeasurementArray" :depends-on ("_package"))
  ))