;; Auto-generated. Do not edit!


(when (boundp 'mavros_msgs::MotorSetpoint)
  (if (not (find-package "MAVROS_MSGS"))
    (make-package "MAVROS_MSGS"))
  (shadow 'MotorSetpoint (find-package "MAVROS_MSGS")))
(unless (find-package "MAVROS_MSGS::MOTORSETPOINT")
  (make-package "MAVROS_MSGS::MOTORSETPOINT"))

(in-package "ROS")
;;//! \htmlinclude MotorSetpoint.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass mavros_msgs::MotorSetpoint
  :super ros::object
  :slots (_header _setpoint ))

(defmethod mavros_msgs::MotorSetpoint
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:setpoint __setpoint) (make-array 8 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _setpoint __setpoint)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:setpoint
   (&optional __setpoint)
   (if __setpoint (setq _setpoint __setpoint)) _setpoint)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32[8] _setpoint
    (* 4    8)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32[8] _setpoint
     (dotimes (i 8)
       (sys::poke (elt _setpoint i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32[8] _setpoint
   (dotimes (i (length _setpoint))
     (setf (elt _setpoint i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;;
   self)
  )

(setf (get mavros_msgs::MotorSetpoint :md5sum-) "8b3cd59674ada67977a971a5213a7739")
(setf (get mavros_msgs::MotorSetpoint :datatype-) "mavros_msgs/MotorSetpoint")
(setf (get mavros_msgs::MotorSetpoint :definition-)
      "std_msgs/Header header
# motor throttle value [-1..1]
float32[8] setpoint
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :mavros_msgs/MotorSetpoint "8b3cd59674ada67977a971a5213a7739")


