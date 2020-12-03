;; Auto-generated. Do not edit!


(when (boundp 'bluerov_sim::ActuatorCommands)
  (if (not (find-package "BLUEROV_SIM"))
    (make-package "BLUEROV_SIM"))
  (shadow 'ActuatorCommands (find-package "BLUEROV_SIM")))
(unless (find-package "BLUEROV_SIM::ACTUATORCOMMANDS")
  (make-package "BLUEROV_SIM::ACTUATORCOMMANDS"))

(in-package "ROS")
;;//! \htmlinclude ActuatorCommands.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass bluerov_sim::ActuatorCommands
  :super ros::object
  :slots (_header _roll _pitch _yaw _thrust _lateral_thrust _vertical_thrust ))

(defmethod bluerov_sim::ActuatorCommands
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:roll __roll) 0.0)
    ((:pitch __pitch) 0.0)
    ((:yaw __yaw) 0.0)
    ((:thrust __thrust) 0.0)
    ((:lateral_thrust __lateral_thrust) 0.0)
    ((:vertical_thrust __vertical_thrust) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _roll (float __roll))
   (setq _pitch (float __pitch))
   (setq _yaw (float __yaw))
   (setq _thrust (float __thrust))
   (setq _lateral_thrust (float __lateral_thrust))
   (setq _vertical_thrust (float __vertical_thrust))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:roll
   (&optional __roll)
   (if __roll (setq _roll __roll)) _roll)
  (:pitch
   (&optional __pitch)
   (if __pitch (setq _pitch __pitch)) _pitch)
  (:yaw
   (&optional __yaw)
   (if __yaw (setq _yaw __yaw)) _yaw)
  (:thrust
   (&optional __thrust)
   (if __thrust (setq _thrust __thrust)) _thrust)
  (:lateral_thrust
   (&optional __lateral_thrust)
   (if __lateral_thrust (setq _lateral_thrust __lateral_thrust)) _lateral_thrust)
  (:vertical_thrust
   (&optional __vertical_thrust)
   (if __vertical_thrust (setq _vertical_thrust __vertical_thrust)) _vertical_thrust)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32 _roll
    4
    ;; float32 _pitch
    4
    ;; float32 _yaw
    4
    ;; float32 _thrust
    4
    ;; float32 _lateral_thrust
    4
    ;; float32 _vertical_thrust
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32 _roll
       (sys::poke _roll (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _pitch
       (sys::poke _pitch (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _yaw
       (sys::poke _yaw (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _thrust
       (sys::poke _thrust (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _lateral_thrust
       (sys::poke _lateral_thrust (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _vertical_thrust
       (sys::poke _vertical_thrust (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32 _roll
     (setq _roll (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _pitch
     (setq _pitch (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _yaw
     (setq _yaw (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _thrust
     (setq _thrust (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _lateral_thrust
     (setq _lateral_thrust (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _vertical_thrust
     (setq _vertical_thrust (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get bluerov_sim::ActuatorCommands :md5sum-) "6deaf3fbb15f1ee6b5555ba450666513")
(setf (get bluerov_sim::ActuatorCommands :datatype-) "bluerov_sim/ActuatorCommands")
(setf (get bluerov_sim::ActuatorCommands :definition-)
      "std_msgs/Header header

float32 roll
float32 pitch
float32 yaw
float32 thrust
float32 lateral_thrust
float32 vertical_thrust

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



(provide :bluerov_sim/ActuatorCommands "6deaf3fbb15f1ee6b5555ba450666513")


