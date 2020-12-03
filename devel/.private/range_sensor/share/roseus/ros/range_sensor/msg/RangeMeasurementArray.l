;; Auto-generated. Do not edit!


(when (boundp 'range_sensor::RangeMeasurementArray)
  (if (not (find-package "RANGE_SENSOR"))
    (make-package "RANGE_SENSOR"))
  (shadow 'RangeMeasurementArray (find-package "RANGE_SENSOR")))
(unless (find-package "RANGE_SENSOR::RANGEMEASUREMENTARRAY")
  (make-package "RANGE_SENSOR::RANGEMEASUREMENTARRAY"))

(in-package "ROS")
;;//! \htmlinclude RangeMeasurementArray.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass range_sensor::RangeMeasurementArray
  :super ros::object
  :slots (_header _measurements ))

(defmethod range_sensor::RangeMeasurementArray
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:measurements __measurements) (let (r) (dotimes (i 0) (push (instance range_sensor::RangeMeasurement :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _measurements __measurements)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:measurements
   (&rest __measurements)
   (if (keywordp (car __measurements))
       (send* _measurements __measurements)
     (progn
       (if __measurements (setq _measurements (car __measurements)))
       _measurements)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; range_sensor/RangeMeasurement[] _measurements
    (apply #'+ (send-all _measurements :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; range_sensor/RangeMeasurement[] _measurements
     (write-long (length _measurements) s)
     (dolist (elem _measurements)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; range_sensor/RangeMeasurement[] _measurements
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _measurements (let (r) (dotimes (i n) (push (instance range_sensor::RangeMeasurement :init) r)) r))
     (dolist (elem- _measurements)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get range_sensor::RangeMeasurementArray :md5sum-) "f973ebf7b330759b20c6280d292daea9")
(setf (get range_sensor::RangeMeasurementArray :datatype-) "range_sensor/RangeMeasurementArray")
(setf (get range_sensor::RangeMeasurementArray :definition-)
      "std_msgs/Header header
RangeMeasurement[] measurements

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

================================================================================
MSG: range_sensor/RangeMeasurement
std_msgs/Header header

int32 id       # ID of anchor
float64 range  # distance to anchor

")



(provide :range_sensor/RangeMeasurementArray "f973ebf7b330759b20c6280d292daea9")


