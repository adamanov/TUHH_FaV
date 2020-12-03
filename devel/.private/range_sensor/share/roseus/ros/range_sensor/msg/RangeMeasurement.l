;; Auto-generated. Do not edit!


(when (boundp 'range_sensor::RangeMeasurement)
  (if (not (find-package "RANGE_SENSOR"))
    (make-package "RANGE_SENSOR"))
  (shadow 'RangeMeasurement (find-package "RANGE_SENSOR")))
(unless (find-package "RANGE_SENSOR::RANGEMEASUREMENT")
  (make-package "RANGE_SENSOR::RANGEMEASUREMENT"))

(in-package "ROS")
;;//! \htmlinclude RangeMeasurement.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass range_sensor::RangeMeasurement
  :super ros::object
  :slots (_header _id _range ))

(defmethod range_sensor::RangeMeasurement
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:id __id) 0)
    ((:range __range) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _id (round __id))
   (setq _range (float __range))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:range
   (&optional __range)
   (if __range (setq _range __range)) _range)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int32 _id
    4
    ;; float64 _range
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int32 _id
       (write-long _id s)
     ;; float64 _range
       (sys::poke _range (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64 _range
     (setq _range (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get range_sensor::RangeMeasurement :md5sum-) "d36c99edc955a5c530e4f3ab55de9616")
(setf (get range_sensor::RangeMeasurement :datatype-) "range_sensor/RangeMeasurement")
(setf (get range_sensor::RangeMeasurement :definition-)
      "std_msgs/Header header

int32 id       # ID of anchor
float64 range  # distance to anchor

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



(provide :range_sensor/RangeMeasurement "d36c99edc955a5c530e4f3ab55de9616")


