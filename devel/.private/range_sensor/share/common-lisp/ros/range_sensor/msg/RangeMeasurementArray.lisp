; Auto-generated. Do not edit!


(cl:in-package range_sensor-msg)


;//! \htmlinclude RangeMeasurementArray.msg.html

(cl:defclass <RangeMeasurementArray> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (measurements
    :reader measurements
    :initarg :measurements
    :type (cl:vector range_sensor-msg:RangeMeasurement)
   :initform (cl:make-array 0 :element-type 'range_sensor-msg:RangeMeasurement :initial-element (cl:make-instance 'range_sensor-msg:RangeMeasurement))))
)

(cl:defclass RangeMeasurementArray (<RangeMeasurementArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RangeMeasurementArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RangeMeasurementArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name range_sensor-msg:<RangeMeasurementArray> is deprecated: use range_sensor-msg:RangeMeasurementArray instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <RangeMeasurementArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader range_sensor-msg:header-val is deprecated.  Use range_sensor-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'measurements-val :lambda-list '(m))
(cl:defmethod measurements-val ((m <RangeMeasurementArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader range_sensor-msg:measurements-val is deprecated.  Use range_sensor-msg:measurements instead.")
  (measurements m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RangeMeasurementArray>) ostream)
  "Serializes a message object of type '<RangeMeasurementArray>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'measurements))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'measurements))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RangeMeasurementArray>) istream)
  "Deserializes a message object of type '<RangeMeasurementArray>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'measurements) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'measurements)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'range_sensor-msg:RangeMeasurement))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RangeMeasurementArray>)))
  "Returns string type for a message object of type '<RangeMeasurementArray>"
  "range_sensor/RangeMeasurementArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RangeMeasurementArray)))
  "Returns string type for a message object of type 'RangeMeasurementArray"
  "range_sensor/RangeMeasurementArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RangeMeasurementArray>)))
  "Returns md5sum for a message object of type '<RangeMeasurementArray>"
  "f973ebf7b330759b20c6280d292daea9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RangeMeasurementArray)))
  "Returns md5sum for a message object of type 'RangeMeasurementArray"
  "f973ebf7b330759b20c6280d292daea9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RangeMeasurementArray>)))
  "Returns full string definition for message of type '<RangeMeasurementArray>"
  (cl:format cl:nil "std_msgs/Header header~%RangeMeasurement[] measurements~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: range_sensor/RangeMeasurement~%std_msgs/Header header~%~%int32 id       # ID of anchor~%float64 range  # distance to anchor~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RangeMeasurementArray)))
  "Returns full string definition for message of type 'RangeMeasurementArray"
  (cl:format cl:nil "std_msgs/Header header~%RangeMeasurement[] measurements~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: range_sensor/RangeMeasurement~%std_msgs/Header header~%~%int32 id       # ID of anchor~%float64 range  # distance to anchor~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RangeMeasurementArray>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'measurements) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RangeMeasurementArray>))
  "Converts a ROS message object to a list"
  (cl:list 'RangeMeasurementArray
    (cl:cons ':header (header msg))
    (cl:cons ':measurements (measurements msg))
))
