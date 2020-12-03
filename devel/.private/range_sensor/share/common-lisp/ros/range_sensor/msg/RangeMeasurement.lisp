; Auto-generated. Do not edit!


(cl:in-package range_sensor-msg)


;//! \htmlinclude RangeMeasurement.msg.html

(cl:defclass <RangeMeasurement> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0)
   (range
    :reader range
    :initarg :range
    :type cl:float
    :initform 0.0))
)

(cl:defclass RangeMeasurement (<RangeMeasurement>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RangeMeasurement>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RangeMeasurement)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name range_sensor-msg:<RangeMeasurement> is deprecated: use range_sensor-msg:RangeMeasurement instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <RangeMeasurement>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader range_sensor-msg:header-val is deprecated.  Use range_sensor-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <RangeMeasurement>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader range_sensor-msg:id-val is deprecated.  Use range_sensor-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'range-val :lambda-list '(m))
(cl:defmethod range-val ((m <RangeMeasurement>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader range_sensor-msg:range-val is deprecated.  Use range_sensor-msg:range instead.")
  (range m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RangeMeasurement>) ostream)
  "Serializes a message object of type '<RangeMeasurement>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'range))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RangeMeasurement>) istream)
  "Deserializes a message object of type '<RangeMeasurement>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'range) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RangeMeasurement>)))
  "Returns string type for a message object of type '<RangeMeasurement>"
  "range_sensor/RangeMeasurement")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RangeMeasurement)))
  "Returns string type for a message object of type 'RangeMeasurement"
  "range_sensor/RangeMeasurement")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RangeMeasurement>)))
  "Returns md5sum for a message object of type '<RangeMeasurement>"
  "d36c99edc955a5c530e4f3ab55de9616")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RangeMeasurement)))
  "Returns md5sum for a message object of type 'RangeMeasurement"
  "d36c99edc955a5c530e4f3ab55de9616")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RangeMeasurement>)))
  "Returns full string definition for message of type '<RangeMeasurement>"
  (cl:format cl:nil "std_msgs/Header header~%~%int32 id       # ID of anchor~%float64 range  # distance to anchor~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RangeMeasurement)))
  "Returns full string definition for message of type 'RangeMeasurement"
  (cl:format cl:nil "std_msgs/Header header~%~%int32 id       # ID of anchor~%float64 range  # distance to anchor~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RangeMeasurement>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RangeMeasurement>))
  "Converts a ROS message object to a list"
  (cl:list 'RangeMeasurement
    (cl:cons ':header (header msg))
    (cl:cons ':id (id msg))
    (cl:cons ':range (range msg))
))
