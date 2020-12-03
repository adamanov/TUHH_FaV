; Auto-generated. Do not edit!


(cl:in-package mavros_msgs-msg)


;//! \htmlinclude MotorSetpoint.msg.html

(cl:defclass <MotorSetpoint> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (setpoint
    :reader setpoint
    :initarg :setpoint
    :type (cl:vector cl:float)
   :initform (cl:make-array 8 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass MotorSetpoint (<MotorSetpoint>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MotorSetpoint>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MotorSetpoint)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mavros_msgs-msg:<MotorSetpoint> is deprecated: use mavros_msgs-msg:MotorSetpoint instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <MotorSetpoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-msg:header-val is deprecated.  Use mavros_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'setpoint-val :lambda-list '(m))
(cl:defmethod setpoint-val ((m <MotorSetpoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-msg:setpoint-val is deprecated.  Use mavros_msgs-msg:setpoint instead.")
  (setpoint m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MotorSetpoint>) ostream)
  "Serializes a message object of type '<MotorSetpoint>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'setpoint))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MotorSetpoint>) istream)
  "Deserializes a message object of type '<MotorSetpoint>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:setf (cl:slot-value msg 'setpoint) (cl:make-array 8))
  (cl:let ((vals (cl:slot-value msg 'setpoint)))
    (cl:dotimes (i 8)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MotorSetpoint>)))
  "Returns string type for a message object of type '<MotorSetpoint>"
  "mavros_msgs/MotorSetpoint")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MotorSetpoint)))
  "Returns string type for a message object of type 'MotorSetpoint"
  "mavros_msgs/MotorSetpoint")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MotorSetpoint>)))
  "Returns md5sum for a message object of type '<MotorSetpoint>"
  "8b3cd59674ada67977a971a5213a7739")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MotorSetpoint)))
  "Returns md5sum for a message object of type 'MotorSetpoint"
  "8b3cd59674ada67977a971a5213a7739")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MotorSetpoint>)))
  "Returns full string definition for message of type '<MotorSetpoint>"
  (cl:format cl:nil "std_msgs/Header header~%# motor throttle value [-1..1]~%float32[8] setpoint~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MotorSetpoint)))
  "Returns full string definition for message of type 'MotorSetpoint"
  (cl:format cl:nil "std_msgs/Header header~%# motor throttle value [-1..1]~%float32[8] setpoint~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MotorSetpoint>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'setpoint) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MotorSetpoint>))
  "Converts a ROS message object to a list"
  (cl:list 'MotorSetpoint
    (cl:cons ':header (header msg))
    (cl:cons ':setpoint (setpoint msg))
))
