; Auto-generated. Do not edit!


(cl:in-package controller-msg)


;//! \htmlinclude newMsg.msg.html

(cl:defclass <newMsg> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass newMsg (<newMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <newMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'newMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name controller-msg:<newMsg> is deprecated: use controller-msg:newMsg instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <newMsg>) ostream)
  "Serializes a message object of type '<newMsg>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <newMsg>) istream)
  "Deserializes a message object of type '<newMsg>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<newMsg>)))
  "Returns string type for a message object of type '<newMsg>"
  "controller/newMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'newMsg)))
  "Returns string type for a message object of type 'newMsg"
  "controller/newMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<newMsg>)))
  "Returns md5sum for a message object of type '<newMsg>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'newMsg)))
  "Returns md5sum for a message object of type 'newMsg"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<newMsg>)))
  "Returns full string definition for message of type '<newMsg>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'newMsg)))
  "Returns full string definition for message of type 'newMsg"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <newMsg>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <newMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'newMsg
))
