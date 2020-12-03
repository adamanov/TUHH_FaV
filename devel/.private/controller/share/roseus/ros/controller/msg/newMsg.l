;; Auto-generated. Do not edit!


(when (boundp 'controller::newMsg)
  (if (not (find-package "CONTROLLER"))
    (make-package "CONTROLLER"))
  (shadow 'newMsg (find-package "CONTROLLER")))
(unless (find-package "CONTROLLER::NEWMSG")
  (make-package "CONTROLLER::NEWMSG"))

(in-package "ROS")
;;//! \htmlinclude newMsg.msg.html


(defclass controller::newMsg
  :super ros::object
  :slots ())

(defmethod controller::newMsg
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(setf (get controller::newMsg :md5sum-) "d41d8cd98f00b204e9800998ecf8427e")
(setf (get controller::newMsg :datatype-) "controller/newMsg")
(setf (get controller::newMsg :definition-)
      "
")



(provide :controller/newMsg "d41d8cd98f00b204e9800998ecf8427e")


