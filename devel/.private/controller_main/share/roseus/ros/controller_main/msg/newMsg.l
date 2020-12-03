;; Auto-generated. Do not edit!


(when (boundp 'controller_main::newMsg)
  (if (not (find-package "CONTROLLER_MAIN"))
    (make-package "CONTROLLER_MAIN"))
  (shadow 'newMsg (find-package "CONTROLLER_MAIN")))
(unless (find-package "CONTROLLER_MAIN::NEWMSG")
  (make-package "CONTROLLER_MAIN::NEWMSG"))

(in-package "ROS")
;;//! \htmlinclude newMsg.msg.html


(defclass controller_main::newMsg
  :super ros::object
  :slots ())

(defmethod controller_main::newMsg
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

(setf (get controller_main::newMsg :md5sum-) "d41d8cd98f00b204e9800998ecf8427e")
(setf (get controller_main::newMsg :datatype-) "controller_main/newMsg")
(setf (get controller_main::newMsg :definition-)
      "
")



(provide :controller_main/newMsg "d41d8cd98f00b204e9800998ecf8427e")


