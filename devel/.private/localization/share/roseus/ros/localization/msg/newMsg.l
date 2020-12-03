;; Auto-generated. Do not edit!


(when (boundp 'localization::newMsg)
  (if (not (find-package "LOCALIZATION"))
    (make-package "LOCALIZATION"))
  (shadow 'newMsg (find-package "LOCALIZATION")))
(unless (find-package "LOCALIZATION::NEWMSG")
  (make-package "LOCALIZATION::NEWMSG"))

(in-package "ROS")
;;//! \htmlinclude newMsg.msg.html


(defclass localization::newMsg
  :super ros::object
  :slots ())

(defmethod localization::newMsg
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

(setf (get localization::newMsg :md5sum-) "d41d8cd98f00b204e9800998ecf8427e")
(setf (get localization::newMsg :datatype-) "localization/newMsg")
(setf (get localization::newMsg :definition-)
      "
")



(provide :localization/newMsg "d41d8cd98f00b204e9800998ecf8427e")


