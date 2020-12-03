;; Auto-generated. Do not edit!


(when (boundp 'controller_main::Num)
  (if (not (find-package "CONTROLLER_MAIN"))
    (make-package "CONTROLLER_MAIN"))
  (shadow 'Num (find-package "CONTROLLER_MAIN")))
(unless (find-package "CONTROLLER_MAIN::NUM")
  (make-package "CONTROLLER_MAIN::NUM"))

(in-package "ROS")
;;//! \htmlinclude Num.msg.html


(defclass controller_main::Num
  :super ros::object
  :slots (_px _py _pz _yaw _pitch _roll ))

(defmethod controller_main::Num
  (:init
   (&key
    ((:px __px) 0)
    ((:py __py) 0)
    ((:pz __pz) 0)
    ((:yaw __yaw) 0)
    ((:pitch __pitch) 0)
    ((:roll __roll) 0)
    )
   (send-super :init)
   (setq _px (round __px))
   (setq _py (round __py))
   (setq _pz (round __pz))
   (setq _yaw (round __yaw))
   (setq _pitch (round __pitch))
   (setq _roll (round __roll))
   self)
  (:px
   (&optional __px)
   (if __px (setq _px __px)) _px)
  (:py
   (&optional __py)
   (if __py (setq _py __py)) _py)
  (:pz
   (&optional __pz)
   (if __pz (setq _pz __pz)) _pz)
  (:yaw
   (&optional __yaw)
   (if __yaw (setq _yaw __yaw)) _yaw)
  (:pitch
   (&optional __pitch)
   (if __pitch (setq _pitch __pitch)) _pitch)
  (:roll
   (&optional __roll)
   (if __roll (setq _roll __roll)) _roll)
  (:serialization-length
   ()
   (+
    ;; int64 _px
    8
    ;; int64 _py
    8
    ;; int64 _pz
    8
    ;; int64 _yaw
    8
    ;; int64 _pitch
    8
    ;; int64 _roll
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _px
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _px (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _px) (= (length (_px . bv)) 2)) ;; bignum
              (write-long (ash (elt (_px . bv) 0) 0) s)
              (write-long (ash (elt (_px . bv) 1) -1) s))
             ((and (class _px) (= (length (_px . bv)) 1)) ;; big1
              (write-long (elt (_px . bv) 0) s)
              (write-long (if (>= _px 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _px s)(write-long (if (>= _px 0) 0 #xffffffff) s)))
     ;; int64 _py
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _py (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _py) (= (length (_py . bv)) 2)) ;; bignum
              (write-long (ash (elt (_py . bv) 0) 0) s)
              (write-long (ash (elt (_py . bv) 1) -1) s))
             ((and (class _py) (= (length (_py . bv)) 1)) ;; big1
              (write-long (elt (_py . bv) 0) s)
              (write-long (if (>= _py 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _py s)(write-long (if (>= _py 0) 0 #xffffffff) s)))
     ;; int64 _pz
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _pz (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _pz) (= (length (_pz . bv)) 2)) ;; bignum
              (write-long (ash (elt (_pz . bv) 0) 0) s)
              (write-long (ash (elt (_pz . bv) 1) -1) s))
             ((and (class _pz) (= (length (_pz . bv)) 1)) ;; big1
              (write-long (elt (_pz . bv) 0) s)
              (write-long (if (>= _pz 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _pz s)(write-long (if (>= _pz 0) 0 #xffffffff) s)))
     ;; int64 _yaw
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _yaw (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _yaw) (= (length (_yaw . bv)) 2)) ;; bignum
              (write-long (ash (elt (_yaw . bv) 0) 0) s)
              (write-long (ash (elt (_yaw . bv) 1) -1) s))
             ((and (class _yaw) (= (length (_yaw . bv)) 1)) ;; big1
              (write-long (elt (_yaw . bv) 0) s)
              (write-long (if (>= _yaw 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _yaw s)(write-long (if (>= _yaw 0) 0 #xffffffff) s)))
     ;; int64 _pitch
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _pitch (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _pitch) (= (length (_pitch . bv)) 2)) ;; bignum
              (write-long (ash (elt (_pitch . bv) 0) 0) s)
              (write-long (ash (elt (_pitch . bv) 1) -1) s))
             ((and (class _pitch) (= (length (_pitch . bv)) 1)) ;; big1
              (write-long (elt (_pitch . bv) 0) s)
              (write-long (if (>= _pitch 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _pitch s)(write-long (if (>= _pitch 0) 0 #xffffffff) s)))
     ;; int64 _roll
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _roll (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _roll) (= (length (_roll . bv)) 2)) ;; bignum
              (write-long (ash (elt (_roll . bv) 0) 0) s)
              (write-long (ash (elt (_roll . bv) 1) -1) s))
             ((and (class _roll) (= (length (_roll . bv)) 1)) ;; big1
              (write-long (elt (_roll . bv) 0) s)
              (write-long (if (>= _roll 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _roll s)(write-long (if (>= _roll 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _px
#+(or :alpha :irix6 :x86_64)
      (setf _px (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _px (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _py
#+(or :alpha :irix6 :x86_64)
      (setf _py (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _py (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _pz
#+(or :alpha :irix6 :x86_64)
      (setf _pz (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _pz (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _yaw
#+(or :alpha :irix6 :x86_64)
      (setf _yaw (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _yaw (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _pitch
#+(or :alpha :irix6 :x86_64)
      (setf _pitch (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _pitch (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _roll
#+(or :alpha :irix6 :x86_64)
      (setf _roll (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _roll (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(setf (get controller_main::Num :md5sum-) "c1a5eba2c46b09a0f5a99beaba4b724f")
(setf (get controller_main::Num :datatype-) "controller_main/Num")
(setf (get controller_main::Num :definition-)
      "int64 px
int64 py
int64 pz

int64 yaw
int64 pitch
int64 roll





")



(provide :controller_main/Num "c1a5eba2c46b09a0f5a99beaba4b724f")


