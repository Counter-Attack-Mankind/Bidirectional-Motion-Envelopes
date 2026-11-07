;; Auto-generated. Do not edit!


(when (boundp 'sandbox_msgs::ChassisCommand)
  (if (not (find-package "SANDBOX_MSGS"))
    (make-package "SANDBOX_MSGS"))
  (shadow 'ChassisCommand (find-package "SANDBOX_MSGS")))
(unless (find-package "SANDBOX_MSGS::CHASSISCOMMAND")
  (make-package "SANDBOX_MSGS::CHASSISCOMMAND"))

(in-package "ROS")
;;//! \htmlinclude ChassisCommand.msg.html


(defclass sandbox_msgs::ChassisCommand
  :super ros::object
  :slots (_target _throttle _steering ))

(defmethod sandbox_msgs::ChassisCommand
  (:init
   (&key
    ((:target __target) 0)
    ((:throttle __throttle) 0.0)
    ((:steering __steering) 0.0)
    )
   (send-super :init)
   (setq _target (round __target))
   (setq _throttle (float __throttle))
   (setq _steering (float __steering))
   self)
  (:target
   (&optional __target)
   (if __target (setq _target __target)) _target)
  (:throttle
   (&optional __throttle)
   (if __throttle (setq _throttle __throttle)) _throttle)
  (:steering
   (&optional __steering)
   (if __steering (setq _steering __steering)) _steering)
  (:serialization-length
   ()
   (+
    ;; int32 _target
    4
    ;; float64 _throttle
    8
    ;; float64 _steering
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _target
       (write-long _target s)
     ;; float64 _throttle
       (sys::poke _throttle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _steering
       (sys::poke _steering (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _target
     (setq _target (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64 _throttle
     (setq _throttle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _steering
     (setq _steering (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get sandbox_msgs::ChassisCommand :md5sum-) "fc19350b070ac8db9efdde1c244e917a")
(setf (get sandbox_msgs::ChassisCommand :datatype-) "sandbox_msgs/ChassisCommand")
(setf (get sandbox_msgs::ChassisCommand :definition-)
      "int32 target

float64 throttle
float64 steering

")



(provide :sandbox_msgs/ChassisCommand "fc19350b070ac8db9efdde1c244e917a")


