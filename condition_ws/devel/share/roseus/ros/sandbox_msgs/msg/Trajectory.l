;; Auto-generated. Do not edit!


(when (boundp 'sandbox_msgs::Trajectory)
  (if (not (find-package "SANDBOX_MSGS"))
    (make-package "SANDBOX_MSGS"))
  (shadow 'Trajectory (find-package "SANDBOX_MSGS")))
(unless (find-package "SANDBOX_MSGS::TRAJECTORY")
  (make-package "SANDBOX_MSGS::TRAJECTORY"))

(in-package "ROS")
;;//! \htmlinclude Trajectory.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass sandbox_msgs::Trajectory
  :super ros::object
  :slots (_header _target _points ))

(defmethod sandbox_msgs::Trajectory
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:target __target) 0)
    ((:points __points) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _target (round __target))
   (setq _points __points)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:target
   (&optional __target)
   (if __target (setq _target __target)) _target)
  (:points
   (&rest __points)
   (if (keywordp (car __points))
       (send* _points __points)
     (progn
       (if __points (setq _points (car __points)))
       _points)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int32 _target
    4
    ;; sandbox_msgs/TrajectoryPoint[] _points
    (apply #'+ (send-all _points :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int32 _target
       (write-long _target s)
     ;; sandbox_msgs/TrajectoryPoint[] _points
     (write-long (length _points) s)
     (dolist (elem _points)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int32 _target
     (setq _target (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; sandbox_msgs/TrajectoryPoint[] _points
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _points (let (r) (dotimes (i n) (push (instance sandbox_msgs::TrajectoryPoint :init) r)) r))
     (dolist (elem- _points)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get sandbox_msgs::Trajectory :md5sum-) "e566b001305190558481913b6d3ad34c")
(setf (get sandbox_msgs::Trajectory :datatype-) "sandbox_msgs/Trajectory")
(setf (get sandbox_msgs::Trajectory :definition-)
      "std_msgs/Header header
int32 target

TrajectoryPoint[] points

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

================================================================================
MSG: sandbox_msgs/TrajectoryPoint
float64 time
float64 x
float64 y
float64 yaw

float64 velocity
float64 acceleration

")



(provide :sandbox_msgs/Trajectory "e566b001305190558481913b6d3ad34c")


