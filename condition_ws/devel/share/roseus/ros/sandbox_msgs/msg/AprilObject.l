;; Auto-generated. Do not edit!


(when (boundp 'sandbox_msgs::AprilObject)
  (if (not (find-package "SANDBOX_MSGS"))
    (make-package "SANDBOX_MSGS"))
  (shadow 'AprilObject (find-package "SANDBOX_MSGS")))
(unless (find-package "SANDBOX_MSGS::APRILOBJECT")
  (make-package "SANDBOX_MSGS::APRILOBJECT"))

(in-package "ROS")
;;//! \htmlinclude AprilObject.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*VEHICLE*" (find-package "SANDBOX_MSGS::APRILOBJECT"))
(shadow '*VEHICLE* (find-package "SANDBOX_MSGS::APRILOBJECT"))
(defconstant sandbox_msgs::AprilObject::*VEHICLE* 0)
(intern "*CALIBRE*" (find-package "SANDBOX_MSGS::APRILOBJECT"))
(shadow '*CALIBRE* (find-package "SANDBOX_MSGS::APRILOBJECT"))
(defconstant sandbox_msgs::AprilObject::*CALIBRE* 1)
(intern "*OBSTACLE*" (find-package "SANDBOX_MSGS::APRILOBJECT"))
(shadow '*OBSTACLE* (find-package "SANDBOX_MSGS::APRILOBJECT"))
(defconstant sandbox_msgs::AprilObject::*OBSTACLE* 2)
(intern "*GOAL*" (find-package "SANDBOX_MSGS::APRILOBJECT"))
(shadow '*GOAL* (find-package "SANDBOX_MSGS::APRILOBJECT"))
(defconstant sandbox_msgs::AprilObject::*GOAL* 3)

(defun sandbox_msgs::AprilObject-to-symbol (const)
  (cond
        ((= const 0) 'sandbox_msgs::AprilObject::*VEHICLE*)
        ((= const 1) 'sandbox_msgs::AprilObject::*CALIBRE*)
        ((= const 2) 'sandbox_msgs::AprilObject::*OBSTACLE*)
        ((= const 3) 'sandbox_msgs::AprilObject::*GOAL*)
        (t nil)))

(defclass sandbox_msgs::AprilObject
  :super ros::object
  :slots (_header _type _id _x _y _yaw _polygon ))

(defmethod sandbox_msgs::AprilObject
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:type __type) 0)
    ((:id __id) 0)
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:yaw __yaw) 0.0)
    ((:polygon __polygon) (instance geometry_msgs::Polygon :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _type (round __type))
   (setq _id (round __id))
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _yaw (float __yaw))
   (setq _polygon __polygon)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:yaw
   (&optional __yaw)
   (if __yaw (setq _yaw __yaw)) _yaw)
  (:polygon
   (&rest __polygon)
   (if (keywordp (car __polygon))
       (send* _polygon __polygon)
     (progn
       (if __polygon (setq _polygon (car __polygon)))
       _polygon)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int32 _type
    4
    ;; int32 _id
    4
    ;; float64 _x
    8
    ;; float64 _y
    8
    ;; float64 _yaw
    8
    ;; geometry_msgs/Polygon _polygon
    (send _polygon :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int32 _type
       (write-long _type s)
     ;; int32 _id
       (write-long _id s)
     ;; float64 _x
       (sys::poke _x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y
       (sys::poke _y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _yaw
       (sys::poke _yaw (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; geometry_msgs/Polygon _polygon
       (send _polygon :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int32 _type
     (setq _type (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64 _x
     (setq _x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y
     (setq _y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _yaw
     (setq _yaw (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; geometry_msgs/Polygon _polygon
     (send _polygon :deserialize buf ptr-) (incf ptr- (send _polygon :serialization-length))
   ;;
   self)
  )

(setf (get sandbox_msgs::AprilObject :md5sum-) "f6ef16263f83ad2df90a9f9f37adbd1b")
(setf (get sandbox_msgs::AprilObject :datatype-) "sandbox_msgs/AprilObject")
(setf (get sandbox_msgs::AprilObject :definition-)
      "std_msgs/Header header

int32 VEHICLE=0
int32 CALIBRE=1
int32 OBSTACLE=2
int32 GOAL=3

int32 type
int32 id
float64 x
float64 y
float64 yaw

geometry_msgs/Polygon polygon

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
MSG: geometry_msgs/Polygon
#A specification of a polygon where the first and last points are assumed to be connected
Point32[] points

================================================================================
MSG: geometry_msgs/Point32
# This contains the position of a point in free space(with 32 bits of precision).
# It is recommeded to use Point wherever possible instead of Point32.  
# 
# This recommendation is to promote interoperability.  
#
# This message is designed to take up less space when sending
# lots of points at once, as in the case of a PointCloud.  

float32 x
float32 y
float32 z
")



(provide :sandbox_msgs/AprilObject "f6ef16263f83ad2df90a9f9f37adbd1b")


