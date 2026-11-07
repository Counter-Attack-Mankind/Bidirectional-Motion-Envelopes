; Auto-generated. Do not edit!


(cl:in-package sandbox_msgs-msg)


;//! \htmlinclude ChassisCommand.msg.html

(cl:defclass <ChassisCommand> (roslisp-msg-protocol:ros-message)
  ((target
    :reader target
    :initarg :target
    :type cl:integer
    :initform 0)
   (throttle
    :reader throttle
    :initarg :throttle
    :type cl:float
    :initform 0.0)
   (steering
    :reader steering
    :initarg :steering
    :type cl:float
    :initform 0.0))
)

(cl:defclass ChassisCommand (<ChassisCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ChassisCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ChassisCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sandbox_msgs-msg:<ChassisCommand> is deprecated: use sandbox_msgs-msg:ChassisCommand instead.")))

(cl:ensure-generic-function 'target-val :lambda-list '(m))
(cl:defmethod target-val ((m <ChassisCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sandbox_msgs-msg:target-val is deprecated.  Use sandbox_msgs-msg:target instead.")
  (target m))

(cl:ensure-generic-function 'throttle-val :lambda-list '(m))
(cl:defmethod throttle-val ((m <ChassisCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sandbox_msgs-msg:throttle-val is deprecated.  Use sandbox_msgs-msg:throttle instead.")
  (throttle m))

(cl:ensure-generic-function 'steering-val :lambda-list '(m))
(cl:defmethod steering-val ((m <ChassisCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sandbox_msgs-msg:steering-val is deprecated.  Use sandbox_msgs-msg:steering instead.")
  (steering m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ChassisCommand>) ostream)
  "Serializes a message object of type '<ChassisCommand>"
  (cl:let* ((signed (cl:slot-value msg 'target)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'throttle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'steering))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ChassisCommand>) istream)
  "Deserializes a message object of type '<ChassisCommand>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'target) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'throttle) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'steering) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ChassisCommand>)))
  "Returns string type for a message object of type '<ChassisCommand>"
  "sandbox_msgs/ChassisCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ChassisCommand)))
  "Returns string type for a message object of type 'ChassisCommand"
  "sandbox_msgs/ChassisCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ChassisCommand>)))
  "Returns md5sum for a message object of type '<ChassisCommand>"
  "fc19350b070ac8db9efdde1c244e917a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ChassisCommand)))
  "Returns md5sum for a message object of type 'ChassisCommand"
  "fc19350b070ac8db9efdde1c244e917a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ChassisCommand>)))
  "Returns full string definition for message of type '<ChassisCommand>"
  (cl:format cl:nil "int32 target~%~%float64 throttle~%float64 steering~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ChassisCommand)))
  "Returns full string definition for message of type 'ChassisCommand"
  (cl:format cl:nil "int32 target~%~%float64 throttle~%float64 steering~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ChassisCommand>))
  (cl:+ 0
     4
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ChassisCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'ChassisCommand
    (cl:cons ':target (target msg))
    (cl:cons ':throttle (throttle msg))
    (cl:cons ':steering (steering msg))
))
