; Auto-generated. Do not edit!


(cl:in-package collab_planner-srv)


;//! \htmlinclude KeyboardInput-request.msg.html

(cl:defclass <KeyboardInput-request> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type cl:string
    :initform ""))
)

(cl:defclass KeyboardInput-request (<KeyboardInput-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <KeyboardInput-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'KeyboardInput-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name collab_planner-srv:<KeyboardInput-request> is deprecated: use collab_planner-srv:KeyboardInput-request instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <KeyboardInput-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader collab_planner-srv:data-val is deprecated.  Use collab_planner-srv:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <KeyboardInput-request>) ostream)
  "Serializes a message object of type '<KeyboardInput-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <KeyboardInput-request>) istream)
  "Deserializes a message object of type '<KeyboardInput-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'data) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'data) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<KeyboardInput-request>)))
  "Returns string type for a service object of type '<KeyboardInput-request>"
  "collab_planner/KeyboardInputRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'KeyboardInput-request)))
  "Returns string type for a service object of type 'KeyboardInput-request"
  "collab_planner/KeyboardInputRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<KeyboardInput-request>)))
  "Returns md5sum for a message object of type '<KeyboardInput-request>"
  "e7ac98ca304b04e45988c61adb6bc4ce")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'KeyboardInput-request)))
  "Returns md5sum for a message object of type 'KeyboardInput-request"
  "e7ac98ca304b04e45988c61adb6bc4ce")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<KeyboardInput-request>)))
  "Returns full string definition for message of type '<KeyboardInput-request>"
  (cl:format cl:nil "string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'KeyboardInput-request)))
  "Returns full string definition for message of type 'KeyboardInput-request"
  (cl:format cl:nil "string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <KeyboardInput-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'data))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <KeyboardInput-request>))
  "Converts a ROS message object to a list"
  (cl:list 'KeyboardInput-request
    (cl:cons ':data (data msg))
))
;//! \htmlinclude KeyboardInput-response.msg.html

(cl:defclass <KeyboardInput-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass KeyboardInput-response (<KeyboardInput-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <KeyboardInput-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'KeyboardInput-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name collab_planner-srv:<KeyboardInput-response> is deprecated: use collab_planner-srv:KeyboardInput-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <KeyboardInput-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader collab_planner-srv:success-val is deprecated.  Use collab_planner-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <KeyboardInput-response>) ostream)
  "Serializes a message object of type '<KeyboardInput-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <KeyboardInput-response>) istream)
  "Deserializes a message object of type '<KeyboardInput-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<KeyboardInput-response>)))
  "Returns string type for a service object of type '<KeyboardInput-response>"
  "collab_planner/KeyboardInputResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'KeyboardInput-response)))
  "Returns string type for a service object of type 'KeyboardInput-response"
  "collab_planner/KeyboardInputResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<KeyboardInput-response>)))
  "Returns md5sum for a message object of type '<KeyboardInput-response>"
  "e7ac98ca304b04e45988c61adb6bc4ce")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'KeyboardInput-response)))
  "Returns md5sum for a message object of type 'KeyboardInput-response"
  "e7ac98ca304b04e45988c61adb6bc4ce")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<KeyboardInput-response>)))
  "Returns full string definition for message of type '<KeyboardInput-response>"
  (cl:format cl:nil "bool success~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'KeyboardInput-response)))
  "Returns full string definition for message of type 'KeyboardInput-response"
  (cl:format cl:nil "bool success~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <KeyboardInput-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <KeyboardInput-response>))
  "Converts a ROS message object to a list"
  (cl:list 'KeyboardInput-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'KeyboardInput)))
  'KeyboardInput-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'KeyboardInput)))
  'KeyboardInput-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'KeyboardInput)))
  "Returns string type for a service object of type '<KeyboardInput>"
  "collab_planner/KeyboardInput")