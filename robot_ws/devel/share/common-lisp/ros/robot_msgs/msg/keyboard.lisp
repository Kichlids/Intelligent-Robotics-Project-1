; Auto-generated. Do not edit!


(cl:in-package robot_msgs-msg)


;//! \htmlinclude keyboard.msg.html

(cl:defclass <keyboard> (roslisp-msg-protocol:ros-message)
  ((is_teleop
    :reader is_teleop
    :initarg :is_teleop
    :type cl:boolean
    :initform cl:nil)
   (command
    :reader command
    :initarg :command
    :type cl:string
    :initform ""))
)

(cl:defclass keyboard (<keyboard>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <keyboard>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'keyboard)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_msgs-msg:<keyboard> is deprecated: use robot_msgs-msg:keyboard instead.")))

(cl:ensure-generic-function 'is_teleop-val :lambda-list '(m))
(cl:defmethod is_teleop-val ((m <keyboard>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_msgs-msg:is_teleop-val is deprecated.  Use robot_msgs-msg:is_teleop instead.")
  (is_teleop m))

(cl:ensure-generic-function 'command-val :lambda-list '(m))
(cl:defmethod command-val ((m <keyboard>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_msgs-msg:command-val is deprecated.  Use robot_msgs-msg:command instead.")
  (command m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <keyboard>) ostream)
  "Serializes a message object of type '<keyboard>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_teleop) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'command))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'command))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <keyboard>) istream)
  "Deserializes a message object of type '<keyboard>"
    (cl:setf (cl:slot-value msg 'is_teleop) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'command) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'command) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<keyboard>)))
  "Returns string type for a message object of type '<keyboard>"
  "robot_msgs/keyboard")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'keyboard)))
  "Returns string type for a message object of type 'keyboard"
  "robot_msgs/keyboard")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<keyboard>)))
  "Returns md5sum for a message object of type '<keyboard>"
  "c9b3c6bb662db54412007781db0793c9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'keyboard)))
  "Returns md5sum for a message object of type 'keyboard"
  "c9b3c6bb662db54412007781db0793c9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<keyboard>)))
  "Returns full string definition for message of type '<keyboard>"
  (cl:format cl:nil "bool is_teleop~%string command~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'keyboard)))
  "Returns full string definition for message of type 'keyboard"
  (cl:format cl:nil "bool is_teleop~%string command~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <keyboard>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'command))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <keyboard>))
  "Converts a ROS message object to a list"
  (cl:list 'keyboard
    (cl:cons ':is_teleop (is_teleop msg))
    (cl:cons ':command (command msg))
))
