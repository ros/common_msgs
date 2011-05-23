; Auto-generated. Do not edit!


(cl:in-package visualization_msgs-msg)


;//! \htmlinclude Menu.msg.html

(cl:defclass <Menu> (roslisp-msg-protocol:ros-message)
  ((title
    :reader title
    :initarg :title
    :type cl:string
    :initform "")
   (entries
    :reader entries
    :initarg :entries
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass Menu (<Menu>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Menu>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Menu)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name visualization_msgs-msg:<Menu> is deprecated: use visualization_msgs-msg:Menu instead.")))

(cl:ensure-generic-function 'title-val :lambda-list '(m))
(cl:defmethod title-val ((m <Menu>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader visualization_msgs-msg:title-val is deprecated.  Use visualization_msgs-msg:title instead.")
  (title m))

(cl:ensure-generic-function 'entries-val :lambda-list '(m))
(cl:defmethod entries-val ((m <Menu>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader visualization_msgs-msg:entries-val is deprecated.  Use visualization_msgs-msg:entries instead.")
  (entries m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Menu>) ostream)
  "Serializes a message object of type '<Menu>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'title))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'title))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'entries))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'entries))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Menu>) istream)
  "Deserializes a message object of type '<Menu>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'title) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'title) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'entries) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'entries)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Menu>)))
  "Returns string type for a message object of type '<Menu>"
  "visualization_msgs/Menu")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Menu)))
  "Returns string type for a message object of type 'Menu"
  "visualization_msgs/Menu")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Menu>)))
  "Returns md5sum for a message object of type '<Menu>"
  "16c8d1c975b2f590eea835c5b6e516f1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Menu)))
  "Returns md5sum for a message object of type 'Menu"
  "16c8d1c975b2f590eea835c5b6e516f1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Menu>)))
  "Returns full string definition for message of type '<Menu>"
  (cl:format cl:nil "# menu / entry title~%string title~%~%# entries for this menu~%# if empty, this menu represents a simple menu entry itself~%string[] entries~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Menu)))
  "Returns full string definition for message of type 'Menu"
  (cl:format cl:nil "# menu / entry title~%string title~%~%# entries for this menu~%# if empty, this menu represents a simple menu entry itself~%string[] entries~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Menu>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'title))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'entries) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Menu>))
  "Converts a ROS message object to a list"
  (cl:list 'Menu
    (cl:cons ':title (title msg))
    (cl:cons ':entries (entries msg))
))
