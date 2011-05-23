; Auto-generated. Do not edit!


(cl:in-package visualization_msgs-msg)


;//! \htmlinclude InteractiveMarkerFeedback.msg.html

(cl:defclass <InteractiveMarkerFeedback> (roslisp-msg-protocol:ros-message)
  ((marker_name
    :reader marker_name
    :initarg :marker_name
    :type cl:string
    :initform "")
   (control_name
    :reader control_name
    :initarg :control_name
    :type cl:string
    :initform "")
   (event_type
    :reader event_type
    :initarg :event_type
    :type cl:integer
    :initform 0)
   (pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (selected_menu_entry
    :reader selected_menu_entry
    :initarg :selected_menu_entry
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass InteractiveMarkerFeedback (<InteractiveMarkerFeedback>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <InteractiveMarkerFeedback>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'InteractiveMarkerFeedback)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name visualization_msgs-msg:<InteractiveMarkerFeedback> is deprecated: use visualization_msgs-msg:InteractiveMarkerFeedback instead.")))

(cl:ensure-generic-function 'marker_name-val :lambda-list '(m))
(cl:defmethod marker_name-val ((m <InteractiveMarkerFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader visualization_msgs-msg:marker_name-val is deprecated.  Use visualization_msgs-msg:marker_name instead.")
  (marker_name m))

(cl:ensure-generic-function 'control_name-val :lambda-list '(m))
(cl:defmethod control_name-val ((m <InteractiveMarkerFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader visualization_msgs-msg:control_name-val is deprecated.  Use visualization_msgs-msg:control_name instead.")
  (control_name m))

(cl:ensure-generic-function 'event_type-val :lambda-list '(m))
(cl:defmethod event_type-val ((m <InteractiveMarkerFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader visualization_msgs-msg:event_type-val is deprecated.  Use visualization_msgs-msg:event_type instead.")
  (event_type m))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <InteractiveMarkerFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader visualization_msgs-msg:pose-val is deprecated.  Use visualization_msgs-msg:pose instead.")
  (pose m))

(cl:ensure-generic-function 'selected_menu_entry-val :lambda-list '(m))
(cl:defmethod selected_menu_entry-val ((m <InteractiveMarkerFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader visualization_msgs-msg:selected_menu_entry-val is deprecated.  Use visualization_msgs-msg:selected_menu_entry instead.")
  (selected_menu_entry m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<InteractiveMarkerFeedback>)))
    "Constants for message type '<InteractiveMarkerFeedback>"
  '((:POSE_UPDATE . 0)
    (:RECEIVE_FOCUS . 1)
    (:LOSE_FOCUS . 2)
    (:MENU_SELECT . 3)
    (:BUTTON_CLICK . 4))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'InteractiveMarkerFeedback)))
    "Constants for message type 'InteractiveMarkerFeedback"
  '((:POSE_UPDATE . 0)
    (:RECEIVE_FOCUS . 1)
    (:LOSE_FOCUS . 2)
    (:MENU_SELECT . 3)
    (:BUTTON_CLICK . 4))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <InteractiveMarkerFeedback>) ostream)
  "Serializes a message object of type '<InteractiveMarkerFeedback>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'marker_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'marker_name))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'control_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'control_name))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'event_type)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'selected_menu_entry))))
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
   (cl:slot-value msg 'selected_menu_entry))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <InteractiveMarkerFeedback>) istream)
  "Deserializes a message object of type '<InteractiveMarkerFeedback>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'marker_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'marker_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'control_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'control_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'event_type)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'selected_menu_entry) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'selected_menu_entry)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<InteractiveMarkerFeedback>)))
  "Returns string type for a message object of type '<InteractiveMarkerFeedback>"
  "visualization_msgs/InteractiveMarkerFeedback")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'InteractiveMarkerFeedback)))
  "Returns string type for a message object of type 'InteractiveMarkerFeedback"
  "visualization_msgs/InteractiveMarkerFeedback")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<InteractiveMarkerFeedback>)))
  "Returns md5sum for a message object of type '<InteractiveMarkerFeedback>"
  "f3a9d9042bdfbcd749d8e8358cd412f9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'InteractiveMarkerFeedback)))
  "Returns md5sum for a message object of type 'InteractiveMarkerFeedback"
  "f3a9d9042bdfbcd749d8e8358cd412f9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<InteractiveMarkerFeedback>)))
  "Returns full string definition for message of type '<InteractiveMarkerFeedback>"
  (cl:format cl:nil "# message that is sent back from the GUI ~%# when the status of an interactive marker was modified by the user~%~%# specifies which interactive marker and control this message refers to~%string marker_name~%string control_name~%~%# specifies what type of event happened~%# RECEIVE/LOSE_FOCUS: mouse enters or leaves the control's screen area~%# MENU_SELECT: a menu entry has been selected~%# BUTTON_CLICK: a button control has been clicked~%# POSE_UPDATE: the pose has been changed using one of the controls~%byte POSE_UPDATE = 0~%byte RECEIVE_FOCUS = 1~%byte LOSE_FOCUS = 2~%byte MENU_SELECT = 3~%byte BUTTON_CLICK = 4~%~%byte event_type~%~%# current pose of the marker~%geometry_msgs/Pose pose~%~%# contains the text of a menu / sub menu entry, if one has been selected~%# first entry: selected top-level item (if any)~%# second entry: selected second-level item (if any)~%string[] selected_menu_entry~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'InteractiveMarkerFeedback)))
  "Returns full string definition for message of type 'InteractiveMarkerFeedback"
  (cl:format cl:nil "# message that is sent back from the GUI ~%# when the status of an interactive marker was modified by the user~%~%# specifies which interactive marker and control this message refers to~%string marker_name~%string control_name~%~%# specifies what type of event happened~%# RECEIVE/LOSE_FOCUS: mouse enters or leaves the control's screen area~%# MENU_SELECT: a menu entry has been selected~%# BUTTON_CLICK: a button control has been clicked~%# POSE_UPDATE: the pose has been changed using one of the controls~%byte POSE_UPDATE = 0~%byte RECEIVE_FOCUS = 1~%byte LOSE_FOCUS = 2~%byte MENU_SELECT = 3~%byte BUTTON_CLICK = 4~%~%byte event_type~%~%# current pose of the marker~%geometry_msgs/Pose pose~%~%# contains the text of a menu / sub menu entry, if one has been selected~%# first entry: selected top-level item (if any)~%# second entry: selected second-level item (if any)~%string[] selected_menu_entry~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <InteractiveMarkerFeedback>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'marker_name))
     4 (cl:length (cl:slot-value msg 'control_name))
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'selected_menu_entry) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <InteractiveMarkerFeedback>))
  "Converts a ROS message object to a list"
  (cl:list 'InteractiveMarkerFeedback
    (cl:cons ':marker_name (marker_name msg))
    (cl:cons ':control_name (control_name msg))
    (cl:cons ':event_type (event_type msg))
    (cl:cons ':pose (pose msg))
    (cl:cons ':selected_menu_entry (selected_menu_entry msg))
))
