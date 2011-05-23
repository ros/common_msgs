; Auto-generated. Do not edit!


(cl:in-package visualization_msgs-msg)


;//! \htmlinclude InteractiveMarkerControl.msg.html

(cl:defclass <InteractiveMarkerControl> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (orientation
    :reader orientation
    :initarg :orientation
    :type geometry_msgs-msg:Quaternion
    :initform (cl:make-instance 'geometry_msgs-msg:Quaternion))
   (interaction_mode
    :reader interaction_mode
    :initarg :interaction_mode
    :type cl:integer
    :initform 0)
   (orientation_mode
    :reader orientation_mode
    :initarg :orientation_mode
    :type cl:integer
    :initform 0)
   (always_visible
    :reader always_visible
    :initarg :always_visible
    :type cl:boolean
    :initform cl:nil)
   (markers
    :reader markers
    :initarg :markers
    :type (cl:vector visualization_msgs-msg:Marker)
   :initform (cl:make-array 0 :element-type 'visualization_msgs-msg:Marker :initial-element (cl:make-instance 'visualization_msgs-msg:Marker)))
   (tool_tip
    :reader tool_tip
    :initarg :tool_tip
    :type cl:string
    :initform ""))
)

(cl:defclass InteractiveMarkerControl (<InteractiveMarkerControl>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <InteractiveMarkerControl>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'InteractiveMarkerControl)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name visualization_msgs-msg:<InteractiveMarkerControl> is deprecated: use visualization_msgs-msg:InteractiveMarkerControl instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <InteractiveMarkerControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader visualization_msgs-msg:name-val is deprecated.  Use visualization_msgs-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'orientation-val :lambda-list '(m))
(cl:defmethod orientation-val ((m <InteractiveMarkerControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader visualization_msgs-msg:orientation-val is deprecated.  Use visualization_msgs-msg:orientation instead.")
  (orientation m))

(cl:ensure-generic-function 'interaction_mode-val :lambda-list '(m))
(cl:defmethod interaction_mode-val ((m <InteractiveMarkerControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader visualization_msgs-msg:interaction_mode-val is deprecated.  Use visualization_msgs-msg:interaction_mode instead.")
  (interaction_mode m))

(cl:ensure-generic-function 'orientation_mode-val :lambda-list '(m))
(cl:defmethod orientation_mode-val ((m <InteractiveMarkerControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader visualization_msgs-msg:orientation_mode-val is deprecated.  Use visualization_msgs-msg:orientation_mode instead.")
  (orientation_mode m))

(cl:ensure-generic-function 'always_visible-val :lambda-list '(m))
(cl:defmethod always_visible-val ((m <InteractiveMarkerControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader visualization_msgs-msg:always_visible-val is deprecated.  Use visualization_msgs-msg:always_visible instead.")
  (always_visible m))

(cl:ensure-generic-function 'markers-val :lambda-list '(m))
(cl:defmethod markers-val ((m <InteractiveMarkerControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader visualization_msgs-msg:markers-val is deprecated.  Use visualization_msgs-msg:markers instead.")
  (markers m))

(cl:ensure-generic-function 'tool_tip-val :lambda-list '(m))
(cl:defmethod tool_tip-val ((m <InteractiveMarkerControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader visualization_msgs-msg:tool_tip-val is deprecated.  Use visualization_msgs-msg:tool_tip instead.")
  (tool_tip m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<InteractiveMarkerControl>)))
    "Constants for message type '<InteractiveMarkerControl>"
  '((:NONE . 0)
    (:MENU . 1)
    (:BUTTON . 2)
    (:MOVE_AXIS . 3)
    (:MOVE_PLANE . 4)
    (:ROTATE_AXIS . 5)
    (:MOVE_ROTATE . 6)
    (:INHERIT . 0)
    (:FIXED . 1)
    (:VIEW_FACING . 2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'InteractiveMarkerControl)))
    "Constants for message type 'InteractiveMarkerControl"
  '((:NONE . 0)
    (:MENU . 1)
    (:BUTTON . 2)
    (:MOVE_AXIS . 3)
    (:MOVE_PLANE . 4)
    (:ROTATE_AXIS . 5)
    (:MOVE_ROTATE . 6)
    (:INHERIT . 0)
    (:FIXED . 1)
    (:VIEW_FACING . 2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <InteractiveMarkerControl>) ostream)
  "Serializes a message object of type '<InteractiveMarkerControl>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'orientation) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'interaction_mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'orientation_mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'always_visible) 1 0)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'markers))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'markers))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'tool_tip))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'tool_tip))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <InteractiveMarkerControl>) istream)
  "Deserializes a message object of type '<InteractiveMarkerControl>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'orientation) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'interaction_mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'orientation_mode)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'always_visible) (cl:not (cl:zerop (cl:read-byte istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'markers) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'markers)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'visualization_msgs-msg:Marker))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'tool_tip) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'tool_tip) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<InteractiveMarkerControl>)))
  "Returns string type for a message object of type '<InteractiveMarkerControl>"
  "visualization_msgs/InteractiveMarkerControl")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'InteractiveMarkerControl)))
  "Returns string type for a message object of type 'InteractiveMarkerControl"
  "visualization_msgs/InteractiveMarkerControl")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<InteractiveMarkerControl>)))
  "Returns md5sum for a message object of type '<InteractiveMarkerControl>"
  "c9b2f10d73f27af53253f9632fd3ae7d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'InteractiveMarkerControl)))
  "Returns md5sum for a message object of type 'InteractiveMarkerControl"
  "c9b2f10d73f27af53253f9632fd3ae7d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<InteractiveMarkerControl>)))
  "Returns full string definition for message of type '<InteractiveMarkerControl>"
  (cl:format cl:nil "# represents a control that is to be displayed together with an interactive marker~%~%# Identifying string for this control.~%# You need to assign a unique value to this to receive feedback from the GUI~%# on what actions the user performs on this control (e.g. a button click).~%string name~%~%# Defines the local coordinate frame (relative to the pose of the parent~%# interactive marker) in which is being rotated and translated.~%# Note: Does not influence the pose of the contained markers.~%geometry_msgs/Quaternion orientation~%~%# Interaction mode for this control~%# ~%# NONE: this control is only meant for visualization, context menu is deactivated~%# MENU: like none, but right-click menu is active~%# BUTTON: element can be left-clicked~%# MOVE_AXIS: Translate along local x-axis~%# MOVE_PLANE: Translate in local y-z plane~%# ROTATE_AXIS: Rotate around local x-axis~%# MOVE_ROTATE: Combines MOVE_PLANE and ROTATE_AXIS~%byte NONE = 0 ~%byte MENU = 1~%byte BUTTON = 2~%byte MOVE_AXIS = 3 ~%byte MOVE_PLANE = 4~%byte ROTATE_AXIS = 5~%byte MOVE_ROTATE = 6~%~%byte interaction_mode~%~%# how should the orientation be updated?~%# INHERIT: follow orientation of interactive marker~%# FIXED: keep orientation fixed at initial state~%# VIEW_FACING: align y-z plane with screen (x: forward, y:left, z:up)~%byte INHERIT = 0 ~%byte FIXED = 1~%byte VIEW_FACING = 2 ~%~%byte orientation_mode~%~%# if true, the contained markers will also be visible~%# when the gui is not in interactive mode~%bool always_visible~%~%# Markers to be displayed as custom visual representation.~%# Leave this empty to use the default control handles.~%#~%# Note: ~%# - The markers can be defined in an arbitrary coordinate frame,~%#   but will be transformed into the local frame of the interactive marker.~%# - If the header of a marker is empty, its pose will be interpreted as ~%#   relative to the pose of the parent interactive marker.~%Marker[] markers~%~%# Short description (<80 characters) of what this control does,~%# e.g. \"Move the robot\". If no tool tip is given, a generic description~%# based on the interaction mode is assigned.~%string tool_tip~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: visualization_msgs/Marker~%# See http://www.ros.org/wiki/rviz/DisplayTypes/Marker and http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes for more information on using this message with rviz~%~%byte ARROW=0~%byte CUBE=1~%byte SPHERE=2~%byte CYLINDER=3~%byte LINE_STRIP=4~%byte LINE_LIST=5~%byte CUBE_LIST=6~%byte SPHERE_LIST=7~%byte POINTS=8~%byte TEXT_VIEW_FACING=9~%byte MESH_RESOURCE=10~%byte TRIANGLE_LIST=11~%~%byte ADD=0~%byte MODIFY=0~%byte DELETE=2~%~%Header header                        # header for time/frame information~%string ns                            # Namespace to place this object in... used in conjunction with id to create a unique name for the object~%int32 id 		                         # object ID useful in conjunction with the namespace for manipulating and deleting the object later~%int32 type 		                       # Type of object~%int32 action 	                       # 0 add/modify an object, 1 (deprecated), 2 deletes an object~%geometry_msgs/Pose pose                 # Pose of the object~%geometry_msgs/Vector3 scale             # Scale of the object 1,1,1 means default (usually 1 meter square)~%std_msgs/ColorRGBA color             # Color [0.0-1.0]~%duration lifetime                    # How long the object should last before being automatically deleted.  0 means forever~%bool frame_locked                    # If this marker should be frame-locked, i.e. retransformed into its frame every timestep~%~%#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)~%geometry_msgs/Point[] points~%#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)~%#number of colors must either be 0 or equal to the number of points~%#NOTE: alpha is not yet used~%std_msgs/ColorRGBA[] colors~%~%# NOTE: only used for text markers~%string text~%~%# NOTE: only used for MESH_RESOURCE markers~%string mesh_resource~%bool mesh_use_embedded_materials~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: std_msgs/ColorRGBA~%float32 r~%float32 g~%float32 b~%float32 a~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'InteractiveMarkerControl)))
  "Returns full string definition for message of type 'InteractiveMarkerControl"
  (cl:format cl:nil "# represents a control that is to be displayed together with an interactive marker~%~%# Identifying string for this control.~%# You need to assign a unique value to this to receive feedback from the GUI~%# on what actions the user performs on this control (e.g. a button click).~%string name~%~%# Defines the local coordinate frame (relative to the pose of the parent~%# interactive marker) in which is being rotated and translated.~%# Note: Does not influence the pose of the contained markers.~%geometry_msgs/Quaternion orientation~%~%# Interaction mode for this control~%# ~%# NONE: this control is only meant for visualization, context menu is deactivated~%# MENU: like none, but right-click menu is active~%# BUTTON: element can be left-clicked~%# MOVE_AXIS: Translate along local x-axis~%# MOVE_PLANE: Translate in local y-z plane~%# ROTATE_AXIS: Rotate around local x-axis~%# MOVE_ROTATE: Combines MOVE_PLANE and ROTATE_AXIS~%byte NONE = 0 ~%byte MENU = 1~%byte BUTTON = 2~%byte MOVE_AXIS = 3 ~%byte MOVE_PLANE = 4~%byte ROTATE_AXIS = 5~%byte MOVE_ROTATE = 6~%~%byte interaction_mode~%~%# how should the orientation be updated?~%# INHERIT: follow orientation of interactive marker~%# FIXED: keep orientation fixed at initial state~%# VIEW_FACING: align y-z plane with screen (x: forward, y:left, z:up)~%byte INHERIT = 0 ~%byte FIXED = 1~%byte VIEW_FACING = 2 ~%~%byte orientation_mode~%~%# if true, the contained markers will also be visible~%# when the gui is not in interactive mode~%bool always_visible~%~%# Markers to be displayed as custom visual representation.~%# Leave this empty to use the default control handles.~%#~%# Note: ~%# - The markers can be defined in an arbitrary coordinate frame,~%#   but will be transformed into the local frame of the interactive marker.~%# - If the header of a marker is empty, its pose will be interpreted as ~%#   relative to the pose of the parent interactive marker.~%Marker[] markers~%~%# Short description (<80 characters) of what this control does,~%# e.g. \"Move the robot\". If no tool tip is given, a generic description~%# based on the interaction mode is assigned.~%string tool_tip~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: visualization_msgs/Marker~%# See http://www.ros.org/wiki/rviz/DisplayTypes/Marker and http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes for more information on using this message with rviz~%~%byte ARROW=0~%byte CUBE=1~%byte SPHERE=2~%byte CYLINDER=3~%byte LINE_STRIP=4~%byte LINE_LIST=5~%byte CUBE_LIST=6~%byte SPHERE_LIST=7~%byte POINTS=8~%byte TEXT_VIEW_FACING=9~%byte MESH_RESOURCE=10~%byte TRIANGLE_LIST=11~%~%byte ADD=0~%byte MODIFY=0~%byte DELETE=2~%~%Header header                        # header for time/frame information~%string ns                            # Namespace to place this object in... used in conjunction with id to create a unique name for the object~%int32 id 		                         # object ID useful in conjunction with the namespace for manipulating and deleting the object later~%int32 type 		                       # Type of object~%int32 action 	                       # 0 add/modify an object, 1 (deprecated), 2 deletes an object~%geometry_msgs/Pose pose                 # Pose of the object~%geometry_msgs/Vector3 scale             # Scale of the object 1,1,1 means default (usually 1 meter square)~%std_msgs/ColorRGBA color             # Color [0.0-1.0]~%duration lifetime                    # How long the object should last before being automatically deleted.  0 means forever~%bool frame_locked                    # If this marker should be frame-locked, i.e. retransformed into its frame every timestep~%~%#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)~%geometry_msgs/Point[] points~%#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)~%#number of colors must either be 0 or equal to the number of points~%#NOTE: alpha is not yet used~%std_msgs/ColorRGBA[] colors~%~%# NOTE: only used for text markers~%string text~%~%# NOTE: only used for MESH_RESOURCE markers~%string mesh_resource~%bool mesh_use_embedded_materials~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: std_msgs/ColorRGBA~%float32 r~%float32 g~%float32 b~%float32 a~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <InteractiveMarkerControl>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'orientation))
     1
     1
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'markers) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:length (cl:slot-value msg 'tool_tip))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <InteractiveMarkerControl>))
  "Converts a ROS message object to a list"
  (cl:list 'InteractiveMarkerControl
    (cl:cons ':name (name msg))
    (cl:cons ':orientation (orientation msg))
    (cl:cons ':interaction_mode (interaction_mode msg))
    (cl:cons ':orientation_mode (orientation_mode msg))
    (cl:cons ':always_visible (always_visible msg))
    (cl:cons ':markers (markers msg))
    (cl:cons ':tool_tip (tool_tip msg))
))
