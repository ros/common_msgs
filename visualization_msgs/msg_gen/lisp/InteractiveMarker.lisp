; Auto-generated. Do not edit!


(cl:in-package visualization_msgs-msg)


;//! \htmlinclude InteractiveMarker.msg.html

(cl:defclass <InteractiveMarker> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (scale
    :reader scale
    :initarg :scale
    :type cl:float
    :initform 0.0)
   (menu
    :reader menu
    :initarg :menu
    :type (cl:vector visualization_msgs-msg:Menu)
   :initform (cl:make-array 0 :element-type 'visualization_msgs-msg:Menu :initial-element (cl:make-instance 'visualization_msgs-msg:Menu)))
   (controls
    :reader controls
    :initarg :controls
    :type (cl:vector visualization_msgs-msg:InteractiveMarkerControl)
   :initform (cl:make-array 0 :element-type 'visualization_msgs-msg:InteractiveMarkerControl :initial-element (cl:make-instance 'visualization_msgs-msg:InteractiveMarkerControl)))
   (frame_locked
    :reader frame_locked
    :initarg :frame_locked
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass InteractiveMarker (<InteractiveMarker>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <InteractiveMarker>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'InteractiveMarker)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name visualization_msgs-msg:<InteractiveMarker> is deprecated: use visualization_msgs-msg:InteractiveMarker instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <InteractiveMarker>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader visualization_msgs-msg:header-val is deprecated.  Use visualization_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <InteractiveMarker>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader visualization_msgs-msg:name-val is deprecated.  Use visualization_msgs-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <InteractiveMarker>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader visualization_msgs-msg:pose-val is deprecated.  Use visualization_msgs-msg:pose instead.")
  (pose m))

(cl:ensure-generic-function 'scale-val :lambda-list '(m))
(cl:defmethod scale-val ((m <InteractiveMarker>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader visualization_msgs-msg:scale-val is deprecated.  Use visualization_msgs-msg:scale instead.")
  (scale m))

(cl:ensure-generic-function 'menu-val :lambda-list '(m))
(cl:defmethod menu-val ((m <InteractiveMarker>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader visualization_msgs-msg:menu-val is deprecated.  Use visualization_msgs-msg:menu instead.")
  (menu m))

(cl:ensure-generic-function 'controls-val :lambda-list '(m))
(cl:defmethod controls-val ((m <InteractiveMarker>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader visualization_msgs-msg:controls-val is deprecated.  Use visualization_msgs-msg:controls instead.")
  (controls m))

(cl:ensure-generic-function 'frame_locked-val :lambda-list '(m))
(cl:defmethod frame_locked-val ((m <InteractiveMarker>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader visualization_msgs-msg:frame_locked-val is deprecated.  Use visualization_msgs-msg:frame_locked instead.")
  (frame_locked m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <InteractiveMarker>) ostream)
  "Serializes a message object of type '<InteractiveMarker>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'scale))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'menu))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'menu))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'controls))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'controls))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'frame_locked) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <InteractiveMarker>) istream)
  "Deserializes a message object of type '<InteractiveMarker>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'scale) (roslisp-utils:decode-single-float-bits bits)))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'menu) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'menu)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'visualization_msgs-msg:Menu))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'controls) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'controls)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'visualization_msgs-msg:InteractiveMarkerControl))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
    (cl:setf (cl:slot-value msg 'frame_locked) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<InteractiveMarker>)))
  "Returns string type for a message object of type '<InteractiveMarker>"
  "visualization_msgs/InteractiveMarker")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'InteractiveMarker)))
  "Returns string type for a message object of type 'InteractiveMarker"
  "visualization_msgs/InteractiveMarker")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<InteractiveMarker>)))
  "Returns md5sum for a message object of type '<InteractiveMarker>"
  "96318414ab713ab56c50ee14846f6197")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'InteractiveMarker)))
  "Returns md5sum for a message object of type 'InteractiveMarker"
  "96318414ab713ab56c50ee14846f6197")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<InteractiveMarker>)))
  "Returns full string definition for message of type '<InteractiveMarker>"
  (cl:format cl:nil "#Time/frame info~%#The frame_id of this pose is used as 'parent frame' for the controls~%#which have a fixed orientation.~%Header header~%~%# Name of this marker. Only necessary if you are sending multiple markers.~%# See InteractiveMarkerArray for details.~%string name~%~%# Initial pose of the interactive marker. Defines the pivot point for rotations.~%geometry_msgs/Pose pose~%~%# Scale to be used for default controls (default=1).~%float32 scale~%~%# Menu associated with this marker (max. depth is 2).~%Menu[] menu~%~%# List of controls displayed for this marker.~%# To delete an interactive marker, send one without controls.~%InteractiveMarkerControl[] controls~%~%# If this marker should be frame-locked, i.e. retransformed ~%# into its parent frame every timestep.~%bool frame_locked~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: visualization_msgs/Menu~%# menu / entry title~%string title~%~%# entries for this menu~%# if empty, this menu represents a simple menu entry itself~%string[] entries~%~%================================================================================~%MSG: visualization_msgs/InteractiveMarkerControl~%# represents a control that is to be displayed together with an interactive marker~%~%# Identifying string for this control.~%# You need to assign a unique value to this to receive feedback from the GUI~%# on what actions the user performs on this control (e.g. a button click).~%string name~%~%# Defines the local coordinate frame (relative to the pose of the parent~%# interactive marker) in which is being rotated and translated.~%# Note: Does not influence the pose of the contained markers.~%geometry_msgs/Quaternion orientation~%~%# Interaction mode for this control~%# ~%# NONE: this control is only meant for visualization, context menu is deactivated~%# MENU: like none, but right-click menu is active~%# BUTTON: element can be left-clicked~%# MOVE_AXIS: Translate along local x-axis~%# MOVE_PLANE: Translate in local y-z plane~%# ROTATE_AXIS: Rotate around local x-axis~%# MOVE_ROTATE: Combines MOVE_PLANE and ROTATE_AXIS~%byte NONE = 0 ~%byte MENU = 1~%byte BUTTON = 2~%byte MOVE_AXIS = 3 ~%byte MOVE_PLANE = 4~%byte ROTATE_AXIS = 5~%byte MOVE_ROTATE = 6~%~%byte interaction_mode~%~%# how should the orientation be updated?~%# INHERIT: follow orientation of interactive marker~%# FIXED: keep orientation fixed at initial state~%# VIEW_FACING: align y-z plane with screen (x: forward, y:left, z:up)~%byte INHERIT = 0 ~%byte FIXED = 1~%byte VIEW_FACING = 2 ~%~%byte orientation_mode~%~%# if true, the contained markers will also be visible~%# when the gui is not in interactive mode~%bool always_visible~%~%# Markers to be displayed as custom visual representation.~%# Leave this empty to use the default control handles.~%#~%# Note: ~%# - The markers can be defined in an arbitrary coordinate frame,~%#   but will be transformed into the local frame of the interactive marker.~%# - If the header of a marker is empty, its pose will be interpreted as ~%#   relative to the pose of the parent interactive marker.~%Marker[] markers~%~%# Short description (<80 characters) of what this control does,~%# e.g. \"Move the robot\". If no tool tip is given, a generic description~%# based on the interaction mode is assigned.~%string tool_tip~%~%================================================================================~%MSG: visualization_msgs/Marker~%# See http://www.ros.org/wiki/rviz/DisplayTypes/Marker and http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes for more information on using this message with rviz~%~%byte ARROW=0~%byte CUBE=1~%byte SPHERE=2~%byte CYLINDER=3~%byte LINE_STRIP=4~%byte LINE_LIST=5~%byte CUBE_LIST=6~%byte SPHERE_LIST=7~%byte POINTS=8~%byte TEXT_VIEW_FACING=9~%byte MESH_RESOURCE=10~%byte TRIANGLE_LIST=11~%~%byte ADD=0~%byte MODIFY=0~%byte DELETE=2~%~%Header header                        # header for time/frame information~%string ns                            # Namespace to place this object in... used in conjunction with id to create a unique name for the object~%int32 id 		                         # object ID useful in conjunction with the namespace for manipulating and deleting the object later~%int32 type 		                       # Type of object~%int32 action 	                       # 0 add/modify an object, 1 (deprecated), 2 deletes an object~%geometry_msgs/Pose pose                 # Pose of the object~%geometry_msgs/Vector3 scale             # Scale of the object 1,1,1 means default (usually 1 meter square)~%std_msgs/ColorRGBA color             # Color [0.0-1.0]~%duration lifetime                    # How long the object should last before being automatically deleted.  0 means forever~%bool frame_locked                    # If this marker should be frame-locked, i.e. retransformed into its frame every timestep~%~%#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)~%geometry_msgs/Point[] points~%#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)~%#number of colors must either be 0 or equal to the number of points~%#NOTE: alpha is not yet used~%std_msgs/ColorRGBA[] colors~%~%# NOTE: only used for text markers~%string text~%~%# NOTE: only used for MESH_RESOURCE markers~%string mesh_resource~%bool mesh_use_embedded_materials~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: std_msgs/ColorRGBA~%float32 r~%float32 g~%float32 b~%float32 a~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'InteractiveMarker)))
  "Returns full string definition for message of type 'InteractiveMarker"
  (cl:format cl:nil "#Time/frame info~%#The frame_id of this pose is used as 'parent frame' for the controls~%#which have a fixed orientation.~%Header header~%~%# Name of this marker. Only necessary if you are sending multiple markers.~%# See InteractiveMarkerArray for details.~%string name~%~%# Initial pose of the interactive marker. Defines the pivot point for rotations.~%geometry_msgs/Pose pose~%~%# Scale to be used for default controls (default=1).~%float32 scale~%~%# Menu associated with this marker (max. depth is 2).~%Menu[] menu~%~%# List of controls displayed for this marker.~%# To delete an interactive marker, send one without controls.~%InteractiveMarkerControl[] controls~%~%# If this marker should be frame-locked, i.e. retransformed ~%# into its parent frame every timestep.~%bool frame_locked~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: visualization_msgs/Menu~%# menu / entry title~%string title~%~%# entries for this menu~%# if empty, this menu represents a simple menu entry itself~%string[] entries~%~%================================================================================~%MSG: visualization_msgs/InteractiveMarkerControl~%# represents a control that is to be displayed together with an interactive marker~%~%# Identifying string for this control.~%# You need to assign a unique value to this to receive feedback from the GUI~%# on what actions the user performs on this control (e.g. a button click).~%string name~%~%# Defines the local coordinate frame (relative to the pose of the parent~%# interactive marker) in which is being rotated and translated.~%# Note: Does not influence the pose of the contained markers.~%geometry_msgs/Quaternion orientation~%~%# Interaction mode for this control~%# ~%# NONE: this control is only meant for visualization, context menu is deactivated~%# MENU: like none, but right-click menu is active~%# BUTTON: element can be left-clicked~%# MOVE_AXIS: Translate along local x-axis~%# MOVE_PLANE: Translate in local y-z plane~%# ROTATE_AXIS: Rotate around local x-axis~%# MOVE_ROTATE: Combines MOVE_PLANE and ROTATE_AXIS~%byte NONE = 0 ~%byte MENU = 1~%byte BUTTON = 2~%byte MOVE_AXIS = 3 ~%byte MOVE_PLANE = 4~%byte ROTATE_AXIS = 5~%byte MOVE_ROTATE = 6~%~%byte interaction_mode~%~%# how should the orientation be updated?~%# INHERIT: follow orientation of interactive marker~%# FIXED: keep orientation fixed at initial state~%# VIEW_FACING: align y-z plane with screen (x: forward, y:left, z:up)~%byte INHERIT = 0 ~%byte FIXED = 1~%byte VIEW_FACING = 2 ~%~%byte orientation_mode~%~%# if true, the contained markers will also be visible~%# when the gui is not in interactive mode~%bool always_visible~%~%# Markers to be displayed as custom visual representation.~%# Leave this empty to use the default control handles.~%#~%# Note: ~%# - The markers can be defined in an arbitrary coordinate frame,~%#   but will be transformed into the local frame of the interactive marker.~%# - If the header of a marker is empty, its pose will be interpreted as ~%#   relative to the pose of the parent interactive marker.~%Marker[] markers~%~%# Short description (<80 characters) of what this control does,~%# e.g. \"Move the robot\". If no tool tip is given, a generic description~%# based on the interaction mode is assigned.~%string tool_tip~%~%================================================================================~%MSG: visualization_msgs/Marker~%# See http://www.ros.org/wiki/rviz/DisplayTypes/Marker and http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes for more information on using this message with rviz~%~%byte ARROW=0~%byte CUBE=1~%byte SPHERE=2~%byte CYLINDER=3~%byte LINE_STRIP=4~%byte LINE_LIST=5~%byte CUBE_LIST=6~%byte SPHERE_LIST=7~%byte POINTS=8~%byte TEXT_VIEW_FACING=9~%byte MESH_RESOURCE=10~%byte TRIANGLE_LIST=11~%~%byte ADD=0~%byte MODIFY=0~%byte DELETE=2~%~%Header header                        # header for time/frame information~%string ns                            # Namespace to place this object in... used in conjunction with id to create a unique name for the object~%int32 id 		                         # object ID useful in conjunction with the namespace for manipulating and deleting the object later~%int32 type 		                       # Type of object~%int32 action 	                       # 0 add/modify an object, 1 (deprecated), 2 deletes an object~%geometry_msgs/Pose pose                 # Pose of the object~%geometry_msgs/Vector3 scale             # Scale of the object 1,1,1 means default (usually 1 meter square)~%std_msgs/ColorRGBA color             # Color [0.0-1.0]~%duration lifetime                    # How long the object should last before being automatically deleted.  0 means forever~%bool frame_locked                    # If this marker should be frame-locked, i.e. retransformed into its frame every timestep~%~%#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)~%geometry_msgs/Point[] points~%#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)~%#number of colors must either be 0 or equal to the number of points~%#NOTE: alpha is not yet used~%std_msgs/ColorRGBA[] colors~%~%# NOTE: only used for text markers~%string text~%~%# NOTE: only used for MESH_RESOURCE markers~%string mesh_resource~%bool mesh_use_embedded_materials~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: std_msgs/ColorRGBA~%float32 r~%float32 g~%float32 b~%float32 a~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <InteractiveMarker>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'name))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'menu) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'controls) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <InteractiveMarker>))
  "Converts a ROS message object to a list"
  (cl:list 'InteractiveMarker
    (cl:cons ':header (header msg))
    (cl:cons ':name (name msg))
    (cl:cons ':pose (pose msg))
    (cl:cons ':scale (scale msg))
    (cl:cons ':menu (menu msg))
    (cl:cons ':controls (controls msg))
    (cl:cons ':frame_locked (frame_locked msg))
))
