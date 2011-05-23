; Auto-generated. Do not edit!


(cl:in-package visualization_msgs-msg)


;//! \htmlinclude MarkerArray.msg.html

(cl:defclass <MarkerArray> (roslisp-msg-protocol:ros-message)
  ((markers
    :reader markers
    :initarg :markers
    :type (cl:vector visualization_msgs-msg:Marker)
   :initform (cl:make-array 0 :element-type 'visualization_msgs-msg:Marker :initial-element (cl:make-instance 'visualization_msgs-msg:Marker))))
)

(cl:defclass MarkerArray (<MarkerArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MarkerArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MarkerArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name visualization_msgs-msg:<MarkerArray> is deprecated: use visualization_msgs-msg:MarkerArray instead.")))

(cl:ensure-generic-function 'markers-val :lambda-list '(m))
(cl:defmethod markers-val ((m <MarkerArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader visualization_msgs-msg:markers-val is deprecated.  Use visualization_msgs-msg:markers instead.")
  (markers m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MarkerArray>) ostream)
  "Serializes a message object of type '<MarkerArray>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'markers))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'markers))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MarkerArray>) istream)
  "Deserializes a message object of type '<MarkerArray>"
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
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MarkerArray>)))
  "Returns string type for a message object of type '<MarkerArray>"
  "visualization_msgs/MarkerArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MarkerArray)))
  "Returns string type for a message object of type 'MarkerArray"
  "visualization_msgs/MarkerArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MarkerArray>)))
  "Returns md5sum for a message object of type '<MarkerArray>"
  "f10fe193d6fac1bf68fad5d31da421a7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MarkerArray)))
  "Returns md5sum for a message object of type 'MarkerArray"
  "f10fe193d6fac1bf68fad5d31da421a7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MarkerArray>)))
  "Returns full string definition for message of type '<MarkerArray>"
  (cl:format cl:nil "Marker[] markers~%~%================================================================================~%MSG: visualization_msgs/Marker~%# See http://www.ros.org/wiki/rviz/DisplayTypes/Marker and http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes for more information on using this message with rviz~%~%byte ARROW=0~%byte CUBE=1~%byte SPHERE=2~%byte CYLINDER=3~%byte LINE_STRIP=4~%byte LINE_LIST=5~%byte CUBE_LIST=6~%byte SPHERE_LIST=7~%byte POINTS=8~%byte TEXT_VIEW_FACING=9~%byte MESH_RESOURCE=10~%byte TRIANGLE_LIST=11~%~%byte ADD=0~%byte MODIFY=0~%byte DELETE=2~%~%Header header                        # header for time/frame information~%string ns                            # Namespace to place this object in... used in conjunction with id to create a unique name for the object~%int32 id 		                         # object ID useful in conjunction with the namespace for manipulating and deleting the object later~%int32 type 		                       # Type of object~%int32 action 	                       # 0 add/modify an object, 1 (deprecated), 2 deletes an object~%geometry_msgs/Pose pose                 # Pose of the object~%geometry_msgs/Vector3 scale             # Scale of the object 1,1,1 means default (usually 1 meter square)~%std_msgs/ColorRGBA color             # Color [0.0-1.0]~%duration lifetime                    # How long the object should last before being automatically deleted.  0 means forever~%bool frame_locked                    # If this marker should be frame-locked, i.e. retransformed into its frame every timestep~%~%#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)~%geometry_msgs/Point[] points~%#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)~%#number of colors must either be 0 or equal to the number of points~%#NOTE: alpha is not yet used~%std_msgs/ColorRGBA[] colors~%~%# NOTE: only used for text markers~%string text~%~%# NOTE: only used for MESH_RESOURCE markers~%string mesh_resource~%bool mesh_use_embedded_materials~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: std_msgs/ColorRGBA~%float32 r~%float32 g~%float32 b~%float32 a~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MarkerArray)))
  "Returns full string definition for message of type 'MarkerArray"
  (cl:format cl:nil "Marker[] markers~%~%================================================================================~%MSG: visualization_msgs/Marker~%# See http://www.ros.org/wiki/rviz/DisplayTypes/Marker and http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes for more information on using this message with rviz~%~%byte ARROW=0~%byte CUBE=1~%byte SPHERE=2~%byte CYLINDER=3~%byte LINE_STRIP=4~%byte LINE_LIST=5~%byte CUBE_LIST=6~%byte SPHERE_LIST=7~%byte POINTS=8~%byte TEXT_VIEW_FACING=9~%byte MESH_RESOURCE=10~%byte TRIANGLE_LIST=11~%~%byte ADD=0~%byte MODIFY=0~%byte DELETE=2~%~%Header header                        # header for time/frame information~%string ns                            # Namespace to place this object in... used in conjunction with id to create a unique name for the object~%int32 id 		                         # object ID useful in conjunction with the namespace for manipulating and deleting the object later~%int32 type 		                       # Type of object~%int32 action 	                       # 0 add/modify an object, 1 (deprecated), 2 deletes an object~%geometry_msgs/Pose pose                 # Pose of the object~%geometry_msgs/Vector3 scale             # Scale of the object 1,1,1 means default (usually 1 meter square)~%std_msgs/ColorRGBA color             # Color [0.0-1.0]~%duration lifetime                    # How long the object should last before being automatically deleted.  0 means forever~%bool frame_locked                    # If this marker should be frame-locked, i.e. retransformed into its frame every timestep~%~%#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)~%geometry_msgs/Point[] points~%#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)~%#number of colors must either be 0 or equal to the number of points~%#NOTE: alpha is not yet used~%std_msgs/ColorRGBA[] colors~%~%# NOTE: only used for text markers~%string text~%~%# NOTE: only used for MESH_RESOURCE markers~%string mesh_resource~%bool mesh_use_embedded_materials~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: std_msgs/ColorRGBA~%float32 r~%float32 g~%float32 b~%float32 a~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MarkerArray>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'markers) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MarkerArray>))
  "Converts a ROS message object to a list"
  (cl:list 'MarkerArray
    (cl:cons ':markers (markers msg))
))
