; Auto-generated. Do not edit!


(cl:in-package visualization_msgs-msg)


;//! \htmlinclude ImageMarker.msg.html

(cl:defclass <ImageMarker> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (ns
    :reader ns
    :initarg :ns
    :type cl:string
    :initform "")
   (id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0)
   (type
    :reader type
    :initarg :type
    :type cl:integer
    :initform 0)
   (action
    :reader action
    :initarg :action
    :type cl:integer
    :initform 0)
   (position
    :reader position
    :initarg :position
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (scale
    :reader scale
    :initarg :scale
    :type cl:float
    :initform 0.0)
   (outline_color
    :reader outline_color
    :initarg :outline_color
    :type std_msgs-msg:ColorRGBA
    :initform (cl:make-instance 'std_msgs-msg:ColorRGBA))
   (filled
    :reader filled
    :initarg :filled
    :type cl:integer
    :initform 0)
   (fill_color
    :reader fill_color
    :initarg :fill_color
    :type std_msgs-msg:ColorRGBA
    :initform (cl:make-instance 'std_msgs-msg:ColorRGBA))
   (lifetime
    :reader lifetime
    :initarg :lifetime
    :type cl:real
    :initform 0)
   (points
    :reader points
    :initarg :points
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point)))
   (outline_colors
    :reader outline_colors
    :initarg :outline_colors
    :type (cl:vector std_msgs-msg:ColorRGBA)
   :initform (cl:make-array 0 :element-type 'std_msgs-msg:ColorRGBA :initial-element (cl:make-instance 'std_msgs-msg:ColorRGBA))))
)

(cl:defclass ImageMarker (<ImageMarker>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ImageMarker>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ImageMarker)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name visualization_msgs-msg:<ImageMarker> is deprecated: use visualization_msgs-msg:ImageMarker instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ImageMarker>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader visualization_msgs-msg:header-val is deprecated.  Use visualization_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'ns-val :lambda-list '(m))
(cl:defmethod ns-val ((m <ImageMarker>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader visualization_msgs-msg:ns-val is deprecated.  Use visualization_msgs-msg:ns instead.")
  (ns m))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <ImageMarker>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader visualization_msgs-msg:id-val is deprecated.  Use visualization_msgs-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <ImageMarker>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader visualization_msgs-msg:type-val is deprecated.  Use visualization_msgs-msg:type instead.")
  (type m))

(cl:ensure-generic-function 'action-val :lambda-list '(m))
(cl:defmethod action-val ((m <ImageMarker>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader visualization_msgs-msg:action-val is deprecated.  Use visualization_msgs-msg:action instead.")
  (action m))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <ImageMarker>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader visualization_msgs-msg:position-val is deprecated.  Use visualization_msgs-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'scale-val :lambda-list '(m))
(cl:defmethod scale-val ((m <ImageMarker>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader visualization_msgs-msg:scale-val is deprecated.  Use visualization_msgs-msg:scale instead.")
  (scale m))

(cl:ensure-generic-function 'outline_color-val :lambda-list '(m))
(cl:defmethod outline_color-val ((m <ImageMarker>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader visualization_msgs-msg:outline_color-val is deprecated.  Use visualization_msgs-msg:outline_color instead.")
  (outline_color m))

(cl:ensure-generic-function 'filled-val :lambda-list '(m))
(cl:defmethod filled-val ((m <ImageMarker>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader visualization_msgs-msg:filled-val is deprecated.  Use visualization_msgs-msg:filled instead.")
  (filled m))

(cl:ensure-generic-function 'fill_color-val :lambda-list '(m))
(cl:defmethod fill_color-val ((m <ImageMarker>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader visualization_msgs-msg:fill_color-val is deprecated.  Use visualization_msgs-msg:fill_color instead.")
  (fill_color m))

(cl:ensure-generic-function 'lifetime-val :lambda-list '(m))
(cl:defmethod lifetime-val ((m <ImageMarker>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader visualization_msgs-msg:lifetime-val is deprecated.  Use visualization_msgs-msg:lifetime instead.")
  (lifetime m))

(cl:ensure-generic-function 'points-val :lambda-list '(m))
(cl:defmethod points-val ((m <ImageMarker>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader visualization_msgs-msg:points-val is deprecated.  Use visualization_msgs-msg:points instead.")
  (points m))

(cl:ensure-generic-function 'outline_colors-val :lambda-list '(m))
(cl:defmethod outline_colors-val ((m <ImageMarker>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader visualization_msgs-msg:outline_colors-val is deprecated.  Use visualization_msgs-msg:outline_colors instead.")
  (outline_colors m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<ImageMarker>)))
    "Constants for message type '<ImageMarker>"
  '((:CIRCLE . 0)
    (:LINE_STRIP . 1)
    (:LINE_LIST . 2)
    (:POLYGON . 3)
    (:POINTS . 4)
    (:ADD . 0)
    (:REMOVE . 1))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'ImageMarker)))
    "Constants for message type 'ImageMarker"
  '((:CIRCLE . 0)
    (:LINE_STRIP . 1)
    (:LINE_LIST . 2)
    (:POLYGON . 3)
    (:POINTS . 4)
    (:ADD . 0)
    (:REMOVE . 1))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ImageMarker>) ostream)
  "Serializes a message object of type '<ImageMarker>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'ns))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'ns))
  (cl:let* ((signed (cl:slot-value msg 'id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'type)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'action)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'scale))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'outline_color) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'filled)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'fill_color) ostream)
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'lifetime)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'lifetime) (cl:floor (cl:slot-value msg 'lifetime)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'points))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'points))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'outline_colors))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'outline_colors))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ImageMarker>) istream)
  "Deserializes a message object of type '<ImageMarker>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ns) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'ns) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'type) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'action) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'scale) (roslisp-utils:decode-single-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'outline_color) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'filled)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'fill_color) istream)
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'lifetime) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'points) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'points)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'outline_colors) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'outline_colors)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'std_msgs-msg:ColorRGBA))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ImageMarker>)))
  "Returns string type for a message object of type '<ImageMarker>"
  "visualization_msgs/ImageMarker")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ImageMarker)))
  "Returns string type for a message object of type 'ImageMarker"
  "visualization_msgs/ImageMarker")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ImageMarker>)))
  "Returns md5sum for a message object of type '<ImageMarker>"
  "e185c670c5f817486e6a216cbee30f1c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ImageMarker)))
  "Returns md5sum for a message object of type 'ImageMarker"
  "e185c670c5f817486e6a216cbee30f1c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ImageMarker>)))
  "Returns full string definition for message of type '<ImageMarker>"
  (cl:format cl:nil "byte CIRCLE=0~%byte LINE_STRIP=1~%byte LINE_LIST=2~%byte POLYGON=3~%byte POINTS=4~%~%byte ADD=0~%byte REMOVE=1~%~%Header header~%string ns		# namespace, used with id to form a unique id~%int32 id          	# unique id within the namespace~%int32 type        	# CIRCLE/LINE_STRIP/etc.~%int32 action      	# ADD/REMOVE~%geometry_msgs/Point position # 2D, in pixel-coords~%float32 scale	 	# the diameter for a circle, etc.~%std_msgs/ColorRGBA outline_color~%byte filled		# whether to fill in the shape with color~%std_msgs/ColorRGBA fill_color # color [0.0-1.0]~%duration lifetime       # How long the object should last before being automatically deleted.  0 means forever~%~%~%geometry_msgs/Point[] points # used for LINE_STRIP/LINE_LIST/POINTS/etc., 2D in pixel coords~%std_msgs/ColorRGBA[] outline_colors # a color for each line, point, etc.~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: std_msgs/ColorRGBA~%float32 r~%float32 g~%float32 b~%float32 a~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ImageMarker)))
  "Returns full string definition for message of type 'ImageMarker"
  (cl:format cl:nil "byte CIRCLE=0~%byte LINE_STRIP=1~%byte LINE_LIST=2~%byte POLYGON=3~%byte POINTS=4~%~%byte ADD=0~%byte REMOVE=1~%~%Header header~%string ns		# namespace, used with id to form a unique id~%int32 id          	# unique id within the namespace~%int32 type        	# CIRCLE/LINE_STRIP/etc.~%int32 action      	# ADD/REMOVE~%geometry_msgs/Point position # 2D, in pixel-coords~%float32 scale	 	# the diameter for a circle, etc.~%std_msgs/ColorRGBA outline_color~%byte filled		# whether to fill in the shape with color~%std_msgs/ColorRGBA fill_color # color [0.0-1.0]~%duration lifetime       # How long the object should last before being automatically deleted.  0 means forever~%~%~%geometry_msgs/Point[] points # used for LINE_STRIP/LINE_LIST/POINTS/etc., 2D in pixel coords~%std_msgs/ColorRGBA[] outline_colors # a color for each line, point, etc.~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: std_msgs/ColorRGBA~%float32 r~%float32 g~%float32 b~%float32 a~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ImageMarker>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'ns))
     4
     4
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position))
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'outline_color))
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'fill_color))
     8
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'points) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'outline_colors) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ImageMarker>))
  "Converts a ROS message object to a list"
  (cl:list 'ImageMarker
    (cl:cons ':header (header msg))
    (cl:cons ':ns (ns msg))
    (cl:cons ':id (id msg))
    (cl:cons ':type (type msg))
    (cl:cons ':action (action msg))
    (cl:cons ':position (position msg))
    (cl:cons ':scale (scale msg))
    (cl:cons ':outline_color (outline_color msg))
    (cl:cons ':filled (filled msg))
    (cl:cons ':fill_color (fill_color msg))
    (cl:cons ':lifetime (lifetime msg))
    (cl:cons ':points (points msg))
    (cl:cons ':outline_colors (outline_colors msg))
))
