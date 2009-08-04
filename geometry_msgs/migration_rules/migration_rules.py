class update_robot_msgs_Vector3_4a842b65f413084dc2b10fb484ea7f17(MessageUpdateRule):
	old_type = "robot_msgs/Vector3"
	old_full_text = """
float64 x
float64 y
float64 z
"""

	new_type = "geometry_msgs/Vector3"
	new_full_text = """
float64 x
float64 y
float64 z
"""

	order = 0
	migrated_types = []

	valid = True

	def update(self, old_msg, new_msg):
		new_msg.x = old_msg.x
		new_msg.y = old_msg.y
		new_msg.z = old_msg.z


class update_robot_msgs_Twist_104c6ef591961bed596cfa30f858271c(MessageUpdateRule):
	old_type = "robot_msgs/Twist"
	old_full_text = """
Header header
robot_msgs/Vector3  vel
robot_msgs/Vector3  rot

================================================================================
MSG: roslib/Header
#Standard metadata for higher-level flow data types
#sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: robot_msgs/Vector3
float64 x
float64 y
float64 z
"""

	new_type = "geometry_msgs/Twist"
	new_full_text = """
Vector3  linear
Vector3  angular

================================================================================
MSG: geometry_msgs/Vector3
float64 x
float64 y
float64 z
"""

	order = 0
	migrated_types = [('Vector3', 'Vector3')]

	valid = True

	def update(self, old_msg, new_msg):
                self.migrate(old_msg.vel, new_msg.linear)
                self.migrate(old_msg.rot, new_msg.angular)

class update_robot_msgs_Quaternion_a779879fadf0160734f906b8c19c7004(MessageUpdateRule):
	old_type = "robot_msgs/Quaternion"
	old_full_text = """
# xyz - vector rotation axis, w - scalar term (cos(ang/2))
float64 x
float64 y
float64 z
float64 w
"""

	new_type = "geometry_msgs/Quaternion"
	new_full_text = """
# xyz - vector rotation axis, w - scalar term (cos(ang/2))
float64 x
float64 y
float64 z
float64 w
"""

	order = 0
	migrated_types = []

	valid = True

	def update(self, old_msg, new_msg):
		new_msg.x = old_msg.x
		new_msg.y = old_msg.y
		new_msg.z = old_msg.z
		new_msg.w = old_msg.w

class update_robot_msgs_QuaternionStamped_e57f1e547e0e1fd13504588ffc8334e2(MessageUpdateRule):
	old_type = "robot_msgs/QuaternionStamped"
	old_full_text = """
Header header
Quaternion quaternion

================================================================================
MSG: roslib/Header
#Standard metadata for higher-level flow data types
#sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: robot_msgs/Quaternion
# xyz - vector rotation axis, w - scalar term (cos(ang/2))
float64 x
float64 y
float64 z
float64 w
"""

	new_type = "geometry_msgs/QuaternionStamped"
	new_full_text = """
Header header
Quaternion quaternion

================================================================================
MSG: roslib/Header
#Standard metadata for higher-level flow data types
#sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: geometry_msgs/Quaternion
# xyz - vector rotation axis, w - scalar term (cos(ang/2))
float64 x
float64 y
float64 z
float64 w
"""

	order = 0
	migrated_types = [
		("Header","Header"),
		("Quaternion","Quaternion"),]

	valid = True

	def update(self, old_msg, new_msg):
		self.migrate(old_msg.header, new_msg.header)
		self.migrate(old_msg.quaternion, new_msg.quaternion)

class update_robot_msgs_Vector3Stamped_7b324c7325e683bf02a9b14b01090ec7(MessageUpdateRule):
	old_type = "robot_msgs/Vector3Stamped"
	old_full_text = """
Header header
Vector3 vector

================================================================================
MSG: roslib/Header
#Standard metadata for higher-level flow data types
#sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: robot_msgs/Vector3
float64 x
float64 y
float64 z
"""

	new_type = "geometry_msgs/Vector3Stamped"
	new_full_text = """
Header header
Vector3 vector

================================================================================
MSG: roslib/Header
#Standard metadata for higher-level flow data types
#sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: geometry_msgs/Vector3
float64 x
float64 y
float64 z
"""

	order = 0
	migrated_types = [
		("Header","Header"),
		("Vector3","Vector3"),]

	valid = True

	def update(self, old_msg, new_msg):
		self.migrate(old_msg.header, new_msg.header)
		self.migrate(old_msg.vector, new_msg.vector)

class update_robot_msgs_Point_4a842b65f413084dc2b10fb484ea7f17(MessageUpdateRule):
	old_type = "robot_msgs/Point"
	old_full_text = """
float64 x
float64 y
float64 z
"""

	new_type = "geometry_msgs/Point"
	new_full_text = """
float64 x
float64 y
float64 z
"""

	order = 0
	migrated_types = []

	valid = True

	def update(self, old_msg, new_msg):
		new_msg.x = old_msg.x
		new_msg.y = old_msg.y
		new_msg.z = old_msg.z

class update_robot_msgs_PointStamped_c63aecb41bfdfd6b7e1fac37c7cbe7bf(MessageUpdateRule):
	old_type = "robot_msgs/PointStamped"
	old_full_text = """
Header header
Point point

================================================================================
MSG: roslib/Header
#Standard metadata for higher-level flow data types
#sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: robot_msgs/Point
float64 x
float64 y
float64 z
"""

	new_type = "geometry_msgs/PointStamped"
	new_full_text = """
Header header
Point point

================================================================================
MSG: roslib/Header
#Standard metadata for higher-level flow data types
#sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: geometry_msgs/Point
float64 x
float64 y
float64 z
"""

	order = 0
	migrated_types = [
		("Header","Header"),
		("Point","Point"),]

	valid = True

	def update(self, old_msg, new_msg):
		self.migrate(old_msg.header, new_msg.header)
		self.migrate(old_msg.point, new_msg.point)

class update_robot_msgs_Transform_ac9eff44abf714214112b05d54a3cf9b(MessageUpdateRule):
	old_type = "robot_msgs/Transform"
	old_full_text = """
Vector3 translation
Quaternion rotation

================================================================================
MSG: robot_msgs/Vector3
float64 x
float64 y
float64 z
================================================================================
MSG: robot_msgs/Quaternion
# xyz - vector rotation axis, w - scalar term (cos(ang/2))
float64 x
float64 y
float64 z
float64 w
"""

	new_type = "geometry_msgs/Transform"
	new_full_text = """
Vector3 translation
Quaternion rotation

================================================================================
MSG: geometry_msgs/Vector3
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# xyz - vector rotation axis, w - scalar term (cos(ang/2))
float64 x
float64 y
float64 z
float64 w
"""

	order = 0
	migrated_types = [
		("Vector3","Vector3"),
		("Quaternion","Quaternion"),]

	valid = True

	def update(self, old_msg, new_msg):
		self.migrate(old_msg.translation, new_msg.translation)
		self.migrate(old_msg.rotation, new_msg.rotation)

class update_robot_msgs_TransformStamped_e0b1e16e4f459246e737dca6251c020b(MessageUpdateRule):
	old_type = "robot_msgs/TransformStamped"
	old_full_text = """
Header header
string parent_id # the frame id of the parent frame
Transform transform

================================================================================
MSG: roslib/Header
#Standard metadata for higher-level flow data types
#sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: robot_msgs/Transform
Vector3 translation
Quaternion rotation

================================================================================
MSG: robot_msgs/Vector3
float64 x
float64 y
float64 z
================================================================================
MSG: robot_msgs/Quaternion
# xyz - vector rotation axis, w - scalar term (cos(ang/2))
float64 x
float64 y
float64 z
float64 w
"""

	new_type = "geometry_msgs/TransformStamped"
	new_full_text = """
Header header
string parent_id # the frame id of the parent frame
Transform transform

================================================================================
MSG: roslib/Header
#Standard metadata for higher-level flow data types
#sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: geometry_msgs/Transform
Vector3 translation
Quaternion rotation

================================================================================
MSG: geometry_msgs/Vector3
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# xyz - vector rotation axis, w - scalar term (cos(ang/2))
float64 x
float64 y
float64 z
float64 w
"""

	order = 0
	migrated_types = [
		("Header","Header"),
		("Transform","Transform"),]

	valid = True

	def update(self, old_msg, new_msg):
		self.migrate(old_msg.header, new_msg.header)
		new_msg.parent_id = old_msg.parent_id
		self.migrate(old_msg.transform, new_msg.transform)

class update_robot_msgs_Pose_e45d45a5a1ce597b249e23fb30fc871f(MessageUpdateRule):
	old_type = "robot_msgs/Pose"
	old_full_text = """
Point position
Quaternion orientation

================================================================================
MSG: robot_msgs/Point
float64 x
float64 y
float64 z

================================================================================
MSG: robot_msgs/Quaternion
# xyz - vector rotation axis, w - scalar term (cos(ang/2))
float64 x
float64 y
float64 z
float64 w
"""

	new_type = "geometry_msgs/Pose"
	new_full_text = """
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# xyz - vector rotation axis, w - scalar term (cos(ang/2))
float64 x
float64 y
float64 z
float64 w
"""

	order = 0
	migrated_types = [
		("Point","Point"),
		("Quaternion","Quaternion"),]

	valid = True

	def update(self, old_msg, new_msg):
		self.migrate(old_msg.position, new_msg.position)
		self.migrate(old_msg.orientation, new_msg.orientation)

class update_robot_msgs_PoseStamped_d3812c3cbc69362b77dc0b19b345f8f5(MessageUpdateRule):
	old_type = "robot_msgs/PoseStamped"
	old_full_text = """
Header header
Pose pose

================================================================================
MSG: roslib/Header
#Standard metadata for higher-level flow data types
#sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: robot_msgs/Pose
Point position
Quaternion orientation

================================================================================
MSG: robot_msgs/Point
float64 x
float64 y
float64 z

================================================================================
MSG: robot_msgs/Quaternion
# xyz - vector rotation axis, w - scalar term (cos(ang/2))
float64 x
float64 y
float64 z
float64 w
"""

	new_type = "geometry_msgs/PoseStamped"
	new_full_text = """
Header header
Pose pose

================================================================================
MSG: roslib/Header
#Standard metadata for higher-level flow data types
#sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: geometry_msgs/Pose
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# xyz - vector rotation axis, w - scalar term (cos(ang/2))
float64 x
float64 y
float64 z
float64 w
"""

	order = 0
	migrated_types = [
		("Header","Header"),
		("Pose","Pose"),]

	valid = True

	def update(self, old_msg, new_msg):
		self.migrate(old_msg.header, new_msg.header)
		self.migrate(old_msg.pose, new_msg.pose)

class update_robot_msgs_Velocity_ffb367ff390f5e01cb55c0c75927c19a(MessageUpdateRule):
	old_type = "robot_msgs/Velocity"
	old_full_text = """
float64 vx
float64 vy
float64 vz
"""

	new_type = "geometry_msgs/Vector3"
	new_full_text = """
float64 x
float64 y
float64 z
"""

	order = 0
	migrated_types = []

	valid = True

	def update(self, old_msg, new_msg):
		new_msg.x = old_msg.vx
		new_msg.y = old_msg.vy
		new_msg.z = old_msg.vz
class update_robot_msgs_Point32_cc153912f1453b708d221682bc23d9ac(MessageUpdateRule):
	old_type = "robot_msgs/Point32"
	old_full_text = """
float32 x
float32 y
float32 z
"""

	new_type = "geometry_msgs/Point32"
	new_full_text = """
float32 x
float32 y
float32 z
"""

	order = 0
	migrated_types = []

	valid = True

	def update(self, old_msg, new_msg):
		new_msg.x = old_msg.x
		new_msg.y = old_msg.y
		new_msg.z = old_msg.z

