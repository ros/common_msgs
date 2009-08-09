class update_robot_msgs_MapMetaData_10cfc8a2818024d3248802c00c95f11b(MessageUpdateRule):
	old_type = "robot_msgs/MapMetaData"
	old_full_text = """
# The time at which the map was loaded
time map_load_time
# The map resolution [m/cell]
float32 resolution
# Map width [cells]
uint32 width
# Map height [cells]
uint32 height
# The origin of the map [m, m, rad].  This is the real-world pose of the
# cell (0,0) in the map.
robot_msgs/Pose origin
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

	new_type = "nav_msgs/MapMetaData"
	new_full_text = """
# The time at which the map was loaded
time map_load_time
# The map resolution [m/cell]
float32 resolution
# Map width [cells]
uint32 width
# Map height [cells]
uint32 height
# The origin of the map [m, m, rad].  This is the real-world pose of the
# cell (0,0) in the map.
geometry_msgs/Pose origin
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
		("Pose","geometry_msgs/Pose"),]

	valid = True

	def update(self, old_msg, new_msg):
		new_msg.map_load_time = old_msg.map_load_time
		new_msg.resolution = old_msg.resolution
		new_msg.width = old_msg.width
		new_msg.height = old_msg.height
		self.migrate(old_msg.origin, new_msg.origin)
class update_robot_msgs_OccGrid_97b32e1f188a9205cdc57db3617fc4ca(MessageUpdateRule):
	old_type = "robot_msgs/OccGrid"
	old_full_text = """
# A 2-D grid map, in which each cell represents the probability of
# occupancy.  Occupancy values are integers in the range [0,100], or -1 
# for unknown.

#MetaData for the map
MapMetaData info

# The map data, in row-major order, starting with (0,0).  Occupancy
# probabilities are in the range [0,100].  Unknown is -1.
byte[] data

================================================================================
MSG: robot_msgs/MapMetaData
# The time at which the map was loaded
time map_load_time
# The map resolution [m/cell]
float32 resolution
# Map width [cells]
uint32 width
# Map height [cells]
uint32 height
# The origin of the map [m, m, rad].  This is the real-world pose of the
# cell (0,0) in the map.
robot_msgs/Pose origin
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

	new_type = "nav_msgs/OccupancyGrid"
	new_full_text = """
# A 2-D grid map, in which each cell represents the probability of
# occupancy.  Occupancy values are integers in the range [0,100], or -1 
# for unknown.

Header header 

#MetaData for the map
MapMetaData info

# The map data, in row-major order, starting with (0,0).  Occupancy
# probabilities are in the range [0,100].  Unknown is -1.
byte[] data

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
MSG: nav_msgs/MapMetaData
# The time at which the map was loaded
time map_load_time
# The map resolution [m/cell]
float32 resolution
# Map width [cells]
uint32 width
# Map height [cells]
uint32 height
# The origin of the map [m, m, rad].  This is the real-world pose of the
# cell (0,0) in the map.
geometry_msgs/Pose origin
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
		("MapMetaData","MapMetaData"),]

	valid = True

	def update(self, old_msg, new_msg):
                import rospy
		new_msg.header = self.get_new_class('Header')(0, rospy.Time(0,0), "/map")
		self.migrate(old_msg.info, new_msg.info)
		new_msg.data = old_msg.data

