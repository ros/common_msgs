class update_robot_msgs_ChannelFloat32_61c47e4621e471c885edb248b5dcafd5(MessageUpdateRule):
	old_type = "robot_msgs/ChannelFloat32"
	old_full_text = """
string name
float32[] vals
"""

	new_type = "sensor_msgs/ChannelFloat32"
	new_full_text = """
string name
float32[] vals
"""

	order = 0
	migrated_types = []

	valid = True

	def update(self, old_msg, new_msg):
                assert 0, "md5sum matches, update should never be called."

class update_robot_msgs_PointCloud_c47b5cedd2b77d241b27547ed7624840(MessageUpdateRule):
	old_type = "robot_msgs/PointCloud"
	old_full_text = """
Header header
Point32[] pts
ChannelFloat32[] chan

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
MSG: robot_msgs/Point32
float32 x
float32 y
float32 z
================================================================================
MSG: robot_msgs/ChannelFloat32
string name
float32[] vals
"""

	new_type = "sensor_msgs/PointCloud"
	new_full_text = """
Header header
geometry_msgs/Point32[] pts
ChannelFloat32[] chan

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
MSG: geometry_msgs/Point32
float32 x
float32 y
float32 z
================================================================================
MSG: sensor_msgs/ChannelFloat32
string name
float32[] vals
"""

	order = 0
	migrated_types = [
		("Header","Header"),
		("Point32", "geometry_msgs/Point32"),
		("ChannelFloat32", "ChannelFloat32")]

	valid = True

	def update(self, old_msg, new_msg):
                assert 0, "md5sum matches, update should never be called."

