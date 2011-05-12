#Time/frame info
#The frame_id of this pose is used as 'parent frame' for the controls
#which have a fixed orientation.
Header header

# Name of this marker. Only necessary if you are sending multiple markers.
# See InteractiveMarkerArray for details.
string name

# Initial pose of the interactive marker. Defines the pivot point for rotations.
geometry_msgs/Pose pose

# Scale to be used for default controls (default=1).
float32 size

# Menu associated with this marker (max. depth is 2).
Menu[] menu

# List of controls displayed for this marker.
# To delete an interactive marker, send one without controls.
InteractiveMarkerControl[] controls

# If this marker should be frame-locked, i.e. retransformed 
# into its parent frame every timestep.
bool frame_locked
