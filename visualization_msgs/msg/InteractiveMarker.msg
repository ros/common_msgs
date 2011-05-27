# Time/frame info.
Header header

# Identifying string. Must be globally unique in
# the topic that this message is sent through.
string name

# Short description (< 40 characters).
string description

# Initial pose. Also, defines the pivot point for rotations.
geometry_msgs/Pose pose

# Scale to be used for default controls (default=1).
float32 scale

# Menu associated with this marker (max. depth is 2).
Menu[] menu

# List of controls displayed for this marker.
InteractiveMarkerControl[] controls

# If this marker should be frame-locked, i.e. retransformed 
# into the frame specified in the header on every timestep.
bool frame_locked
