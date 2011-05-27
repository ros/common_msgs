# Time/frame info.
Header header

# Initial pose. Also, defines the pivot point for rotations.
geometry_msgs/Pose pose

# Identifying string. Must be globally unique in
# the topic that this message is sent through.
string name

# Short description (< 40 characters).
string description

# Scale to be used for default controls (default=1).
float32 scale

# Menu associated with this marker (max. depth is 2).
Menu[] menu

# List of controls displayed for this marker.
InteractiveMarkerControl[] controls

# If this marker should be frame-locked, i.e. retransformed 
# into the reference frame
bool frame_locked

# Reference frame to be used for pose feedback and 
# retransforming (see above).
# Default: same as header.frame_id
string reference_frame_id
