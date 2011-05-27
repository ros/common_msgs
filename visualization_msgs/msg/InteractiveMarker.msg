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
# into the frame specified in the header.
# Note: If this is set to true, you will receive pose feedback
# in the frame specified in the header of this message.
# If set to false, you can receive feedback in any arbitrary
# frame.
# For rviz, this will be the current 'fixed frame' set by the user.
bool frame_locked
