# represents a control that is to be displayed together with an interactive marker

# Identifying string for this control.
# You need to assign a unique value to this to receive feedback from the GUI
# on what actions the user performs on this control (e.g. a button click).
string name

# Defines the local coordinate frame (relative to the pose of the parent
# interactive marker) in which is being rotated and translated.
# Note: Does not influence the pose of the contained markers.
geometry_msgs/Quaternion orientation

# Interaction mode for this control
# 
# NONE: this control is only meant for visualization, context menu is deactivated
# MENU: like none, but right-click menu is active
# BUTTON: element can be left-clicked
# MOVE_AXIS: Translate along local x-axis
# MOVE_PLANE: Translate in local y-z plane
# ROTATE_AXIS: Rotate around local x-axis
# MOVE_ROTATE: Combines MOVE_PLANE and ROTATE_AXIS
byte NONE = 0 
byte MENU = 1
byte BUTTON = 2
byte MOVE_AXIS = 3 
byte MOVE_PLANE = 4
byte ROTATE_AXIS = 5
byte MOVE_ROTATE = 6

byte interaction_mode

# how should the orientation be updated?
# INHERIT: follow orientation of interactive marker
# FIXED: keep orientation fixed at initial state
# VIEW_FACING: align y-z plane with screen (x: forward, y:left, z:up)
byte INHERIT = 0 
byte FIXED = 1
byte VIEW_FACING = 2 

byte orientation_mode

# if true, the contained markers will also be visible
# when the gui is not in interactive mode
bool always_visible

# Markers to be displayed as custom visual representation.
# Leave this empty to use the default control handles.
#
# Note: 
# - The markers can be defined in an arbitrary coordinate frame,
#   but will be transformed into the local frame of the interactive marker.
# - If the header of a marker is empty, its pose will be interpreted as 
#   relative to the pose of the parent interactive marker.
Marker[] markers

# Short description (<80 characters) of what this control does,
# e.g. "Move the robot". If no tool tip is given, a generic description
# based on the interaction mode is assigned.
string tool_tip
