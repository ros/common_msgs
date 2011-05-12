# represents a control that is to be displayed together with an interactive marker

# moving / rotating will always be relative to the local coordinate frame
# defined by the control's orientation and 
# - 1d moving happens along local x-axis
# - 2d moving happens in local y-z plane 
# - rotating happens in local y-z plane with the center of rotation being (0,0,0) 

# interaction mode for this control
byte NONE = 0 
byte MOVE_AXIS = 1 
byte MOVE_PLANE = 2
byte ROTATE_AXIS = 3
byte MOVE_ROTATE = 4
byte BUTTON = 5
byte MENU = 6

byte mode

# Defines the local coordinate frame (relative to the pose of the parent
# interactive marker) in which is being rotated and translated.
# Does _not_ influence the pose of the contained markers.
geometry_msgs/Quaternion orientation

# if true, the contained markers will also be visible
# when the gui is not in interactive mode
bool always_visible

# Short description (<80 characters) of what this control does,
# e.g. "Move the robot".
string tool_tip

# in which orientation should the controls be displayed?
byte INHERIT = 0 # follow orientation of interactive marker
byte FIXED = 1 # keep orientation fixed at initial state
byte VIEW_FACING = 2 # align y-z plane with screen (y:left, z:up)

byte marker_orientation

# Markers to be displayed as custom visual representation.
# Leave this empty to use the default control handles.
#
# Note: 
# - The markers can be defined in an arbitrary coordinate frame,
#   but will be transformed into the local frame of the interactive marker.
# - If the header of a marker is empty, its pose will be interpreted as 
#   relative to the pose of the parent interactive marker.
Marker[] markers
