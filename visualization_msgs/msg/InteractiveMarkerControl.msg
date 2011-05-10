# represents a control that is to be displayed together with an interactive marker

# moving / rotating will always be relative to the local coordinate frame
# defined by the control's pose
# - 1d moving happens along local x-axis
# - 2d moving happens in local y-z plane 
# - rotating happens in local y-z plane with the center of rotation being (0,0,0) 
byte NONE = 0 
byte MOVE_AXIS = 1 
byte MOVE_PLANE = 2
byte ROTATE_PLANE = 3
byte MOVE_ROTATE_PLANE = 4
byte BUTTON = 5
byte MENU = 6

# interaction mode for this control (see above)
byte mode

# Pose of this control relative to pose of interactive marker.
# Determines where the default controls are shown
# and what axis or plane will be used for rotating / moving
geometry_msgs/Vector3 vec

# true: always keep orientation fixed relative to the parent frame
#       of the interactive marker
# false: orientation will be relative to the 
#        pose of the interactive marker
bool fixed_orientation

# if true, the contained markers will also be visible
# when the gui is not in interactive mode
bool always_visible

# Markers to be displayed as visual representation
# (leave empty to show default controls)
# Note: the markers can be defined in an arbitrary coordinate frame,
#       but will be transformed into the interactive marker's local frame
#       and will move together with the interactive marker
Marker[] markers
