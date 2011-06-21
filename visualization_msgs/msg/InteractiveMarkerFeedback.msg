# Time/frame info.
Header header

# Identifying string. Must be unique in the topic namespace.
string client_id

# Feedback message sent back from the GUI, e.g.
# when the status of an interactive marker was modified by the user.

# Specifies which interactive marker and control this message refers to
string marker_name
string control_name

# Type of the event
# KEEP_ALIVE: sent while dragging to keep up control of the marker
# MENU_SELECT: a menu entry has been selected
# BUTTON_CLICK: a button control has been clicked
# POSE_UPDATE: the pose has been changed using one of the controls
byte KEEP_ALIVE = 0
byte POSE_UPDATE = 1
byte MENU_SELECT = 2
byte BUTTON_CLICK = 3

byte MOUSE_DOWN = 4
byte MOUSE_UP = 5

byte event_type

# Current pose of the marker
# Note: Has to be valid for all feedback types.
geometry_msgs/Pose pose

# Contains the command of the selected menu entry
# Only valid for MENU_SELECT events.
string command
