# message that is sent back from the GUI 
# when the status of an interactive marker was modified by the user

# specifies which interactive marker and control this message refers to
string marker_name
string control_name

# specifies what type of event happened
# RECEIVE/LOSE_FOCUS: mouse enters or leaves the control's screen area
# MENU_SELECT: a menu entry has been selected
# BUTTON_CLICK: a button control has been clicked
# POSE_UPDATE: the pose has been changed using one of the controls
byte POSE_UPDATE = 0
byte MENU_SELECT = 1
byte BUTTON_CLICK = 2

byte event_type

# current pose of the marker
geometry_msgs/Pose pose

# contains the text of a menu / sub menu entry, if one has been selected
# first entry: selected top-level item (if any)
# second entry: selected second-level item (if any)
string[] selected_menu_entry

# indicates if the marker is currently being dragged by the user
bool dragging
