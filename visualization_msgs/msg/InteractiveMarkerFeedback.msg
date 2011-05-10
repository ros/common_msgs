# message that is sent back from the GUI 
# when the status of an interactive marker was modified by the user

# current pose of the marker
geometry_msgs/Pose pose

# contains the text of a menu / sub menu entry, if one has been selected
# empty: nothing was selected
# one item: top-level item was selected
# two items: second-level item was selected
# the first item specifies the top-level item, the second one the second-level item
string[] selected_menu_entry