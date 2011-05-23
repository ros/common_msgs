FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/visualization_msgs/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/visualization_msgs/msg/__init__.py"
  "../src/visualization_msgs/msg/_InteractiveMarkerUpdate.py"
  "../src/visualization_msgs/msg/_Marker.py"
  "../src/visualization_msgs/msg/_InteractiveMarkerControl.py"
  "../src/visualization_msgs/msg/_InteractiveMarkerPose.py"
  "../src/visualization_msgs/msg/_MarkerArray.py"
  "../src/visualization_msgs/msg/_InteractiveMarker.py"
  "../src/visualization_msgs/msg/_ImageMarker.py"
  "../src/visualization_msgs/msg/_InteractiveMarkerFeedback.py"
  "../src/visualization_msgs/msg/_Menu.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
