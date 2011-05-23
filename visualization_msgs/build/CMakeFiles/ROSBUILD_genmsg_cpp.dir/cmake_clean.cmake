FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/visualization_msgs/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/visualization_msgs/InteractiveMarkerUpdate.h"
  "../msg_gen/cpp/include/visualization_msgs/Marker.h"
  "../msg_gen/cpp/include/visualization_msgs/InteractiveMarkerControl.h"
  "../msg_gen/cpp/include/visualization_msgs/InteractiveMarkerPose.h"
  "../msg_gen/cpp/include/visualization_msgs/MarkerArray.h"
  "../msg_gen/cpp/include/visualization_msgs/InteractiveMarker.h"
  "../msg_gen/cpp/include/visualization_msgs/ImageMarker.h"
  "../msg_gen/cpp/include/visualization_msgs/InteractiveMarkerFeedback.h"
  "../msg_gen/cpp/include/visualization_msgs/Menu.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
