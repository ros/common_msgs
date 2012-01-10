project(visualization_msgs)

add_message_files(
  DIRECTORY msg
  FILES
  ImageMarker.msg
  InteractiveMarkerControl.msg
  InteractiveMarkerFeedback.msg
  InteractiveMarkerInit.msg
  InteractiveMarker.msg
  InteractiveMarkerPose.msg
  InteractiveMarkerUpdate.msg
  MarkerArray.msg
  Marker.msg
  MenuEntry.msg
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_project(visualization_msgs
  )

