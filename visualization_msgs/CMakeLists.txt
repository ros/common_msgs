cmake_minimum_required(VERSION 2.8)
project(visualization_msgs)

find_package(catkin REQUIRED COMPONENTS geometry_msgs std_msgs)

catkin_package()

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

generate_messages(DEPENDENCIES geometry_msgs std_msgs)
