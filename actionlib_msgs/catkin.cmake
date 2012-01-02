project(actionlib_msgs)

include_directories(include)

add_message_files(
  DIRECTORY msg
  FILES 
  GoalID.msg GoalStatusArray.msg GoalStatus.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_project(actionlib_msgs
  VERSION 0.0.0
  CFG_EXTRAS actionlib_msgs-extras.cmake
  MSG_DIRS msg
  )

#add_library(${PROJECT_NAME} 
#  src/image_encodings.cpp src/point_cloud_conversion.cpp src/distortion_models.cpp
#  )
