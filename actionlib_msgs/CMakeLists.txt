project(actionlib_msgs)

include_directories(include)

add_message_files(
  DIRECTORY msg
  FILES
  GoalID.msg GoalStatusArray.msg GoalStatus.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_project(actionlib_msgs
  CFG_EXTRAS actionlib_msgs-extras.cmake
  MSG_DIRS msg
  )
 
