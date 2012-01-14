project(actionlib_msgs)

add_message_files(
  DIRECTORY msg
  FILES
  GoalID.msg GoalStatusArray.msg GoalStatus.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_project(actionlib_msgs
  CFG_EXTRAS actionlib_msgs-extras.cmake
  )

install(FILES cmake/actionbuild.cmake
        DESTINATION share/actionlib_msgs/cmake)
install(PROGRAMS rosbuild/scripts/genaction.py
        DESTINATION share/actionlib_msgs/rosbuild/scripts)
