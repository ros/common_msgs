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

# install the .action -> .msg generator
install(PROGRAMS scripts/genaction.py
        DESTINATION share/actionlib_msgs/scripts)

# install the legacy rosbuild cmake support
install(FILES cmake/actionbuild.cmake
        DESTINATION share/actionlib_msgs/cmake)
