project(actionlib_msgs)
catkin_project(actionlib_msgs
  CFG_EXTRAS actionlib_msgs-extras.cmake)

add_message_files(
  DIRECTORY msg
  FILES
  GoalID.msg
  GoalStatus.msg
  GoalStatusArray.msg)
generate_messages(DEPENDENCIES std_msgs)

# install the .action -> .msg generator
install(PROGRAMS scripts/genaction.py
  DESTINATION ${PROJECT_LIBEXEC_DESTINATION})

# install the legacy rosbuild cmake support
install(FILES cmake/actionbuild.cmake
  DESTINATION ${PROJECT_SHARE_DESTINATION}/cmake)
