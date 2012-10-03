cmake_minimum_required(VERSION 2.8.3)
project(actionlib_msgs)
find_package(catkin REQUIRED COMPONENTS genmsg std_msgs)
catkin_package(CFG_EXTRAS actionlib_msgs-extras.cmake)

add_message_files(
  DIRECTORY msg
  FILES
  GoalID.msg
  GoalStatus.msg
  GoalStatusArray.msg)
generate_messages(DEPENDENCIES std_msgs)

# install the .action -> .msg generator
install(PROGRAMS scripts/genaction.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# install the legacy rosbuild cmake support
install(FILES cmake/actionbuild.cmake
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake)
