cmake_minimum_required(VERSION 2.8)
project(trajectory_msgs)
find_package(catkin REQUIRED COMPONENTS genmsg std_msgs)
catkin_package()

add_message_files(
  DIRECTORY msg
  FILES
  JointTrajectory.msg
  JointTrajectoryPoint.msg)

generate_messages(DEPENDENCIES std_msgs)
