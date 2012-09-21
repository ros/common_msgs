project(trajectory_msgs)
catkin_package_export()

add_message_files(
  DIRECTORY msg
  FILES
  JointTrajectory.msg
  JointTrajectoryPoint.msg)

generate_messages(DEPENDENCIES std_msgs)
