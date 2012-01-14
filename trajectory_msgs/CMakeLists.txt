project(trajectory_msgs)

add_message_files(
  DIRECTORY msg
  FILES
  JointTrajectory.msg
  JointTrajectoryPoint.msg
  )

generate_messages(DEPENDENCIES std_msgs)

catkin_project(trajectory_msgs
  )

