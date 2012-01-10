project(stereo_msgs)

add_message_files(
  DIRECTORY msg
  FILES
  DisparityImage.msg
)

generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_project(stereo_msgs
  )

