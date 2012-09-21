project(stereo_msgs)
catkin_package_export()

add_message_files(
  DIRECTORY msg
  FILES
  DisparityImage.msg)

generate_messages(DEPENDENCIES std_msgs sensor_msgs)
