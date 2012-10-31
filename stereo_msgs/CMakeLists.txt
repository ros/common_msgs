cmake_minimum_required(VERSION 2.8.3)
project(stereo_msgs)

find_package(catkin REQUIRED COMPONENTS genmsg sensor_msgs std_msgs)

add_message_files(
  DIRECTORY msg
  FILES
  DisparityImage.msg)

generate_messages(DEPENDENCIES sensor_msgs std_msgs)

catkin_package()
