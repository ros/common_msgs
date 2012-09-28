cmake_minimum_required(VERSION 2.8)
project(diagnostic_msgs)
find_package(catkin REQUIRED COMPONENTS genmsg std_msgs)
catkin_package()

add_message_files(
  DIRECTORY msg
  FILES
  DiagnosticArray.msg
  DiagnosticStatus.msg
  KeyValue.msg)

add_service_files(
  DIRECTORY srv
  FILES
  SelfTest.srv)

generate_messages(DEPENDENCIES std_msgs)
