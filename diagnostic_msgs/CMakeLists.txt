project(diagnostic_msgs)
catkin_package_export()

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
