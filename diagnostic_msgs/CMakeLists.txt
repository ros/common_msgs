project(diagnostic_msgs)

add_message_files(
  DIRECTORY msg
  FILES
  DiagnosticArray.msg
  DiagnosticStatus.msg
  KeyValue.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  SelfTest.srv
  )

generate_messages(DEPENDENCIES std_msgs)

catkin_project(diagnostic_msgs
  )

