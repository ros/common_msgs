project(shape_msgs)

catkin_project(shape_msgs
  INCLUDE_DIRS include
)

add_message_files(
  DIRECTORY msg
  FILES Shape.msg
  )

find_package(geometry_msgs)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

