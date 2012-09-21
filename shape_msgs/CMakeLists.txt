project(shape_msgs)

catkin_package_export()

add_message_files(
  DIRECTORY msg
  FILES
  Mesh.msg
  MeshTriangle.msg
  Plane.msg
  SolidPrimitive.msg)

find_package(geometry_msgs)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)
