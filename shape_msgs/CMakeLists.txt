cmake_minimum_required(VERSION 2.8)
project(shape_msgs)
find_package(catkin REQUIRED COMPONENTS geometry_msgs std_msgs)
catkin_package()

add_message_files(
  DIRECTORY msg
  FILES
  Mesh.msg
  MeshTriangle.msg
  Plane.msg
  SolidPrimitive.msg)

generate_messages(DEPENDENCIES geometry_msgs std_msgs)
