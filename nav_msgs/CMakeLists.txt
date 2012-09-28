cmake_minimum_required(VERSION 2.8)
project(nav_msgs)
find_package(catkin REQUIRED COMPONENTS genmsg geometry_msgs std_msgs)
catkin_package()

add_message_files(
  DIRECTORY msg
  FILES
  GridCells.msg
  MapMetaData.msg
  OccupancyGrid.msg
  Odometry.msg
  Path.msg)

add_service_files(
  DIRECTORY srv
  FILES
  GetMap.srv
  GetPlan.srv)

generate_messages(DEPENDENCIES geometry_msgs std_msgs)
