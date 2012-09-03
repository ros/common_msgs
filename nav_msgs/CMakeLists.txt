project(nav_msgs)
find_package(catkin REQUIRED COMPONENTS geometry_msgs)
catkin_project(${PROJECT_NAME})

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

generate_messages(DEPENDENCIES std_msgs geometry_msgs)
