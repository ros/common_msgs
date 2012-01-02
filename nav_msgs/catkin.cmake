project(nav_msgs)
find_package(geometry_msgs)

include_directories(include)

add_message_files(
  DIRECTORY msg
  FILES GridCells.msg OccupancyGrid.msg Path.msg MapMetaData.msg Odometry.msg)

add_service_files(
  DIRECTORY srv
  FILES GetMap.srv GetPlan.srv)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_project(${PROJECT_NAME}
  VERSION 0.0.1
  MSG_DIRS msg
  )
