project(nav_msgs)
find_package(geometry_msgs)

include_directories(include)

generate_msgs(${PROJECT_NAME}
  PATH msg

  MESSAGES 
  msg/GridCells.msg    msg/OccupancyGrid.msg  msg/Path.msg
  msg/MapMetaData.msg  msg/Odometry.msg

  SERVICES
  srv/GetMap.srv	srv/GetPlan.srv

  DEPENDENCIES std_msgs geometry_msgs 
  )

#add_library(${PROJECT_NAME} 
#  src/image_encodings.cpp src/point_cloud_conversion.cpp src/distortion_models.cpp
#  )
