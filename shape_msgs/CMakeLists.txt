project(shape_msgs)

catkin_project(shape_msgs
  INCLUDE_DIRS include
  LIBRARIES shape_msgs)

include_directories(include)

add_message_files(
  DIRECTORY msg
  FILES Shape.msg StaticShape.msg
  )

find_package(geometry_msgs)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

find_package(rostime)
include_directories(${rostime_INCLUDE_DIRS})

add_library(shape_msgs 
  src/shape_extents.cpp src/shape_to_marker.cpp
)
add_dependencies(shape_msgs shape_msgs_gencpp)

install(TARGETS shape_msgs
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib)

install(DIRECTORY include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE)
