cmake_minimum_required(VERSION 2.8)
message("ROSBUILD DOT CMAKE!  YAY!")
find_package(rosbuild)
find_package(std_msgs)
find_package(genmsg)

foreach(subdir
    geometry_msgs
    sensor_msgs
    nav_msgs
    )

  message("common_msgs: ${subdir}")
  add_subdirectory(${subdir})
endforeach()