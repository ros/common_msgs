cmake_minimum_required(VERSION 2.8)
project(common_msgs)

find_package(catkin REQUIRED)
find_package(genmsg REQUIRED)
find_package(std_msgs REQUIRED)

foreach(subdir
    geometry_msgs
    sensor_msgs
    nav_msgs
    actionlib_msgs
    )

  add_subdirectory(${subdir})
endforeach()

catkin_package(common_msgs)
