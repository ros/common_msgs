cmake_minimum_required(VERSION 2.8)
project(common_msgs)

find_package(catkin REQUIRED COMPONENTS genmsg std_msgs)

catkin_stack()

foreach(subdir
    geometry_msgs
    sensor_msgs
    nav_msgs
    actionlib_msgs
    visualization_msgs
    stereo_msgs
    diagnostic_msgs
    trajectory_msgs
    shape_msgs
    )

  add_subdirectory(${subdir})
endforeach()
