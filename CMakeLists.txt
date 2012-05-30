cmake_minimum_required(VERSION 2.8)
project(common_msgs)

find_package(catkin REQUIRED)
find_package(genmsg REQUIRED)
find_package(std_msgs REQUIRED)

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
    )

  add_subdirectory(${subdir})
endforeach()

# install all manifests
install_matching_to_share(manifest.xml)

install(FILES stack.xml
        DESTINATION share/common_msgs)
