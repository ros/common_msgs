project(sensor_msgs)
catkin_project(sensor_msgs
  LIBRARIES sensor_msgs
  INCLUDE_DIRS include
  )

include_directories(include)

add_message_files(
  DIRECTORY msg
  FILES
  CameraInfo.msg      JoyFeedbackArray.msg PointCloud2.msg
  ChannelFloat32.msg  JoyFeedback.msg      PointCloud.msg
  CompressedImage.msg Joy.msg              PointField.msg
  Image.msg           LaserScan.msg        Range.msg
  Imu.msg             NavSatFix.msg        RegionOfInterest.msg
  JointState.msg      NavSatStatus.msg
)

add_service_files(
  DIRECTORY srv
  FILES SetCameraInfo.srv
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)
find_package(Boost COMPONENTS format)

find_package(ROS COMPONENTS
  cpp_common
  roscpp_serialization
  rostime
  roscpp_traits
  )
include_directories(${ROS_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS} )

add_library(sensor_msgs SHARED
  src/image_encodings.cpp src/point_cloud_conversion.cpp src/distortion_models.cpp
)
target_link_libraries(sensor_msgs
  ${ROS_LIBRARIES}
  ${Boost_LIBRARIES}
  )

add_dependencies(sensor_msgs sensor_msgs_gencpp)
install(TARGETS sensor_msgs
        LIBRARY DESTINATION lib)
