project(sensor_msgs)
# We want boost/format.hpp, which isn't in its own component.
find_package(Boost REQUIRED)
catkin_project(sensor_msgs
  LIBRARIES ${Boost_LIBRARIES}
  INCLUDE_DIRS include)

include_directories(include)

add_message_files(
  DIRECTORY msg
  FILES
  CameraInfo.msg
  ChannelFloat32.msg
  CompressedImage.msg
  Image.msg
  Imu.msg
  JointState.msg
  Joy.msg
  JoyFeedback.msg
  JoyFeedbackArray.msg
  LaserScan.msg
  NavSatFix.msg
  NavSatStatus.msg
  PointCloud.msg
  PointCloud2.msg
  PointField.msg
  Range.msg
  RegionOfInterest.msg
  TimeReference.msg)

add_service_files(
  DIRECTORY srv
  FILES SetCameraInfo.srv)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

install(DIRECTORY include/
  DESTINATION ${PROJECT_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")
