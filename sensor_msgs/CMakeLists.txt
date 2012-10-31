cmake_minimum_required(VERSION 2.8.3)
project(sensor_msgs)

find_package(catkin REQUIRED COMPONENTS genmsg geometry_msgs std_msgs)
# We want boost/format.hpp, which isn't in its own component.
find_package(Boost REQUIRED)

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

generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS Boost)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")
