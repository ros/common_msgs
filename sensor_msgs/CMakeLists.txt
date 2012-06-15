project(sensor_msgs)
# We want boost/format.hpp, which isn't in its own component.
find_package(Boost REQUIRED)
catkin_project(sensor_msgs
  LIBRARIES ${Boost_LIBRARIES}
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
  JointState.msg      NavSatStatus.msg     TimeReference.msg
)

add_service_files(
  DIRECTORY srv
  FILES SetCameraInfo.srv
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

install(DIRECTORY include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE)
