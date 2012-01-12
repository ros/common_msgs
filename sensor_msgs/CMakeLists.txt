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

add_library(sensor_msgs SHARED
  src/image_encodings.cpp src/point_cloud_conversion.cpp src/distortion_models.cpp
)
install(TARGETS sensor_msgs
        LIBRARY DESTINATION lib)
