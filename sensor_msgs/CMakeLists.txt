project(sensor_msgs)

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

catkin_project(sensor_msgs
  )

message("TODO: re-enable sensor_msgs library")
#add_library(${PROJECT_NAME}
#  src/image_encodings.cpp src/point_cloud_conversion.cpp src/distortion_models.cpp
#  )
