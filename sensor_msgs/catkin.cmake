project(sensor_msgs)
find_package(geometry_msgs)

include_directories(include)

generate_msgs(${PROJECT_NAME}
  PATH msg

  MESSAGES 
  msg/CameraInfo.msg	 msg/JoyFeedbackArray.msg  msg/PointCloud2.msg
  msg/ChannelFloat32.msg	 msg/JoyFeedback.msg	   msg/PointCloud.msg
  msg/CompressedImage.msg  msg/Joy.msg		   msg/PointField.msg
  msg/Image.msg		 msg/LaserScan.msg	   msg/Range.msg
  msg/Imu.msg		 msg/NavSatFix.msg	   msg/RegionOfInterest.msg
  msg/JointState.msg	 msg/NavSatStatus.msg

  SERVICES
  srv/SetCameraInfo.srv

  DEPENDENCIES std_msgs geometry_msgs 
  )

install_cmake_infrastructure(${PROJECT_NAME}
  VERSION 0.0.1
  MSG_DIRS msg
  )

#add_library(${PROJECT_NAME} 
#  src/image_encodings.cpp src/point_cloud_conversion.cpp src/distortion_models.cpp
#  )
