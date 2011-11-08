cmake_minimum_required(VERSION 2.8)
project(geometry_msgs)

find_package(rosbuild)
find_package(std_msgs)
find_package(genmsg)

#add_subdirectory(msg)
set(ROSBUILD_LOG 9)
generate_msgs(${PROJECT_NAME}
  PATH msg
  MESSAGES 
  msg/Point32.msg			   msg/QuaternionStamped.msg
  msg/Point.msg			   msg/Transform.msg
  msg/PointStamped.msg		   msg/TransformStamped.msg
  msg/Polygon.msg			   msg/Twist.msg
  msg/PolygonStamped.msg		   msg/TwistStamped.msg
  msg/Pose2D.msg			   msg/TwistWithCovariance.msg
  msg/PoseArray.msg		   msg/TwistWithCovarianceStamped.msg
  msg/Pose.msg			   msg/Vector3.msg
  msg/PoseStamped.msg		   msg/Vector3Stamped.msg
  msg/PoseWithCovariance.msg	   msg/Wrench.msg
  msg/PoseWithCovarianceStamped.msg  msg/WrenchStamped.msg
  msg/Quaternion.msg
  DEPENDENCIES std_msgs
  )

install_cmake_infrastructure(${PROJECT_NAME}
  VERSION 0.0.1
  MSG_DIRS msg
  )

