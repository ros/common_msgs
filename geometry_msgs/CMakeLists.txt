cmake_minimum_required(VERSION 2.8.3)
project(geometry_msgs)

find_package(catkin REQUIRED COMPONENTS genmsg std_msgs)

add_message_files(
  DIRECTORY msg
  FILES
  Point.msg
  Point32.msg
  PointStamped.msg
  Polygon.msg
  PolygonStamped.msg
  Pose2D.msg
  Pose.msg
  PoseArray.msg
  PoseStamped.msg
  PoseWithCovariance.msg
  PoseWithCovarianceStamped.msg
  Quaternion.msg
  QuaternionStamped.msg
  Transform.msg
  TransformStamped.msg
  Twist.msg
  TwistStamped.msg
  TwistWithCovariance.msg
  TwistWithCovarianceStamped.msg
  Vector3.msg
  Vector3Stamped.msg
  Wrench.msg
  WrenchStamped.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package()
