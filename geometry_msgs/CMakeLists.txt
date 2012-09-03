project(geometry_msgs)
catkin_project(geometry_msgs)

add_message_files(
  DIRECTORY msg
  FILES
  Point.msg
  Point32.msg
  PointStamped.msg
  Polygon.msg
  PolygonStamped.msg
  Pose2D.msg
  PoseArray.msg
  Pose.msg
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
