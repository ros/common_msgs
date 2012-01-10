project(geometry_msgs)

add_message_files(
  DIRECTORY msg
  FILES
  Point32.msg QuaternionStamped.msg
  Point.msg Transform.msg
  PointStamped.msg TransformStamped.msg
  Polygon.msg Twist.msg
  PolygonStamped.msg TwistStamped.msg
  Pose2D.msg TwistWithCovariance.msg
  PoseArray.msg TwistWithCovarianceStamped.msg
  Pose.msg Vector3.msg
  PoseStamped.msg Vector3Stamped.msg
  PoseWithCovariance.msg Wrench.msg
  PoseWithCovarianceStamped.msg WrenchStamped.msg
  Quaternion.msg
  )

generate_messages(DEPENDENCIES std_msgs)

catkin_project(geometry_msgs
  )
