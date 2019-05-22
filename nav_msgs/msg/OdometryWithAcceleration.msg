# This represents an estimate of a position, velocity, and acceleration in free space.  
# The pose in this message should be specified in the coordinate frame given by header.frame_id.
# The velocity in this message should be specified in the coordinate frame given by the child_frame_id.
# The acceleration in this message should be specified in the coordinate frame given by the child_frame_id.
#
# If the covariance of the measurement is known, it should be filled in (if all you know is the
# variance of each measurement, e.g. from a datasheet, just put those along the diagonal).
# A covariance matrix of all zeros will be interpreted as "covariance unknown", and to use the
# data a covariance will have to be assumed or gotten from some other source.
#
# If you have no estimate for one of the data elements, please set _the respective diagonal_ element
# of the associated covariance matrix to -1.
# If you are interpreting this message, please check for a value of -1 in the _diagonal_ elements
# of each covariance matrix, and disregard the associated estimates.

Header header
string child_frame_id
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist
geometry_msgs/AccelWithCovariance accel
