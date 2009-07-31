# This message defines meta information for a stereo pair. It should
# be in a stereo namespace and accompanied by 2 camera namespaces, and
# a disparity image, named:
#
# left, right, and disparity, respectively
#
# The meaning of the stereo parameters are described in detail at
# http://pr.willowgarage.com/wiki/Camera_Calibration.

Header header

# Resolution in pixels
uint32 height
uint32 width

# Pose of right camera in the left camera coordinate frame
float64[3]  T  # translation vector
float64[3]  Om # rotation, encoded as a Rodigues vector

# Reprojection matrix
float64[16] RP # 4x4 row-major matrix
