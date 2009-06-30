# This message defines meta information for a stereo pair. It should
# be in a stereo namespace and accompanied by 2 camera namespaces, and
# a disparity image, named:
#
# left, right, and disparity, respectively

Header header

uint32 height
uint32 width

float64[3]  T  # Pose of right camera in left camera coords
float64[3]  Om # rotation vector
float64[16] RP # Reprojection Matrix
