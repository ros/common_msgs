# This message defines meta information for a camera. It should be in a
# camera namespace and accompanied by up to 5 image topics named:
# 
# image_raw, image, image_color, image_rect, and image_rect_color
#
# The meaning of the camera parameters are described in detail at
# http://pr.willowgarage.com/wiki/Camera_Calibration.

##########################
# Image acquisition info #
##########################

# Time of image acquisition, camera coordinate frame ID
Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of camera
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image

# Camera resolution in pixels
uint32 height
uint32 width

# Region of interest (subwindow of full camera resolution), if applicable
RegionOfInterest roi

########################
# Intrinsic parameters #
########################

# Distortion parameters: k1, k2, t1, t2, k3
# These model radial and tangential distortion of the camera.
float64[5]  D # 5x1 vector

# Original camera matrix
# Projects 3D points in the camera coordinate frame to 2D pixel
# coordinates using the focal lengths (fx, fy) and principal point
# (cx, cy):
#     [fx  0 cx]
# K = [ 0 fy cy]
#     [ 0  0  1]
float64[9]  K # 3x3 row-major matrix

########################
# Extrinsic parameters #
########################

# Rectification matrix (stereo cameras only)
# A homography which takes an image to the ideal stereo image plane
# so that epipolar lines in both stereo images are parallel.
float64[9]  R # 3x3 row-major matrix

# Projection/camera matrix
# Projects 3D points in a world coordinate frame to 2D pixel coordinates.
float64[12] P # 3x4 row-major matrix
