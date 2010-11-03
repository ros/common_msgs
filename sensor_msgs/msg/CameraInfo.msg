# This message defines meta information for a camera. It should be in a
# camera namespace on topic "camera_info" and accompanied by up to five
# image topics named:
#
#   image_raw - raw data from the camera driver, possibly Bayer encoded
#   image            - monochrome, distorted
#   image_color      - color, distorted
#   image_rect       - monochrome, rectified
#   image_rect_color - color, rectified
#
# The image_pipeline contains packages (image_proc, stereo_image_proc) for
# producing the four processed image topics from image_raw and camera_info.
# The meaning of the camera parameters are described in detail at
# http://www.ros.org/wiki/image_pipeline/CameraInfo.
#
# The image_geometry package provides a user-friendly interface to common
# operations using this meta information. If you want to, e.g., project a
# 3d point into image coordinates, we strongly recommend using image_geometry.
#
# If the camera is uncalibrated, the matrices D, K, R, P should be left zeroed
# out. In particular, clients may assume that K[0] == 0.0 indicates an
# uncalibrated camera.

##########################
# Image acquisition info #
##########################

# Time of image acquisition, camera coordinate frame ID
Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of camera
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into the plane of the image

##########################################
# Calibration Parameters
##########################################

# Camera resolution in pixels
uint32 height
uint32 width

##########################################
# Internal parameters                    #
# Used for warping to:                   #
#  1. An undistorted image (just D and K)#
#  2. A rectified image (D, K, R)        #
# Users should not normally need these   #
##########################################

# Distortion parameters: k1, k2, t1, t2, k3
# These model radial and tangential distortion of the camera.
string distortion_model
float64[]  D # 5x1 vector

# Intrinsic camera matrix for the raw (distorted) images
# Projects 3D points in the camera coordinate frame to 2D pixel
# coordinates using the focal lengths (fx, fy) and principal point
# (cx, cy):
#     [fx  0 cx]
# K = [ 0 fy cy]
#     [ 0  0  1]

float64[9]  K # 3x3 row-major matrix


# Rectification matrix (stereo cameras only)
# A homography which takes an image to the ideal stereo image plane
# so that epipolar lines in both stereo images are parallel.
float64[9]  R # 3x3 row-major matrix

######################################
# Projection/camera matrix           #
# This is a 3x4 projection matrix    #
#   for going from 3D to 2D coords   #
######################################

# Projection/camera matrix
# By convention, this matrix specifies the intrinsic (camera)
#   matrix of the processed (rectified) image.
# Upper 3x3 portion is the normal camera intrinsic matrix for
#   the rectified image
# Projects 3D points in the camera coordinate frame to 2D pixel
# coordinates using the focal lengths (fx, fy) and principal point
# (cx, cy):
#              [fx  0 cx]
# P[1:3,1:3] = [ 0 fy cy]
#              [ 0  0  1]
# For the right camera of a stereo pair, P[4,1] is the position
#  of the right camera center in the left camera's frame, times
#  the focal length fx.
# 
# Given a 3D point q=[XYZ]^T in the left camera frame, the projection
#  of the point onto the image is given by [uvw]^T = Pq, with x=u/w and y=v/w.
#  This holds for both left and right images of a stereo pair.  
#  For monocular cameras and the left image of a stereo pair, P[4,1] is always 0.

float64[12] P # 3x4 row-major matrix

uint32 binning_x
uint32 binning_y

# Region of interest (subwindow of full camera resolution), if applicable
RegionOfInterest roi

