# This message defines meta information for a panorama image. It should
# be in the same namespace with a image topic.

#######################################################################
#                     Image acquisition info                          #
#######################################################################

# Time of image acquisition, panorama image coordinate frame ID
Header header    # Header timestamp should be acquisition time of image
                 # Header frame_id should be frame of panorama image
                 # origin of frame should be projection center
                 # +x should point to the front of a sphere
                 # +z should point to the top of a sphere

#######################################################################
#                      Image projection info                          #
#######################################################################

# The projection model from spherical view to a flat image.
# E.g. "Equirectangular"
string projection_model

#######################################################################
#                     Field of View Parameters                         #
#######################################################################

# Specifying field of view of a panorama image.
# theta, phi are azimuthal and polar angle of spherical coordinate.
float64 theta_min
float64 theta_max
float64 phi_min
float64 phi_max
