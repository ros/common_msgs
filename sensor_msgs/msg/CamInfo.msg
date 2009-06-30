# This message defines meta information for a camera. It should be in a
# camera namespace and accompanied by up to 5 image topics named:
# 
# image_raw, image, image_color, image_rect, and image_rect_color

Header header

uint32 height
uint32 width

float64[5]  D # Distortion: k1, k2, t1, t2, k3
float64[9]  K # original camera matrix
float64[9]  R # rectification matrix
float64[12] P # projection/camera matrix

# Should put exposure, gain, etc. information here as well
