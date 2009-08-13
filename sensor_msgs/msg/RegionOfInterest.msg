# This message is used to specify a region of interest within an image
#
# When used to specify the ROI setting of the camera when the image was taken,
# the height and width fields should either match the height and width
# fields for the associated image or be zeroes to indicate that the full image
# was captured

uint32 x_offset  #Leftmost pixel of the ROI (0 if the left edge of the image is included in the ROI)
uint32 y_offset  #Topmost pixel of the ROI (0 if the top edge of the image is included in the ROI)
uint32 height    #Height of ROI
uint32 width     #Width of ROI
