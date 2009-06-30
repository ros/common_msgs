Header header        # Header
string label         # Label for the image
string encoding      # Specifies the color encoding of the data
                     #   Acceptable values are:
                     #    1 channel types:
                     #     mono, bayer_rggb, bayer_gbrg, bayer_grbg, bayer_bggr
                     #    3 channel types:
                     #     rgb, bgr
                     #    4 channel types:
                     #     rgba, bgra, yuv422
                     #    6 channel types:
                     #     yuv411
                     #    N channel types:
                     #     other
string depth         # Specifies the depth of the data:
                     #   Acceptable values:
                     #    uint8, int8, uint16, int16, uint32, int32, uint64, int64, float32, float64

# Based on depth ONE of the following MultiArrays may contain data.
# The multi-array MUST have 3 dimensions, labeled as "height",
# "width", and "channel", though depending on usage the ordering of
# the dimensions may very.  Note that IPL Image convention will order
# these as: height, width, channel, which is the preferred ordering
# unless performance dictates otherwise.
#
# Height, width, and number of channels are specified in the dimension
# sizes within the appropriate MultiArray

std_msgs/UInt8MultiArray   uint8_data
std_msgs/Int8MultiArray    int8_data
std_msgs/UInt16MultiArray  uint16_data   
std_msgs/Int16MultiArray   int16_data
std_msgs/UInt32MultiArray  uint32_data
std_msgs/Int32MultiArray   int32_data
std_msgs/UInt64MultiArray  uint64_data
std_msgs/Int64MultiArray   int64_data
std_msgs/Float32MultiArray float32_data
std_msgs/Float64MultiArray float64_data
