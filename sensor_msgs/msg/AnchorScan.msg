# This is a message to hold the fixed anchors' (UWB sensors) information at known coordinates
# and Time Difference of Arrival (TDOA) values.
#
# The indoor positioning systems developed using UWB signals include multiple sensors
# that transmit UWB signals in an environment.
#
# Calculations such as position, anchor selection etc. are made by using TDOA measurements 
# from these sensors and coordinate information of the sensors.
#
# With the defined AnchorScan message, it allows the publish of this information
# from the sensors via any developed firmware.

Header header               

int32[] AnchorID                # ID of fixed sensors.

float64[] x                     # x coordinates of fixed sensors respectively in meter
float64[] y                     # y coordinates of fixed sensors respectively in meter
float64[] z                     # z coordinates of fixed sensors respectively in meter

float64[] tdoa_of_anchors       # TDOA values of UWB signals according to sensor with minimum ID.
