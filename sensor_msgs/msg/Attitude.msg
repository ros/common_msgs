# This message represents 3D attitude

int8 ENU=1
int8 NED=2
int8 ENU_FLOATING=3
int8 NED_FLOATING=4

std_msgs/Header header

geometry_msgs/Quaternion orientation
float64[9] orientation_covariance # Row major about x, y, z axes

uint8 reference_frame_type # NED, ENU, NED_FLOATING, ENU_FLOATING
string sensor_frame # The coordinate frame in the system of the observation.
