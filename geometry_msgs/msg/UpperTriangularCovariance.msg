# Row-major representation of 6x6 velocity cross-covariance matrix in an upper right triangular
# Order: vx, vy, vz, rotation rate about X axis, rotation rate about Y axis, rotation rate about Z axis

# If linear velocity covariance invalid/unknown, first cell is NaN
# If angular velocity covariance invalid/unknown, 16th cell is NaN

# Covariance matrix index constants
uint8 MATRIX_LINEAR_X_VARIANCE=0
uint8 MATRIX_LINEAR_Y_VARIANCE=6
uint8 MATRIX_LINEAR_Z_VARIANCE=11
uint8 MATRIX_ANGULAR_X_VARIANCE=15
uint8 MATRIX_ANGULAR_Y_VARIANCE=18
uint8 MATRIX_ANGULAR_Z_VARIANCE=20

# Convenient aliases for common names.
uint8 MATRIX_ROLL_VARIANCE=15
uint8 MATRIX_PITCH_VARIANCE=18
uint8 MATRIX_YAW_VARIANCE=20

# Constants to index to cells to check for invalid/unknown data
uint8 MATRIX_LINEAR_VALID_CELL=0
uint8 MATRIX_ANGULAR_VALID_CELL=15

# Raw Storage
float32[21] covariance