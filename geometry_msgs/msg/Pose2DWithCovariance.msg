# This expresses a position and orientation on a 2D manifold.

float64 x
float64 y
float64 theta


# Row-major representation of the 3x3 covariance matrix
# In order, the parameters are:
# (x, y, theta)
float64[9] covariance