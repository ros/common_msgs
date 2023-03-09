# This expresses a vector in free space with uncertainty.

Vector3 vector

# Row-major representation of the 3x3 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are: (x, y, z)
float64[9] covariance
