# Define box, sphere, cylinder, cone 

uint16 BOX=1
uint16 SPHERE=2
uint16 CYLINDER=3
uint16 CONE=4

uint16 type

geometry_msgs/Vector3 dimensions

# For the BOX type, the X, Y, and Z dimensions are the length of the corresponding
# sides of the box.
#
# For the CYLINDER and CONE types, the center line is oriented along the Z axis.
# Therefore the Z component of dimensions gives the height of the cone or cylinder.
# The X component of dimensions gives the diameter of the base of the cone or
# cylinder.  Cone and cylinder primitives are defined to be circular, so the
# Y component is ignored.
#
# For the SPHERE type, only the X component is used, and it gives the diameter of
# the sphere.
#
# All shapes are defined to have their bounding boxes centered around 0,0,0.
