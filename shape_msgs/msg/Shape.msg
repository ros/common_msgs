# A shape is a GeometricPrimitive or a Mesh

uint8 PRIMITIVE=0
uint8 MESH=1

uint8 type

GeometricPrimitive primitive
Mesh mesh

# scaling for the shape (on x, y and z)
geometry_msgs/Vector3 scale
