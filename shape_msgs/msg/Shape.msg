uint16 SPHERE=0
uint16 BOX=1
uint16 CYLINDER=2
uint16 CONE=3
uint16 MESH=4
uint16 PLANE=5

uint16 type


#### define sphere, box, cylinder, cone, plane ####
# the origin of each shape is considered at the shape's center unless specified otherwise

# for sphere
# radius := dimensions[0]

# for box
# size_x := dimensions[0]
# size_y := dimensions[1]
# size_z := dimensions[2]

# for cylinder
# radius := dimensions[0] (of the base)
# length := dimensions[1]
# the length is along the Z axis

# for cone (the origin is at the center of the base in this case)
# radius := dimensions[0] (of the base)
# length := dimensions[1]
# the length is along the Z axis

# for plane ax + by + cz + d = 0 (there is no notion of center in this case)
# a := dimensions[0]
# b := dimensions[1]
# c := dimensions[2]
# d := dimensions[3]

float64[] dimensions


#### define mesh (dimensions[] is not used for mesh) ####

# list of triangles; triangle k is defined by the vertices located
# at indices triangles[3k], triangles[3k+1], triangles[3k+2]
int32[] triangles
geometry_msgs/Point[] vertices
