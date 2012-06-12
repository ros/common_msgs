byte SPHERE=0
byte BOX=1
byte CYLINDER=2
byte CONE=3
byte MESH=4
byte PLANE=5

byte type


#### define sphere, box, cylinder, cone, plane ####
# the origin of each shape is considered at the shape's center

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

# for cone
# radius := dimensions[0] (of the base)
# length := dimensions[1]
# the length is along the Z axis

# for plane ax + by + cz + d = 0
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
