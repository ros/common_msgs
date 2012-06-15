uint16 BOX=1
uint16 SPHERE=2
uint16 CYLINDER=3
uint16 PLANE=4
uint16 CONE=5

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

# for cone (the origin is at the middle of the cone's vertical axis, straight above the center of the base)
# radius := dimensions[0] (of the base)
# length := dimensions[1]
# the length is along the Z axis

# for plane ax + by + cz + d = 0 (there is no notion of center in this case)
# a := dimensions[0]
# b := dimensions[1]
# c := dimensions[2]
# d := dimensions[3]

float64[] dimensions
