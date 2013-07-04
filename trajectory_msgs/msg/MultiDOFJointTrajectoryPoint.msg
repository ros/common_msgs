# Each multi-dof joint can specify a transform (up to 6 DOF)
geometry_msgs/Transform[] transforms

# There can be a velocity specified for the origin of the joint 
geometry_msgs/Vector3[] velocities

# There can be an acceleration specified for the origin of the joint 
geometry_msgs/Vector3[] accelerations

duration time_from_start
