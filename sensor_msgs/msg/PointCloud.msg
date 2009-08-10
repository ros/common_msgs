#This message holds a collection of 3d points, plus optional additional information about each point.
#Each Point32 should be interpreted as a 3d point in the frame given in the header

Header header
geometry_msgs/Point32[] points  #Array of 3d points
ChannelFloat32[] channels       #Each channel should have the same number of elements as points array, and the data in each channel should correspond 1:1 with each point
