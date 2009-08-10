#This message is used by the PointCloud message to hold optional data associated with each point in the cloud
#The length of the values array should be the same as the length of the points array in the PointCloud, and each value should be associated with the corresponding point

string name       #channel name should give semantics of the channel (e.g. "intensity" instead of "value")
float32[] values  #values array should have same number of elements as the associated PointCloud
