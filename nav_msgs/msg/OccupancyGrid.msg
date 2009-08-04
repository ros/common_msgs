# A 2-D grid map, in which each cell represents the probability of
# occupancy.  Occupancy values are integers in the range [0,100], or -1 
# for unknown.

#MetaData for the map
MapMetaData info

# The map data, in row-major order, starting with (0,0).  Occupancy
# probabilities are in the range [0,100].  Unknown is -1.
byte[] data
