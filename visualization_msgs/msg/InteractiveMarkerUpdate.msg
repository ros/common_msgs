# Sequence number.
# The client will use this to detect if it has missed an update.
uint64 seq_num

#Note: No guarantees on the order of processing.
#      Contents must be kept consistent by sender.

#Markers to be added or updated
InteractiveMarker[] markers

#Poses of markers that should be moved
InteractiveMarkerPose[] poses

#Names of markers that should be areased
string[] erases