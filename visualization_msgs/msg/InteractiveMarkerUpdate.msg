# Identifying string. Must be unique in the topic namespace
# that this server works on.
string server_id

# Sequence number.
# The client will use this to detect if it has missed an update.
uint64 seq_num

# What purpose serves this message?
# INIT: Will cause the client to erase all markers previously 
#       received from this server. Initialize sequence number.
# UPDATE: Incremental update to previous state. 
#         The sequence number must be 1 higher than for
#         the previous update.
# KEEP_ALIVE: Indicates the that the server is still living.
#             The sequence number does not increase.
byte KEEP_ALIVE = 0
byte UPDATE = 1
byte INIT = 2

byte type

#Note: No guarantees on the order of processing.
#      Contents must be kept consistent by sender.

#Markers to be added or updated
InteractiveMarker[] markers

#Poses of markers that should be moved
InteractiveMarkerPose[] poses

#Names of markers to be erased
string[] erases
