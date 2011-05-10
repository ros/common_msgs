# Note: when sending an interactive marker array, it is important
# to assign names to each contained marker, as markers with 
# the same name will overwrite each other.
#
# When sending a new marker array on the same topic, old markers
# will not be removed. Instead, they will be overwritten by new markers
# of the same name.
#
# To delete a previously sent marker, send an empty marker 
# (without controls) that has the same name.

InteractiveMarker[] markers
