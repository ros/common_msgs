Header header        # Header

# The legal values for encoding are in file src/image_encodings.cpp
# If you want to standardize a new string format, join ros-users@lists.sourceforge.net and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size -- taken from the list of strings in src/image_encodings.cpp
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)
uint32 height         # image height, that is, number of rows
uint32 width          # image width, that is, number of columns
uint8 is_bigendian    # is this data bigendian
