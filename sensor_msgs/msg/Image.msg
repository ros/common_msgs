Header header        # Header

# These are the two most popular image types, so make special synonyms for them
uint32 TYPE_MONO8=0
uint32 TYPE_BGR8=16

uint32 TYPE_8UC1=0
uint32 TYPE_8UC2=8
uint32 TYPE_8UC3=16
uint32 TYPE_8UC4=24
uint32 TYPE_8SC1=1
uint32 TYPE_8SC2=9
uint32 TYPE_8SC3=17
uint32 TYPE_8SC4=25
uint32 TYPE_16UC1=2
uint32 TYPE_16UC2=10
uint32 TYPE_16UC3=18
uint32 TYPE_16UC4=26
uint32 TYPE_16SC1=3
uint32 TYPE_16SC2=11
uint32 TYPE_16SC3=19
uint32 TYPE_16SC4=27
uint32 TYPE_32SC1=4
uint32 TYPE_32SC2=12
uint32 TYPE_32SC3=20
uint32 TYPE_32SC4=28
uint32 TYPE_32FC1=5
uint32 TYPE_32FC2=13
uint32 TYPE_32FC3=21
uint32 TYPE_32FC4=29
uint32 TYPE_64FC1=6
uint32 TYPE_64FC2=14
uint32 TYPE_64FC3=22
uint32 TYPE_64FC4=30

uint32 type           # Type of elements, and channels.  Encoded as in OpenCV's TYPE_8UC1 .. TYPE_64FC4.
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)
uint32 height         # image height, that is, number of rows
uint32 width          # image width, that is, number of columns
uint8 is_bigendian    # is this data bigendian
