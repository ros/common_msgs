Header header        # Header

int32 TYPE_8UC1=0
int32 TYPE_8UC2=8
int32 TYPE_8UC3=16
int32 TYPE_8UC4=24
int32 TYPE_8SC1=1
int32 TYPE_8SC2=9
int32 TYPE_8SC3=17
int32 TYPE_8SC4=25
int32 TYPE_16UC1=2
int32 TYPE_16UC2=10
int32 TYPE_16UC3=18
int32 TYPE_16UC4=26
int32 TYPE_16SC1=3
int32 TYPE_16SC2=11
int32 TYPE_16SC3=19
int32 TYPE_16SC4=27
int32 TYPE_32SC1=4
int32 TYPE_32SC2=12
int32 TYPE_32SC3=20
int32 TYPE_32SC4=28
int32 TYPE_32FC1=5
int32 TYPE_32FC2=13
int32 TYPE_32FC3=21
int32 TYPE_32FC4=29
int32 TYPE_64FC1=6
int32 TYPE_64FC2=14
int32 TYPE_64FC3=22
int32 TYPE_64FC4=30

int32 type            # Type of elements, and channels.  Encoded as in OpenCV's TYPE_8UC1 .. TYPE_64FC4.
int32 step            # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)
int32 rows            # Number of rows
int32 cols            # Number of columns
uint8 is_bigendian    # is this data bigendian
