# Pixel encoding of raw camera output
# This is *only* of interest if you are writing a camera driver node, or a
# node which specifically processes raw camera data (i.e. stereoproc).
# Other nodes can assume images are monochrome, or (if desired) BGR color.

# bpp = bits per pixel
uint8 MONO_8         =0  # 8 bpp, monochrome
uint8 MONO_16        =1  # 16 bpp, monochrome
uint8 BAYER_RGGB_8   =2  # 8 bpp, raw un-interpolated data
uint8 BAYER_RGGB_16  =3  # 16 bpp, raw un-interpolated data
uint8 BAYER_GBRG_8   =4  # 8 bpp, raw un-interpolated data
uint8 BAYER_GBRG_16  =5  # 16 bpp, raw un-interpolated data
uint8 BAYER_GRBG_8   =6  # 8 bpp, raw un-interpolated data
uint8 BAYER_GRBG_16  =7  # 16 bpp, raw un-interpolated data
uint8 BAYER_BGGR_8   =8  # 8 bpp, raw un-interpolated data
uint8 BAYER_BGGR_16  =9  # 16 bpp, raw un-interpolated data
uint8 RGB_8          =10 # 24 bpp, RGB color
uint8 RGB_16         =11 # 48 bpp, RGB color
uint8 BGR_8          =12 # 24 bpp, BGR color
uint8 BGR_16         =13 # 48 bpp, BGR color
uint8 RGBA_8         =14 # 32 bpp, RGBA color
uint8 RGBA_16        =15 # 64 bpp, RGBA color
uint8 BGRA_8         =16 # 32 bpp, BGRA color
uint8 BGRA_16        =17 # 64 bpp, BGRA color
uint8 YUV411         =18 # 12 bpp, color
uint8 YUV422         =19 # 16 bpp, color
uint8 YUV444         =20 # 24 bpp, color

uint8 type
