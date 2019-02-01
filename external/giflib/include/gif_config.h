/* config.h generated for CHAI3D */

#define HAVE_DLFCN_H        1
#define HAVE_FCNTL_H        1
#define HAVE_MEMORY_H       1
#define HAVE_STDARG_H       1
#define HAVE_STDLIB_H       1
#define HAVE_STRDUP         1
#define HAVE_STRINGS_H      1
#define HAVE_STRING_H       1
#define HAVE_SYS_STAT_H     1
#define HAVE_SYS_TYPES_H    1
#define HAVE_U_INT32_T      1
#define HAVE_VPRINTF        1

#define STDC_HEADERS        1
#define PROTOTYPES          1
#define __PROTOTYPES        1

/* OS specific settings */
#if !(defined (WIN32) | defined (WIN64))
  #define HAVE_INTTYPES_H 1
  #define HAVE_STDINT_H 1
  #define HAVE_UNISTD_H 1
  #define HAVE_X11_XLIB_H 1
  #define HAVE_X11_XUTIL_H 1
  #define UINT32 u_int32_t
#else
  #undef HAVE_UINT32_T
  #define UINT32 unsigned int
#endif
