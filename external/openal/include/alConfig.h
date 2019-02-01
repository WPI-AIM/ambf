/* config.h generated for CHAI3D */



#define ALSOFT_VERSION "1.15.1"

#define AL_API
#define ALC_API



#if defined(WIN32) | defined(WIN64)

#define strcasecmp _stricmp
#define strncasecmp _strnicmp
#if _MSC_VER < 1900
#define snprintf _snprintf
  #if _MSC_VER < 1800
  #define isfinite _finite
  #define isnan _isnan
  #endif
#endif

#define ALIGN(x) __declspec(align(x))
#ifdef __MINGW32__
#define align(x) aligned(x)
#endif

#define RESTRICT __restrict
#define HAVE__ALIGNED_MALLOC
#define HAVE_SSE
#define HAVE_MMDEVAPI
#define HAVE_DSOUND
#define HAVE_WINMM
#define HAVE_WAVE
#define HAVE_STAT
#define SIZEOF_LONG 4
#define SIZEOF_LONG_LONG 8
#define HAVE_STDINT_H
#define HAVE_WINDOWS_H
#define HAVE_XMMINTRIN_H
#define HAVE_MALLOC_H
#define HAVE_GUIDDEF_H
#define HAVE_FLOAT_H
#define HAVE__CONTROLFP

#ifndef WIN64
#define HAVE___CONTROL87_2
#endif

#endif



#ifdef LINUX

#define ALIGN(x) __attribute__((aligned(x)))

#define RESTRICT __restrict

#define HAVE_POSIX_MEMALIGN
#define HAVE_ALSA
#define HAVE_WAVE
#define HAVE_STAT
#define HAVE_LRINTF
#define HAVE_STRTOF
#define SIZEOF_LONG 4
#define SIZEOF_LONG_LONG 8
#define HAVE_GCC_DESTRUCTOR
#define HAVE_GCC_FORMAT
#define HAVE_STDINT_H
#define HAVE_DLFCN_H
#define HAVE_MALLOC_H
#define HAVE_CPUID_H
#define HAVE_FLOAT_H
#define HAVE_FENV_H
#define HAVE_FESETROUND
#define HAVE_PTHREAD_SETSCHEDPARAM

#if defined(__i386__) | defined(__amd64__)
#define HAVE_SSE
#define HAVE_XMMINTRIN_H
#endif

#endif


#ifdef MACOSX

#define ALIGN(x) __attribute__((aligned(x)))

#define RESTRICT __restrict

#define HAVE_POSIX_MEMALIGN
#define HAVE_SSE
#define HAVE_COREAUDIO
#define HAVE_WAVE
#define HAVE_STAT
#define HAVE_LRINTF
#define HAVE_STRTOF
#define SIZEOF_LONG 8
#define SIZEOF_LONG_LONG 8
#define HAVE_GCC_DESTRUCTOR
#define HAVE_GCC_FORMAT
#define HAVE_STDINT_H
#define HAVE_DLFCN_H
#define HAVE_XMMINTRIN_H
#define HAVE_FLOAT_H
#define HAVE_FENV_H
#define HAVE_FESETROUND
#define HAVE_PTHREAD_SETSCHEDPARAM

#endif
