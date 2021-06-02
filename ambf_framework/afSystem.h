#ifndef AF_SYSTEM_H
#define AF_SYSTEM_H

// Plugins are always visible
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define AF_EXPORT __attribute__ ((dllexport))
  #else
    #define AF_EXPORT __declspec(dllexport)
  #endif
#else
    #define AF_EXPORT
#endif


// AF_SYSTEM
#endif

