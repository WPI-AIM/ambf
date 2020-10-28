//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2003-2016, CHAI3D.
    (www.chai3d.org)

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

    * Neither the name of CHAI3D nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE. 

    \author    <http://www.chai3d.org>
    \author    Francois Conti
    \version   3.2.0 $Rev: 2159 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CGlobalsH
#define CGlobalsH
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CGlobals.h
    \ingroup    system

    \brief  
    Implements option settings for CHAI3D.

    \details

    __CGlobal.h__ also contains a list of compiler settings that control the 
    features included in CHAI3D. Default settings are provided for
    __Windows__, __Linux__ and __Mac OS-X__ operating systems. \n\n

    CHAI3D compiling options are the following:\n

    - __C_USE_OPEN_GL__: Enable or disable support for OpenGL. Disabling OpenGL
    allows you to compile CHAI3D on embedded real-time operating systems for 
    instance, or with applications that may use a different graphics library 
    (i.e. DirectX). 
    
    - __C_USE_EIGEN__: Enable or disable support of external Eigen libraries.
                     If Eigen is disabled, CHAI3D uses its own internal 
                     matrix library and representation.

    - __C_USE_FILE_3DS__: Enable of disable external support for 3DS files.\n
    - __C_USE_FILE_GIF__: Enable of disable external support for GIF files.\n
    - __C_USE_FILE_JPG__: Enable of disable external support for JPG files.\n
    - __C_USE_FILE_PNG__: Enable of disable external support for PNG files.\n
                        
    Disabling one or more features will reduce the overall capabilities of 
    CHAI3D and may affect some of the examples provided with the framework.
    
    For more information about the different compilation settings, please
    review the inline comments in the __CGlobal.h__ header file.
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef DOXYGEN_SHOULD_SKIP_THIS
//------------------------------------------------------------------------------

//==============================================================================
// GENERAL SETTINGS
//==============================================================================

//------------------------------------------------------------------------------
// STANDARD LIBRARIES
//------------------------------------------------------------------------------

#include <cstdlib>
#include <cstring>
#include <memory>


//------------------------------------------------------------------------------
// COMPILING OPTIONS
//------------------------------------------------------------------------------

// OPENGL SUPPORT
// Enable or disable OpenGL and GLEW libraries.
#define C_USE_OPENGL

// EIGEN SUPPORT
// Enable or disable external Eigen math library.
#define C_USE_EIGEN

// 3DS FILE SUPPORT
// Enable or disable external support for 3DS files.
#define C_USE_FILE_3DS

// GIF FILE SUPPORT
// Enable or disable external support for GIF files.
#define C_USE_FILE_GIF

// JPG FILE SUPPORT
// Enable or disable external support for JPG files.
#define C_USE_FILE_JPG

// PNG FILE SUPPORT
// Enable of disable external support for PNG files.
#define C_USE_FILE_PNG 


//==============================================================================
// OPERATING SYSTEM SPECIFIC
//==============================================================================

//------------------------------------------------------------------------------
// WIN32 / WIN64 OS
//------------------------------------------------------------------------------
#if defined(WIN32) | defined(WIN64)

    //--------------------------------------------------------------------
    // GENERAL
    //--------------------------------------------------------------------

    // OS specific
    #if !defined(NOMINMAX)
        #define NOMINMAX
    #endif
    #include <windows.h>

    // if building with .NET, we need to explicitly tell MSVC that we are using unmanaged code
    // http://msdn.microsoft.com/en-us/library/4ex65770(v=vs.110).aspx
    #if (_MANAGED == 1)
      #define nullptr __nullptr
    #endif

    // printf
    #include <conio.h>
    #define cPrint _cprintf

    // math
    #if !defined(C_USE_EIGEN)
        #define _USE_MATH_DEFINES
        #include <cmath>
    #endif

    // OpenGL GLEW library
    #if defined(C_USE_OPENGL)
        #ifndef GLEW_STATIC
        #define GLEW_STATIC
        #endif
    #endif

    //--------------------------------------------------------------------
    // HAPTIC DEVICES
    //--------------------------------------------------------------------
    #define C_ENABLE_CUSTOM_DEVICE_SUPPORT
    #if defined (__arm__)
        #define C_ENABLE_DELTA_DEVICE_SUPPORT
    #endif
    #define C_ENABLE_PHANTOM_DEVICE_SUPPORT
    #define C_ENABLE_LEAP_DEVICE_SUPPORT
    // #define C_ENABLE_SIXENSE_DEVICE_SUPPORT

    //--------------------------------------------------------------------
    // SYSTEM LIBRARIES
    //--------------------------------------------------------------------
    #pragma comment (lib, "winmm.lib")

#endif


//------------------------------------------------------------------------------
// LINUX OS
//------------------------------------------------------------------------------
#if defined(LINUX)

    //--------------------------------------------------------------------
    // GENERAL
    //--------------------------------------------------------------------

    // OS specific
    #include <ctime>
    #include <pthread.h>
    #include <dlfcn.h>

    // printf
    #define cPrint printf

    //--------------------------------------------------------------------
    // HAPTIC DEVICES
    //--------------------------------------------------------------------
    #if !defined (__arm__)
        #define C_ENABLE_DELTA_DEVICE_SUPPORT
    #endif
    #define C_ENABLE_PHANTOM_DEVICE_SUPPORT
    #define C_ENABLE_LEAP_DEVICE_SUPPORT
//    #define C_ENABLE_AMBF_DVRK_DEVICE_SUPPORT
//    #define C_ENABLE_SIXENSE_DEVICE_SUPPORT

#endif


//------------------------------------------------------------------------------
// MAC OS
//------------------------------------------------------------------------------
#if defined(MACOSX)

    //--------------------------------------------------------------------
    // GENERAL
    //--------------------------------------------------------------------

    // OS specific
    #include <mach/mach_time.h>
    #include <pthread.h>
    #include <dlfcn.h>

    // printf
    #define cPrint printf

    //--------------------------------------------------------------------
    // HAPTIC DEVICES
    //--------------------------------------------------------------------
    #if defined (__arm__)
        #define C_ENABLE_DELTA_DEVICE_SUPPORT
    #endif
    #define C_ENABLE_LEAP_DEVICE_SUPPORT
    // #define C_ENABLE_SIXENSE_DEVICE_SUPPORT

#endif


//------------------------------------------------------------------------------
#endif  // DOXYGEN_SHOULD_SKIP_THIS
//------------------------------------------------------------------------------


//==============================================================================
// GENERAL PURPOSE FUNCTIONS:
//==============================================================================

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//! This function suspends the execution of the current thread for a specified interval.
void cSleepMs(const unsigned int a_interval);


//------------------------------------------------------------------------------
#ifndef DOXYGEN_SHOULD_SKIP_THIS
//------------------------------------------------------------------------------

//==============================================================================
// CHAI3D - OPENGL:
//==============================================================================

//------------------------------------------------------------------------------
// The following definitions allow CHAI3D to compile without the external OpenGL 
// libraries. This mode of compilation is controlled by the flag C_USE_OPENGL.
// Compiling the framework without OpenGL can be very useful if you plan to 
// use CHAI3D for haptic rendering purposes only, or on real-time operating 
// systems such as QNX or VxWorks for instance where OpenGL is not supported. 
//------------------------------------------------------------------------------

#ifndef C_USE_OPENGL

typedef unsigned int                GLenum;
typedef unsigned int                GLuint;
typedef int                         GLint;
typedef int                         GLsizei;
typedef unsigned char               GLboolean;
typedef signed char                 GLbyte;
typedef short                       GLshort;
typedef unsigned char               GLubyte;
typedef unsigned short              GLushort;
typedef unsigned long               GLulong;
typedef float                       GLfloat;
typedef float                       GLclampf;
typedef double                      GLdouble;
typedef double                      GLclampd;

#define GL_FALSE                    0
#define GL_TRUE                     1
#define GL_EXP                      0x0800
#define GL_EXP2                     0x0801
#define GL_UNSIGNED_BYTE            0x1401
#define GL_LINE                     0x1B01
#define GL_POINTS                   0x0000
#define GL_BYTE                     0x1400
#define GL_SHORT                    0x1402
#define GL_UNSIGNED_SHORT           0x1403
#define GL_UNSIGNED_INT             0x1405
#define GL_DEPTH_COMPONENT          0x1902
#define GL_RGB                      0x1907
#define GL_RGBA                     0x1908
#define GL_LUMINANCE                0x1909
#define GL_LUMINANCE_ALPHA          0x190A
#define GL_TEXTURE0                 0x84C0
#define GL_TEXTURE1                 0x84C1
#define GL_TEXTURE2                 0x84C2
#define GL_TEXTURE3                 0x84C3
#define GL_TEXTURE4                 0x84C4
#define GL_POINT                    0x1B00
#define GL_LINE                     0x1B01
#define GL_FILL                     0x1B02
#define GL_MODULATE                 0x2100
#define GL_DECAL                    0x2101
#define GL_LINEAR                   0x2601
#define GL_LINEAR_MIPMAP_LINEAR     0x2703
#define GL_CLAMP                    0x2900
#define GL_REPEAT                   0x2901
#define GL_LIGHT0                   0x4000
#define GL_VERTEX_ARRAY             0x8074
#define GLUquadricObj               void
    
#define GL_CLAMP_TO_EDGE            0x812F
#define GL_NEAREST                  0x2600
#define GL_NEAREST_MIPMAP_NEAREST   0x2700

#endif // C_USE_OPENGL

//------------------------------------------------------------------------------
#endif  // DOXYGEN_SHOULD_SKIP_THIS
//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
