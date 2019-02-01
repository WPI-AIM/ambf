//===========================================================================
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

    \author     Sebastien Grange
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef tdLeapH
#define tdLeapH
//---------------------------------------------------------------------------
#if defined(WIN32) | defined(WIN64)
#define __FNCALL __cdecl
#else
#define __FNCALL
#endif
//---------------------------------------------------------------------------
#include "Leap.h"
#include "chai3d.h"
//---------------------------------------------------------------------------
#ifdef __cplusplus
extern "C" {
#endif
//---------------------------------------------------------------------------

//===========================================================================
// API
//===========================================================================

// device management
int __FNCALL tdLeapGetNumDevices();
int __FNCALL tdLeapOpen();
int __FNCALL tdLeapClose();

// data access
bool __FNCALL tdLeapUpdate();
bool __FNCALL tdLeapGetPosition(chai3d::cVector3d a_position[2]);
bool __FNCALL tdLeapGetRotation(chai3d::cMatrix3d a_rotation[2]);
bool __FNCALL tdLeapGetGripperAngleRad(double a_angle[2]);
bool __FNCALL tdLeapGetUserSwitches(unsigned int a_userSwitches[2]);

// raw pointer access
bool __FNCALL tdLeapGetFrame(void* &a_frame);

//---------------------------------------------------------------------------
#ifdef __cplusplus
}
#endif
//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
