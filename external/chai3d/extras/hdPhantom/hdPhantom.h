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

    \author     Federico Barbagli
    \author     Francois Conti
    \author     Sebastien Grange
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef hdPhantomH
#define hdPhantomH
//---------------------------------------------------------------------------
#if defined(WIN32) | defined(WIN64)
#define __FNCALL __stdcall
#endif
#ifdef LINUX
#define __FNCALL
#endif
//---------------------------------------------------------------------------
#ifdef __cplusplus
extern "C" {
#endif
//---------------------------------------------------------------------------

//===========================================================================
// API
//===========================================================================

int __FNCALL hdPhantomGetNumDevices();

int __FNCALL hdPhantomOpen(int a_deviceID);

int __FNCALL hdPhantomClose(int a_deviceID);

int __FNCALL hdPhantomGetPosition(int a_deviceID,
                                  double *a_posX,
                                  double *a_posY,
                                  double *a_posZ);

int __FNCALL hdPhantomGetLinearVelocity(int a_deviceID,
                                        double *a_velX,
                                        double *a_velY,
                                        double *a_velZ);

int __FNCALL hdPhantomGetRotation(int a_deviceID,
                                  double *a_rot00,
                                  double *a_rot01,
                                  double *a_rot02,
                                  double *a_rot10,
                                  double *a_rot11,
                                  double *a_rot12,
                                  double *a_rot20,
                                  double *a_rot21,
                                  double *a_rot22);

int __FNCALL hdPhantomGetButtons(int a_deviceID);

int __FNCALL hdPhantomSetForce(int a_deviceID,
                               double *a_forceX,
                               double *a_forceY,
                               double *a_forceZ);

int __FNCALL hdPhantomSetTorque(int a_deviceID,
                                double *a_torqueX,
                                double *a_torqueY,
                                double *a_torqueZ);

int __FNCALL hdPhantomSetForceAndTorque(int a_deviceID,
                                        double *a_forceX,
                                        double *a_forceY,
                                        double *a_forceZ,
                                        double *a_torqueX,
                                        double *a_torqueY,
                                        double *a_torqueZ);

int __FNCALL hdPhantomGetWorkspaceRadius(int a_deviceID,
                                         double *a_workspaceRadius);

int __FNCALL hdPhantomGetType(int a_deviceID,
                              char* a_typeName);

void __FNCALL hdPhantomStartServo(void);

void __FNCALL hdPhantomStopServo(void);

//---------------------------------------------------------------------------
#ifdef __cplusplus
}
#endif
//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
