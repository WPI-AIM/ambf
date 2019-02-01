//===========================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2003-2016, CHAI3D
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
    \version   3.2.0 $Rev: 1869 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CGenericDemoH
#define CGenericDemoH
//---------------------------------------------------------------------------
#include "chai3d.h"
#include "CBullet.h"
using namespace chai3d;
//---------------------------------------------------------------------------

class cGenericDemo
{   
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

public:

    //! Constructor of cGenericDemo.
    cGenericDemo(const std::string a_resourceRoot,
                 const int a_numDevices, 
                 std::shared_ptr<cGenericHapticDevice> a_hapticDevice0,
                 std::shared_ptr<cGenericHapticDevice> a_hapticDevice);

    //! Destructor of cGenericDemo.
    virtual ~cGenericDemo() {};


    //-----------------------------------------------------------------------
    // PUBLIC METHODS:
    //-----------------------------------------------------------------------

public:

    //! Initialize to demo
    virtual void init() {};

    //! Update haptics
    virtual void updateHaptics();

    //! Update graphics
    virtual void updateGraphics(int a_width, int a_height);

    //! Set stiffness
    virtual void setStiffness(double a_stiffness){};

    //! Set offset
    virtual void setOffset(double a_offset);

    //! Set torque gain
    virtual void setTorqueGain(double a_torqueGain);


    //-----------------------------------------------------------------------
    // PUBLIC MEMBERS:
    //-----------------------------------------------------------------------

public:

    //! bullet world
    cBulletWorld* m_bulletWorld;

    //! camera
    cCamera* m_camera;

    //! light source 0
    cSpotLight *m_light0;

    //! light source 1
    cDirectionalLight *m_light1;

    //! radius of tools
    double m_toolRadius;

    //! tool 0
    cToolGripper* m_tool0;

    //! tool 1
    cToolGripper* m_tool1;

    //! table 
    cMesh* m_ground;

    //! base
    cMultiMesh* m_base;

    //! simulation clock
    cPrecisionClock simClock;

    //! number of tools
    int m_numTools;

    //! table of tools
    cToolGripper* m_tools[2];

    //! bullet planes
    cBulletStaticPlane* m_bulletInvisibleWall0;
    cBulletStaticPlane* m_bulletInvisibleWall1;
    cBulletStaticPlane* m_bulletInvisibleWall2;
    cBulletStaticPlane* m_bulletInvisibleWall3;
    cBulletStaticPlane* m_bulletInvisibleWall4;
    cBulletStaticPlane* m_bulletInvisibleWall5;

    //! torque gain
    double m_torqueGain;

    //! mirroed display
    bool m_mirroredDisplay;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
