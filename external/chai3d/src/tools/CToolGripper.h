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
    \author    Federico Barbagli
    \version   3.2.0 $Rev: 1869 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CToolGripperH
#define CToolGripperH
//------------------------------------------------------------------------------
#include "tools/CGenericTool.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CToolGripper.h

    \brief
    Implements a gripper using two haptic points.
*/
//==============================================================================

//==============================================================================
/*!
    \class      cToolGripper
    \ingroup    tools
    
    \brief
    This class implements a gripper using a two haptic points.

    \details
    cToolGripper implements a gripper tool using two haptic points.
    The distance between both haptic points is controlled by the haptic gripper.
    The resulting forces computed at each haptic point are converted into
    a force, torque, and gripper force, which are then sent to the haptic device.
*/
//==============================================================================
class cToolGripper : public cGenericTool
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cToolGripper.
    cToolGripper(cWorld* a_parentWorld);

    //! Destructor of cToolGripper.
    virtual ~cToolGripper();


    //--------------------------------------------------------------------------
    // PUBLIC MEMBERS
    //--------------------------------------------------------------------------

public:

    // Haptic point modeling the thumb.
    cHapticPoint* m_hapticPointThumb;

    // Haptic point modeling the index.
    cHapticPoint* m_hapticPointFinger;


    //--------------------------------------------------------------------------
    // PUBLIC METHODS
    //--------------------------------------------------------------------------

public:

    //! This method computes the interaction forces between the gripper tool and environment.
    virtual void computeInteractionForces();

    //! This method renders the tools using OpenGL.
    virtual void render(cRenderOptions& a_options);

    //! This method sets a workspace scale factor for the gripper.
    virtual void setGripperWorkspaceScale(double a_gripperWorkspaceScale) { m_gripperWorkspaceScale = fabs(a_gripperWorkspaceScale); }

    //! This method returns the workspace scale factor of the gripper.
    virtual double getGripperWorskpaceScale() { return (m_gripperWorkspaceScale); }


    //--------------------------------------------------------------------------
    // MEMBERS
    //--------------------------------------------------------------------------

protected:

    // Workspace scale factor of force gripper.
    double m_gripperWorkspaceScale;
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------

