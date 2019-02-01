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
#include "tools/CToolCursor.h"
#include "graphics/CTriangleArray.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    Constructor of cToolCursor.

    \param  a_parentWorld  World in which the tool will operate.
*/
//==============================================================================
cToolCursor::cToolCursor(cWorld* a_parentWorld):cGenericTool(a_parentWorld)
{
    // create a single point contact
    m_hapticPoint = new cHapticPoint(this);

    // add point to list
    m_hapticPoints.push_back(m_hapticPoint);

    // show proxy spheres only
    setShowContactPoints(true, false);
}


//==============================================================================
/*!
    Destructor of cToolCursor.
*/
//==============================================================================
cToolCursor::~cToolCursor()
{
    delete m_hapticPoint;
}


//==============================================================================
/*!
    This method updates the position and orientation of the tool image.
*/
//==============================================================================
void cToolCursor::updateToolImagePosition()
{
    // set the position and orientation of the tool image to be equal to the 
    // one of the haptic point proxy.
    cVector3d pos = m_hapticPoint->getLocalPosProxy();
    m_image->setLocalPos(pos);
    m_image->setLocalRot(m_deviceLocalRot);
}


//==============================================================================
/*!
    This method computes the interaction forces between the haptic point and 
    the virtual environment.
*/
//==============================================================================
void cToolCursor::computeInteractionForces()
{
    // compute interaction forces at haptic point in global coordinates
    cVector3d globalForce = m_hapticPoint->computeInteractionForces(m_deviceGlobalPos,
                                                                    m_deviceGlobalRot,
                                                                    m_deviceGlobalLinVel,
                                                                    m_deviceGlobalAngVel);
    cVector3d globalTorque(0.0, 0.0, 0.0);

    // update computed forces to tool
    setDeviceGlobalForce(globalForce);
    setDeviceGlobalTorque(globalTorque);
    setGripperForce(0.0);
}


//==============================================================================
/*!
    This method renders the current tool using OpenGL.

    \param  a_options  Rendering options.
*/
//==============================================================================
void cToolCursor::render(cRenderOptions& a_options)
{
    ///////////////////////////////////////////////////////////////////////
    // render haptic points
    ///////////////////////////////////////////////////////////////////////
    int numContactPoint = (int)(m_hapticPoints.size());
    for (int i=0; i<numContactPoint; i++)
    {
        // get next haptic point
        cHapticPoint* nextContactPoint = m_hapticPoints[i];

        // render tool
        nextContactPoint->render(a_options);
    }

    ///////////////////////////////////////////////////////////////////////
    // render mesh image
    ///////////////////////////////////////////////////////////////////////
    if (m_image != NULL)
    {
        m_image->renderSceneGraph(a_options);
    }
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
