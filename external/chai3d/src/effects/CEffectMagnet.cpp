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
    \version   3.2.0 $Rev: 1869 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "effects/CEffectMagnet.h"
//------------------------------------------------------------------------------
#include "world/CGenericObject.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    Constructor of cEffectMagnet.

    \param  a_parent  Parent object.
*/
//==============================================================================
cEffectMagnet::cEffectMagnet(cGenericObject* a_parent):cGenericEffect(a_parent)
{
    m_enabledInside = false;
}


//==============================================================================
/*!
    This method computes the resulting force effect.

    \param  a_toolPos        Position of tool.
    \param  a_toolVel        Velocity of tool.
    \param  a_toolID         Identification number of the force algorithm stored in the tool.
    \param  a_reactionForce  Return value for the computed force.

    \return __false__ if no interaction force occurs, __true__ otherwise.
*/
//==============================================================================
bool cEffectMagnet::computeForce(const cVector3d& a_toolPos,
                                  const cVector3d& a_toolVel,
                                  const unsigned int& a_toolID,
                                  cVector3d& a_reactionForce)
{
    // compute distance from object to tool
    double distance = cDistance(a_toolPos, m_parent->m_interactionPoint);

    // get parameters of magnet
    double magnetMaxForce = m_parent->m_material->getMagnetMaxForce();
    double magnetMaxDistance = m_parent->m_material->getMagnetMaxDistance();
    double stiffness = m_parent->m_material->getStiffness();
    double forceMagnitude = 0;

    if (m_enabledInside || (!m_parent->m_interactionInside))
    {
        if ((distance < magnetMaxDistance) && (stiffness > 0))
        {
            double d = magnetMaxForce / stiffness;
            if (distance < d)
            {
                forceMagnitude = stiffness * distance;
            }
            else
            {
                double dx = (magnetMaxDistance - d);
                if (dx > 0)
                {
                    double k = magnetMaxForce / dx;
                    forceMagnitude = k * (magnetMaxDistance - distance);
                }
            }

            // compute reaction force
            int sign = -1;
            if (m_parent->m_interactionInside)
            {
                sign = 1;
            }

            a_reactionForce = cMul(sign * forceMagnitude, m_parent->m_interactionNormal);

            return (true);
        }
        else
        {
            return (false);
        }
    }
    else
    {
        // the tool is located outside the magnet zone
        a_reactionForce.zero();
        return (false);
    }
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

