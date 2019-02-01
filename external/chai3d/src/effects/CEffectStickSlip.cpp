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
#include "effects/CEffectStickSlip.h"
//------------------------------------------------------------------------------
#include "math/CMaths.h"
#include "world/CGenericObject.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    Constructor of cEffectStickSlip.

    \param  a_parent Parent object.
*/
//==============================================================================
cEffectStickSlip::cEffectStickSlip(cGenericObject* a_parent):cGenericEffect(a_parent)
{
    for (int i=0; i<C_EFFECT_MAX_IDN; i++)
    {
        m_history[i].m_currentStickPosition.zero();
        m_history[i].m_valid = false;
    }
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
bool cEffectStickSlip::computeForce(const cVector3d& a_toolPos,
                                    const cVector3d& a_toolVel,
                                    const unsigned int& a_toolID,
                                    cVector3d& a_reactionForce)
{
    // check if history for this IDN exists
    if (a_toolID < (unsigned int)C_EFFECT_MAX_IDN)
    {
        if (m_parent->m_interactionInside)
        {
            // check if a recent valid point has been stored previously
            if (!m_history[a_toolID].m_valid)
            {
                m_history[a_toolID].m_currentStickPosition = a_toolPos;
                m_history[a_toolID].m_valid = true;
            }

            // read parameters for stick and slip model
            double stiffness = m_parent->m_material->getStickSlipStiffness();
            double forceMax = m_parent->m_material->getStickSlipForceMax();

            // compute current force between last stick position and current tool position
            double distance = cDistance(a_toolPos, m_history[a_toolID].m_currentStickPosition);
            double forceMag = distance * stiffness;

            if (forceMag > 0)
            {
                // if force above threshold, slip...
                if (forceMag > forceMax)
                {
                    m_history[a_toolID].m_currentStickPosition = a_toolPos;
                    a_reactionForce.zero();
                }
                // ...otherwise stick
                else
                {
                    a_reactionForce = (forceMag / distance) * cSub(m_history[a_toolID].m_currentStickPosition, a_toolPos);
                }
            }
            else
            {
                a_reactionForce.zero();
            }

            return (true);
        }
        else
        {
            // the tool is located outside the object, so zero reaction force
            m_history[a_toolID].m_valid = false;
            a_reactionForce.zero();
            return (false);
        }
    }
    else
    {
        a_reactionForce.zero();
        return (false);
    }
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

