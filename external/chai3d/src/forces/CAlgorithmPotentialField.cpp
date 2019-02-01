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
#include "forces/CAlgorithmPotentialField.h"
//------------------------------------------------------------------------------
#include "forces/CInteractionBasics.h"
#include "world/CWorld.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
unsigned int cAlgorithmPotentialField::m_IDNcounter = 0;
//------------------------------------------------------------------------------

//==============================================================================
/*!
    Constructor of cAlgorithmPotentialField.
*/
//==============================================================================
cAlgorithmPotentialField::cAlgorithmPotentialField()
{
    // define an identification number for this force algorithm
    m_IDN = m_IDNcounter;

    // increment counter
    m_IDNcounter++;
}


//==============================================================================
/*!
    This method computes forces for all haptic effects associated with the
    objects includes in the world and the haptic tool handled by this algorithms.

    \param  a_toolPos  Position of tool.
    \param  a_toolVel  Velocity of tool.
*/
//==============================================================================
cVector3d cAlgorithmPotentialField::computeForces(const cVector3d& a_toolPos,
                                                  const cVector3d& a_toolVel)
{
    // initialize force
    cVector3d force;
    force.zero();
    m_interactionRecorder.m_interactions.clear();

    // compute forces for all haptic effects associated with the objects located
    // in the world
    if (m_world != NULL)
    {
        force = m_world->computeInteractions(a_toolPos,
                                             a_toolVel,
                                             m_IDN,
                                             m_interactionRecorder);
    }

    // return resulting force
    return (force);
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
