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
#ifndef CGenericEffectH
#define CGenericEffectH
//------------------------------------------------------------------------------
#include "math/CVector3d.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
class cGenericObject;
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file CGenericEffect.h

    \brief
    Implements a base class for haptic effects
*/
//==============================================================================

//------------------------------------------------------------------------------
/*!
    Maximum number of force models supported the CHAI3D simulation. Some
    force effects require a history of the tool, such as its  previous position
    for instance. Effects which need such information store it in a local table
    and retrieve the information when they calculate the reaction force.
    See cEffectStickSlip class as an example
*/
//------------------------------------------------------------------------------
const int C_EFFECT_MAX_IDN = 16;


//==============================================================================
/*!
    \class      cGenericEffect
    \ingroup    effects

    \brief
    This class implements a base class for haptic effects.

    \details
    Haptic interaction forces between a virtual tool and this object 
    are defined by the programmed haptic effects (cGenericEffect) associated 
    with it. Haptic effects may include viscosity (cEffectViscosity), 
    vibrations (cEffectVibration), or stick-and-slip (cEffectStickSlip)
    for instance. \n\n

    At every haptic iteration and for each given tool (cGenericTool), the 
    computeInteraction() is called throughtout the entire scenegraph. For 
    each object in the world, the position of the tool is compared to 
    the position and topology of the object; this
    task is handled by the virtual method computeLocalInteraction()
    which decides if the tool is locate inside or outside the object.
    The result is stored in the boolean variable m_interactionInside by
    setting it to true if the tool is located inside the object or 
    false otherwise. The method also computes the nearest point towards 
    the surface of the object and stores the result in variable 
    m_interactionProjectedPoint. \n\n
    
    The computeInteraction() then calls method computeForce() for each
    haptic effect programmed for this object. Haptic effects are stored
    in the list m_effects. If the tool is located inside the object 
    (m_interactionInside == true), then interaction forces are computed
    and added to the virtual tool. \n\n

    Finally, after traversing each object in the scenegraph, the resulting 
    force (sum of all interaction forces) is sent to the haptic device.
*/
//==============================================================================
class cGenericEffect
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of CGenericEffect.
    cGenericEffect(cGenericObject* a_parent); 

    //! Destructor of CGenericEffect.
    virtual ~cGenericEffect() {};


    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

public:

    //! This method computes the resulting force.
    virtual bool computeForce(const cVector3d& a_toolPos,
                              const cVector3d& a_toolVel,
                              const unsigned int& a_toolID,
                              cVector3d& a_reactionForce)
                              {
                                  a_reactionForce.zero();
                                  return (false);
                              }

    //! This method enables or disables this effect.
    inline void setEnabled(bool a_enabled) { m_enabled = a_enabled; }

    //! This method returns __true__ if this effect is enables, __false__ otherwise.
    inline bool getEnabled() const { return (m_enabled); }


    //--------------------------------------------------------------------------
    // PUBLIC MEMBERS:
    //--------------------------------------------------------------------------

public:

    //! Object to which the force effects applies.
    cGenericObject* m_parent;


    //--------------------------------------------------------------------------
    // PROTECTED METHODS:
    //--------------------------------------------------------------------------

protected:

    //! This method initializes the haptic effect model.
    virtual void initialize() { return; }


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS:
    //--------------------------------------------------------------------------

protected:

    //! Is this effect currently enabled?
    bool m_enabled;
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
