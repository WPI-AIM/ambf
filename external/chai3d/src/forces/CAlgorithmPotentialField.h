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
#ifndef CAlgorithmPotentialFieldH
#define CAlgorithmPotentialFieldH
//------------------------------------------------------------------------------
#include "forces/CGenericForceAlgorithm.h"
#include "forces/CInteractionBasics.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CAlgorithmPotentialField.h

    \brief 
    Implements a force algorithm for modeling haptic effects.
*/
//==============================================================================

//==============================================================================
/*!
    \class      cAlgorithmPotentialField
    \ingroup    forces

    \brief
    This class implements a force algorithms for handling all haptic effects
    associated with objects.

    \details
    This class implements a force algorithms for handling all haptic effects
    (\ref cGenericEffect)associated with objects.
*/
//==============================================================================
class cAlgorithmPotentialField : public cGenericForceAlgorithm
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cAlgorithmPotentialField.
    cAlgorithmPotentialField();

    //! Destructor of cAlgorithmPotentialField.
    virtual ~cAlgorithmPotentialField() {};


    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

public:

    //! This method initializes the algorithm by passing the start position of the haptic device.
    void initialize(cWorld* a_world, const cVector3d& a_initialPos) { m_world = a_world; };

    //! This method computes the next force given the updated position of the haptic device.
    virtual cVector3d computeForces(const cVector3d& a_toolPos, const cVector3d& a_toolVel);


    //--------------------------------------------------------------------------
    // PUBLIC MEMBERS:
    //--------------------------------------------------------------------------

public:

    //! Interaction recorder.
    cInteractionRecorder m_interactionRecorder;


    //--------------------------------------------------------------------------
    // PROTECTED METHODS:
    //--------------------------------------------------------------------------

protected:

    //! This method renders the force algorithms graphically in OpenGL. (For debug purposes)
    virtual void render(cRenderOptions& a_options) {}


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS:
    //--------------------------------------------------------------------------

protected:

    //! Identification number for this force algorithm object.
    unsigned int m_IDN;

    //! Static IDN counter for all algorithms of this class.
    static unsigned int m_IDNcounter;
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
