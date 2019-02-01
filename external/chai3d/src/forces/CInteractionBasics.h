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
#ifndef CInteractionBasicsH
#define CInteractionBasicsH
//------------------------------------------------------------------------------
#include "math/CVector3d.h"
#include "math/CMatrix3d.h"
#include "graphics/CVertexArray.h"
#include "materials/CMaterial.h"
//------------------------------------------------------------------------------
#include <vector>
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
class cGenericObject;
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CInteractionBasics.h

    \brief 
    Implements basic structures for handling interaction events.
*/
//==============================================================================

//==============================================================================
/*!  
    \struct     cInteractionEvent
    \ingroup    forces

    \brief
    This structure stores information associated with the interaction of a 
    haptic tool with a shape primitive.

    \details
    This structure stores information associated with the interaction of a 
    haptic tool with a shape primitive.
*/
//==============================================================================
struct cInteractionEvent
{
    //! Pointer to the object.
    cGenericObject* m_object;

    //! If __true__ then the interaction event occurred inside the object.
    bool m_isInside;

    //! Position of the interaction point in reference to the object's coordinate frame (local coordinates).
    cVector3d m_localPos;

    //! Surface normal at the interaction point in reference to the object's coordinate frame (local coordinates).
    cVector3d m_localNormal;

    //! Interaction force in object's local coordinate frame.
    cVector3d m_localForce;

    //! Nearest point to the object's surface in local coordinates
    cVector3d m_localSurfacePos;

    //! This method initialize all data contained in current event.
    void clear()
    {
        m_object   = NULL;
        m_isInside = false;
        m_localPos.zero();
        m_localNormal.set(1,0,0);
        m_localForce.zero();
        m_localSurfacePos.zero();
    }
};


//==============================================================================
/*!
    \class      cInteractionRecorder
    \ingroup    forces

    \brief
    This class stores a list of interaction events.

    \details
    cInteractionRecorder stores a list of interaction events that occur between
    a haptic tool and haptic effects programmed on objects.
*/
//==============================================================================
class cInteractionRecorder
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cInteractionRecorder.
    cInteractionRecorder() { clear(); }

    //! Destructor of cInteractionRecorder.
    virtual ~cInteractionRecorder() {};


    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

public:

    //! This method clears all interaction events.
    void clear()
    {
        m_interactions.clear();
    }


    //--------------------------------------------------------------------------
    // PUBLIC MEMBERS:
    //--------------------------------------------------------------------------

public:

    //! List of interaction events stored in recorder.
    std::vector<cInteractionEvent> m_interactions;
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
