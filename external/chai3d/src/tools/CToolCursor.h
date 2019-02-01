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
#ifndef CToolCursorH
#define CToolCursorH
//------------------------------------------------------------------------------
#include "tools/CGenericTool.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CToolCursor.h
    
    \brief  
    Implements a tool cursor using a single haptic point.
*/
//==============================================================================

//==============================================================================
/*!
    \class      cToolCursor
    \ingroup    tools

    \brief
    This class implements a cursor using a single haptic point.

    \details
    cToolCursor implements a single point contact haptic tool that can 
    apply forces in  three degrees of freedom and maintains three or 
    six degrees of device pose. \n
*/
//==============================================================================
class cToolCursor : public cGenericTool
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cToolCursor.
    cToolCursor(cWorld* a_parentWorld);

    //! Destructor of cToolCursor.
    virtual ~cToolCursor();


    //--------------------------------------------------------------------------
    // PUBLIC MEMBERS
    //--------------------------------------------------------------------------

public:

    // Single haptic point representing a cursor.
    cHapticPoint* m_hapticPoint;


    //--------------------------------------------------------------------------
    // PUBLIC METHODS
    //--------------------------------------------------------------------------

public:

    //! This method computes the interaction forces between the haptic point and the environment.
    virtual void computeInteractionForces();

    //! This method renders the tools using OpenGL.
    virtual void render(cRenderOptions& a_options);

    //! This method updates the position and orientation of the tool image.
    virtual void updateToolImagePosition();
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------

