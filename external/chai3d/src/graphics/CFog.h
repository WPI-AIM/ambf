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
#ifndef CFogH
#define CFogH
//------------------------------------------------------------------------------
#include "graphics/CColor.h"
#include "graphics/CRenderOptions.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CFog.h
    
    \brief
    Implements graphic fog capabilities.
*/
//==============================================================================

//==============================================================================
/*!
    \class      cFog
    \ingroup    graphics

    \brief
    This class implements fog inside the world.

    \details
    This class implements support for OpenGL's fog capability.
    When fog is enabled, objects that are farther from the viewpoint begin to 
    fade into the fog color. You can control the density of the fog, which 
    determines the rate at which objects fade as the distance increases, as 
    well as the fog's color.
*/
//==============================================================================
class cFog
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cFog.
    cFog();

    //! Destructor of cFog.
    virtual ~cFog() {};


    //-----------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

public:

    //! This method renders the fog using OpenGL.
    virtual void render(cRenderOptions& a_options);

    //! This method enables of disables fog.
    void setEnabled(const bool a_enabled) { m_enabled = a_enabled; }

    //! This method returns __true__ if fog is enabled, __false__ otherwise.
    bool getEnabled() const { return(m_enabled); }

    //! This method sets the fog mode by passing the OpenGL fog mode constant as parameter (GL_LINEAR, GL_EXP, GL_EXP2).
    void setFogMode(const GLint a_fogMode) { m_fogMode = a_fogMode; }

    //! This method sets the fog mode to GL_LINEAR.
    void setFogModeLINEAR() { m_fogMode = GL_LINEAR; }

    //! This method sets the fog mode to GL_EXP.
    void setFogModeEXP() { m_fogMode = GL_EXP; }

    //! This method sets the fog mode to GL_EXP2.
    void setFogModeEXP2() { m_fogMode = GL_EXP2; }

    //! This method returns the current fog mode.
    GLint getFogMode() { return (m_fogMode); }

    //! This method sets the fog properties.
    void setProperties(const double a_start,
        const double a_end, 
        const double a_density);


    //-----------------------------------------------------------------------
    // PUBLIC MEMBERS:
    //--------------------------------------------------------------------------

public:

    //! Fog color.
    cColorf m_color;


    //-----------------------------------------------------------------------
    // PROTECTED MEMBERS:
    //--------------------------------------------------------------------------

protected:

    //! If __true__, then fog is enabled, __false__ otherwise.
    bool m_enabled;

    //! Fog mode. (GL_LINEAR, GL_EXP, GL_EXP2).
    GLint m_fogMode;

    //! Fog start distance.
    float m_start;

    //! Fog end distance.
    float m_end;

    //! Fog density.
    float m_density;
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------

