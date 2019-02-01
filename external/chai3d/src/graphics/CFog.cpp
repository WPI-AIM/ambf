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
#include "graphics/CFog.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    Constructor of cFog.
*/
//==============================================================================
cFog::cFog()
{
    // enable fog
    m_enabled = false;

    // set default fog color
    m_color.set(1.0, 1.0, 1.0, 1.0);

    // set default properties
    setProperties(0.1, 10.0, 0.2);

    // set default fog mode
    setFogModeLINEAR();
}


//==============================================================================
/*!
    This method sets the fog properties.

    \param  a_start    Distance from viewport.
    \param  a_end      Distance from viewport.
    \param  a_density  Fog density
*/
//==============================================================================
void cFog::setProperties(const double a_start, const double a_end, const double a_density)
{
    m_start   = (float)a_start;
    m_end     = (float)cMax(a_start, a_end);
    m_density = (float)a_density;
}


//==============================================================================
/*!
    This method render fog using OpenGL.

    \param  a_options  Rendering options
*/
//==============================================================================
void cFog::render(cRenderOptions& a_options)
{
#ifdef C_USE_OPENGL

    if ((!m_enabled) || (a_options.m_creating_shadow_map))
    {
        // fog is disabled
        glDisable(GL_FOG);
        return;
    }

    // enable fog and define attributes
    glEnable(GL_FOG);
    glFogi(GL_FOG_MODE, m_fogMode);
    glFogfv(GL_FOG_COLOR, m_color.getData());
    glFogf(GL_FOG_DENSITY, m_density);
    glHint(GL_FOG_HINT, GL_NICEST);
    glFogf(GL_FOG_START, m_start);
    glFogf(GL_FOG_END, m_end);

#endif
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
