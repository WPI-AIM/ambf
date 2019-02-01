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
#include "lighting/CPositionalLight.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    Constructor of cPositionalLight.

    \param  a_world  Parent world in which the light source is created.
*/
//==============================================================================
cPositionalLight::cPositionalLight(cWorld* a_world):cGenericLight(a_world)
{   
    // constant attenuation term
    m_attConstant = 1.0;

    // linear attenuation term
    m_attLinear = 0.0;

    // quadratic attenuation term
    m_attQuadratic  = 0.0;

    // set color properties of display model of light source
    m_displaySourceColor.setYellowGold();
}


//==============================================================================
/*!
    Destructor of cPositionalLight.
*/
//==============================================================================
cPositionalLight::~cPositionalLight()
{
}


//==============================================================================
/*!
    This method sets the display settings of light source. 
    To be used for debugging purposes to display location and direction of light
    source.

    \param  a_sourceRadius    Radius of displayed light source.
    \param  a_displayEnabled  Display status.
*/
//==============================================================================
void cPositionalLight::setDisplaySettings(const double& a_sourceRadius, 
                                          const bool a_displayEnabled)
{
    m_displaySourceRadius = a_sourceRadius;
    m_displayEnabled = a_displayEnabled;
}


//==============================================================================
/*!
    This method renders a graphic representation of the positional light 
    using OpenGL.

    \param  a_options  Rendering options.
*/
//==============================================================================
void cPositionalLight::render(cRenderOptions& a_options)
{
#ifdef C_USE_OPENGL

    // is display rendering of light source enabled?
    if (!m_displayEnabled) { return; }

    /////////////////////////////////////////////////////////////////////////
    // Render parts that are always opaque
    /////////////////////////////////////////////////////////////////////////
    if (SECTION_RENDER_OPAQUE_PARTS_ONLY(a_options) && (!a_options.m_creating_shadow_map))
    {
        // disable lighting
        glDisable(GL_LIGHTING);

        // render light source
        m_displaySourceColor.render();
        cDrawSphere(m_displaySourceRadius, 8, 8);

        // enable lighting again
        glEnable(GL_LIGHTING);
    }

#endif
}


//==============================================================================
/*!
    This method renders this light source using OpenGL.

    \param  a_options  Rendering options.
*/
//==============================================================================
void cPositionalLight::renderLightSource(cRenderOptions& a_options)
{
#ifdef C_USE_OPENGL

    // check if light sources should be rendered
    if ((!a_options.m_enable_lighting) || (!m_enabled))
    {
        // disable OpenGL light source
        glDisable(m_glLightNumber);
        return;
    }

    // enable this light in OpenGL
    glEnable(m_glLightNumber);

    // enable or disable two side light model
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, m_useTwoSideLightModel);

    // compute global position of current light in world
    computeGlobalPositionsFromRoot();

    // set lighting components
    if (a_options.m_rendering_shadow)
    {
        cColorf diffuse((GLfloat)(a_options.m_shadow_light_level * m_diffuse.getR()),
                                (GLfloat)(a_options.m_shadow_light_level * m_diffuse.getG()),
                                (GLfloat)(a_options.m_shadow_light_level * m_diffuse.getB()));
        cColorf specular(0.0, 0.0, 0.0);
        glLightfv(m_glLightNumber, GL_AMBIENT,  m_ambient.getData());
        glLightfv(m_glLightNumber, GL_DIFFUSE,  diffuse.getData() );
        glLightfv(m_glLightNumber, GL_SPECULAR, specular.getData());
    }
    else
    {
        glLightfv(m_glLightNumber, GL_AMBIENT,  m_ambient.getData());
        glLightfv(m_glLightNumber, GL_DIFFUSE,  m_diffuse.getData() );
        glLightfv(m_glLightNumber, GL_SPECULAR, m_specular.getData());
    }

    // disable cutoff angle
    glLightf(m_glLightNumber, GL_SPOT_CUTOFF, 180); 

    // define the position of the light source. 
    float position[4];
    position[0] = (float)m_globalPos(0) ;
    position[1] = (float)m_globalPos(1) ;
    position[2] = (float)m_globalPos(2) ;
    position[3] = 1.0f;     // defines light to be of type positional
    glLightfv(m_glLightNumber, GL_POSITION, (const float *)&position);

#endif
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

