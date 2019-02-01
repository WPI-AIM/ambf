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
#include "lighting/CDirectionalLight.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    Constructor of cDirectionalLight.

    \param  a_world  Parent world in which the light source is created.
*/
//==============================================================================
cDirectionalLight::cDirectionalLight(cWorld* a_world):cGenericLight(a_world)
{
    // set default direction
    setDir(0,0,-1);
}


//==============================================================================
/*!
    Destructor of cDirectionalLight.
*/
//==============================================================================
cDirectionalLight::~cDirectionalLight()
{
}


//==============================================================================
/*!
    This method sets the direction of the light beam.

    \param  a_direction  Direction of light beam.
*/
//==============================================================================
void cDirectionalLight::setDir(const cVector3d& a_direction)
{

    // We arbitrarily point lights along the x axis of the stored
    // rotation matrix.
    cVector3d c0, c1, c2, t0, t1;
    a_direction.copyto(c0);

    // check vector
    if (c0.lengthsq() < 0.0001) { return; }

    // normalize direction vector
    c0.normalize();

    // compute 2 vector perpendicular to a_direction
    t0.set(0.0, 0.0, 1.0);
    t1.set(0.0, 1.0, 0.0);
    double a0 = cAngle(c0, t0);
    double a1 = cAngle(c0, t1);

    if (sin(a0) > sin(a1))
    {
        c0.crossr(t0, c1);
        c0.crossr(c1, c2);
    }
    else
    {
        c0.crossr(t1, c1);
        c0.crossr(c1, c2);
    }

    c1.normalize();
    c2.normalize();

    // update rotation matrix
    m_localRot.setCol(c0,c1,c2);
}


//==============================================================================
/*!
    This method set the direction of the light beam.

    \param  a_x  X Coordinate of light beam.
    \param  a_y  Y Coordinate of light beam.
    \param  a_z  Z Coordinate of light beam.
*/
//==============================================================================
void cDirectionalLight::setDir(const double a_x, const double a_y, const double a_z)
{
    setDir(cVector3d(a_x, a_y, a_z));
}


//==============================================================================
/*!
    This method renders this light source in OpenGL.

    \param  a_options  Rendering options.
*/
//==============================================================================
void cDirectionalLight::renderLightSource(cRenderOptions& a_options)
{
#ifdef C_USE_OPENGL

    // check if light source should be rendered
    if ((!a_options.m_enable_lighting) || (!m_enabled))
    {
        // disable OpenGL light source
        glDisable(m_glLightNumber);
        return;
    }

    // compute global position of current light in world
    computeGlobalPositionsFromRoot();

    // enable this light in OpenGL
    glEnable(m_glLightNumber);

    // enable or disable two side light model
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, m_useTwoSideLightModel);

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

    // define the direction of the light beam. this is a little counter intuitive,
    // but the direction is actually specified with the GL_POSITION command.
    // OpenGL recognizes the light being a directional light source by detecting that
    // value position[3] is equal to zero
    cVector3d dir = getDir();
    float position[4];
    position[0] = (float)-dir(0) ;
    position[1] = (float)-dir(1) ;
    position[2] = (float)-dir(2) ;
    position[3] = 0.0f; // defines light to be of type directional
    glLightfv(m_glLightNumber, GL_POSITION, (const float *)&position);

#endif
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
