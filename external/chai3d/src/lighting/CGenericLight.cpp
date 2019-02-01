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
#include "lighting/CGenericLight.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    Constructor of cGenericLight.

    \param  a_world  Parent world in which the light source is created.
*/
//==============================================================================
cGenericLight::cGenericLight(cWorld* a_world)
{   
    // set world parent
    m_worldParent =  a_world;
    if (m_worldParent != NULL)
    {
        m_worldParent->addLightSource(this);
    }

    // set default ambient term
    m_ambient.set(0.4f, 0.4f, 0.4f, 1.0f);

    // set default diffuse term
    m_diffuse.set(0.7f, 0.7f, 0.7f, 1.0f);

    // set default specular term
    m_specular.set(1.0f, 1.0f, 1.0f, 1.0f);

    // light is disabled by default
    m_enabled = false;
               
    // disable two sided light model
    m_useTwoSideLightModel = GL_TRUE;

    // hide graphical representation of the light source.
    // enable only for debugging purposes.
    m_displayEnabled = false;
}


//==============================================================================
/*!
    Destructor of cGenericLight.
*/
//==============================================================================
cGenericLight::~cGenericLight()
{
}


//==============================================================================
/*!
    This method enables or disables the two sided lighting mode.

    \param  a_useTwoSideLightModel  Light mode.
*/
//==============================================================================
void cGenericLight::setUseTwoSideLightModel(bool a_useTwoSideLightModel)
{
    if (a_useTwoSideLightModel)
    {
        m_useTwoSideLightModel = GL_TRUE;
    }
    else
    {
        m_useTwoSideLightModel = GL_FALSE;
    }
}


//==============================================================================
/*!
    This method returns the status of the two sided lighting mode.

    \return __true__ if two-sided light mode is enabled. __false__ otherwise.
*/
//==============================================================================
bool cGenericLight::getUseTwoSideLightModel() const
{
    if (m_useTwoSideLightModel == GL_TRUE)
    {
        return (true);
    }
    else
    {
       return (false);
    }
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
