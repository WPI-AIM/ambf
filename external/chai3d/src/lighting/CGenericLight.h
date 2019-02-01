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
    \version   3.2.0 $Rev: 2167 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CGenericLightH
#define CGenericLightH
//------------------------------------------------------------------------------
#include "graphics/CColor.h"
#include "lighting/CShadowMap.h"
#include "math/CMaths.h"
#include "world/CGenericObject.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CGenericLight.h

    \brief
    Implements a base class for light sources.
*/
//==============================================================================

//==============================================================================
/*!
    \class      cGenericLight
    \ingroup    lighting

    \brief
    This class implements a base class for modeling light sources.

    \details
    This class implements a base class for modeling light sources.
*/
//==============================================================================
class cGenericLight : public cGenericObject
{
    friend class cWorld;
    
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cGenericLight.
    cGenericLight(cWorld* a_world);
    
    //! Destructor of cGenericLight.
    virtual ~cGenericLight();


    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //-------------------------------------------------------------------------- 

public:

    //! This method enables or disables the two sided lighting mode.
    void setUseTwoSideLightModel(bool a_useTwoSideLightModel);

    //! This method returns __true__ if the two sided lighting mode is enabled, __false__ otherwise.
    bool getUseTwoSideLightModel() const;

    //! This method enables or disables the graphic representation of light source. (To be used for debugging purposes).
    void setDisplayEnabled(bool a_enabled) { m_displayEnabled = a_enabled; }

    //! This method returns __true__ of graphic display representation of the light source is enabled, __false__ otherwise.
    bool getDisplayEnabled() const { return (m_displayEnabled); }

    //! This method sets the OpenGL Light ID number. Assigning these numbers is normally handled by the world in which the light source is located.
    inline void setGLLightNumber(const GLint a_glLightNumber) { m_glLightNumber = a_glLightNumber; }

    //! This method returns the OpenGL Light ID number for this light source.
    inline GLint getGLLightNumber() const { return(m_glLightNumber); }


    //-----------------------------------------------------------------------
    // PUBLIC MEMBERS:
    //-----------------------------------------------------------------------

public:

    //! Ambient light component.
    cColorf m_ambient;
    
    //! Diffuse light component.
    cColorf m_diffuse;
    
    //! Specular light component.
    cColorf m_specular;


    //-----------------------------------------------------------------------
    // VIRTUAL PROTECTED METHODS:
    //-----------------------------------------------------------------------

protected:

    //! This method renders the lighting properties of this light source using OpenGL.
    virtual void renderLightSource(cRenderOptions& a_options){}

    //! This method renders a graphical representation (display model) of the light source. (used for debugging purposes typically).
    virtual void render(cRenderOptions& a_options){}


    //-----------------------------------------------------------------------
    // PROTECTED MEMBERS:
    //-----------------------------------------------------------------------

protected:

    //! Parent world in which light source is located.
    cWorld* m_worldParent;

    //! OpenGL reference number for the current light source. This number ranges from 0 to 7.
    GLint m_glLightNumber;

    //! If __true__ then a two sided light source model is used.
    GLint m_useTwoSideLightModel;

    //! If __true__ then a graphical representation of the light source is rendered.
    bool m_displayEnabled;
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------

