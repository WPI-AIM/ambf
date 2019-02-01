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
    \version   3.2.0 $Rev: 2159 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CWorldH
#define CWorldH
//------------------------------------------------------------------------------
#include "display/CCamera.h"
#include "graphics/CColor.h"
#include "graphics/CTriangleArray.h"
#include "graphics/CFog.h"
#include "materials/CTexture2d.h"
#include "world/CGenericObject.h"
//------------------------------------------------------------------------------
#include <vector>
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
class cGenericLight;
class cShadowMap;
//------------------------------------------------------------------------------
//! The maximum number of lights that we expect OpenGL to support
#define C_MAXIMUM_OPENGL_LIGHT_COUNT 8
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CWorld.h

    \brief
    Implementation of a virtual world.
*/
//==============================================================================

//==============================================================================
/*!
    \class      cWorld
    \ingroup    world

    \brief
    This class implements a virtual world.

    \details
    cWorld defines the root of node the CHAI3D scene graph. It stores 
    lights, cameras, tools, and objects.
*/
//==============================================================================
class cWorld : public cGenericObject
{
    friend class cGenericLight;

    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cWorld.
    cWorld();
    
    //! Destructor of cWorld.
    virtual ~cWorld();


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - BACKGROUND PROPERTIES
    //--------------------------------------------------------------------------

public:

    //! This method sets the background color of this world.
    void setBackgroundColor(const GLfloat a_red, 
                            const GLfloat a_green,
                            const GLfloat a_blue);

    //! This method sets the background color of this world.
    void setBackgroundColor(const cColorf& a_color);

    //! This method returns the background color of this world.
    cColorf getBackgroundColor() const { return (m_backgroundColor); }


    //-----------------------------------------------------------------------
    // PUBLIC METHODS - LIGHT SOURCES:
    //-----------------------------------------------------------------------

public:

    //! This method enables or disables the rendering of this world's light sources.
    void enableLightSourceRendering(bool enable) { m_renderLightSources = enable; }

    //! This method returns a pointer to a particular light source (between 0 and MAXIMUM_OPENGL_LIGHT_COUNT-1).
    virtual cGenericLight* getLightSource(int index);  


    //-----------------------------------------------------------------------
    // PUBLIC METHODS - COLLISION / INTERACTION:
    //-----------------------------------------------------------------------

public:

    //! This method computes any collision between a segment and all objects in this world.
    virtual bool computeCollisionDetection(const cVector3d& a_segmentPointA,
                                           const cVector3d& a_segmentPointB,
                                           cCollisionRecorder& a_recorder,
                                           cCollisionSettings& a_settings);

    //! This method updates the geometric relationship between the tool and this world.
    virtual void computeLocalInteraction(const cVector3d& a_toolPos,
                                         const cVector3d& a_toolVel,
                                         const unsigned int a_IDN);


    //-----------------------------------------------------------------------
    // PUBLIC METHODS - SHADOW CASTING:
    //-----------------------------------------------------------------------

public:

    //! This method enables or disables shadow casting.
    void setUseShadowCasting(bool a_enabled) { m_useShadowCasting = a_enabled; }

    //! This method returns __true__ if shadow casting is enabled, __false__ otherwise.
    bool getUseShadowCastring() { return(m_useShadowCasting); }

    //! This method sets the shadow Intensity. (0.0 = no shadow - full light. 1.0 = full shadow - no light)
    void setShadowIntensity(double a_intensity) { m_shadowIntensity = cClamp01(a_intensity); }

    // This method returns the shadow intensity.
    double getShadowIntensity() { return (m_shadowIntensity); }

    //! This method updates all shadow maps.
    virtual bool updateShadowMaps(const bool a_mirrorX = false,
                                  const bool a_mirrorY = false);


    //--------------------------------------------------------------------------
    // PUBLIC MEMBERS:
    //--------------------------------------------------------------------------

public:

    //! Background color of the world.
    cColorf m_backgroundColor;
    
    //! Fog property.
    cFog* m_fog;

    //! List of light sources.
    std::vector<cGenericLight*> m_lights;

    //! List of active shadow maps.
    std::list<cShadowMap*> m_shadowMaps;

    //! It's useful to store the world's modelview matrix, for rendering stuff in "global" coordinates.
    double m_worldModelView[16];


    //-----------------------------------------------------------------------
    // VIRTUAL PROTECTED METHODS:
    //-----------------------------------------------------------------------

protected:

    //! This method renders all light sources of this world.
    virtual void render(cRenderOptions& a_options);


    //--------------------------------------------------------------------------
    // PROTECTED METHODS:
    //--------------------------------------------------------------------------

protected:

    //! This method adds a light source to this world.
    bool addLightSource(cGenericLight* a_light);

    //! This method removed a light source from this world.
    bool removeLightSource(cGenericLight* a_light);

    //! This method returns __true__ if shadow casting is supported on this hardware, __false__ otherwise.
    bool isShadowCastingSupported();


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS:
    //--------------------------------------------------------------------------

protected:

    //! If __true__ then light sources should be used.
    bool m_renderLightSources; 

    //! Shadow intensity. Values ranges between 0.0 and 1.0 (0.0 = no shadow - full light. 1.0 = full shadow - no light).
    double m_shadowIntensity;

    //! If __true__ then shadow maps are used.
    bool m_useShadowCasting;
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------

