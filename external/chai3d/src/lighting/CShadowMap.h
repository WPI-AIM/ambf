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
    \version   3.2.0 $Rev: 2185 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CShadowMapH
#define CShadowMapH
//------------------------------------------------------------------------------
#include "display/CFrameBuffer.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*! 
    \file       CShadowMap.h

    \brief
    Implements a shadow map.
*/
//==============================================================================

//==============================================================================
/*!
    \class      cShadowMap
    \ingroup    lighting

    \brief
    This class implements a shadow map for spot lights (\ref cSpotLight).

    \details
    This class implements a shadow map texture for spot lights (\ref cSpotLight).
*/
//==============================================================================
class cShadowMap : public cFrameBuffer
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cShadowMap.
    cShadowMap();

    //! Destructor of cShadowMap.
    virtual ~cShadowMap() {};


    //-----------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

public:

    //! This method enables or disables this shadow map.
    void setEnabled(bool a_enabled) { m_enabled = a_enabled; }

    //! This method returns __true__ if the this shadow map is enabled, __false__ otherwise.
    bool getEnabled() const { return(m_enabled); }

    //! This method sets the quality resolution of the shadow map to 256 x 256 pixels.
    void setQualityVeryLow() { setSize(256, 256); }

    //! This method sets the quality resolution of the shadow map to 512 x 512 pixels.
    void setQualityLow() { setSize(512, 512); }

    //! This method sets the quality resolution of the shadow map to 1024 x 1024 pixels.
    void setQualityMedium() { setSize(1024, 1024); }

    //! This method sets the quality resolution of the shadow map to 2048 x 2048 pixels.
    void setQualityHigh() { setSize(2048, 2048); }

    //! This method sets the quality resolution of the shadow map to 4096 x 4096 pixels.
    void setQualityVeryHigh() { setSize(4096, 4096); }


    //-----------------------------------------------------------------------
    // PUBLIC MEMBERS:
    //--------------------------------------------------------------------------

public:

    //! Projection matrix of the light source creating this shadow.
    cTransform m_lightProjectionMatrix;

    //! View matrix of the light source creating this shadow.
    cTransform m_lightViewMatrix;


    //-----------------------------------------------------------------------
    // PUBLIC METHODS: (CHAI3D INTERNAL)
    //--------------------------------------------------------------------------

public:

    //! This method contains the OpenGL rendering code for shadow maps. (Do not call this code directly.)
    virtual void render(cRenderOptions& a_options);
    
    //! This method updates the shadow map. (Do not call this code directly.)
    bool updateMap(cWorld* a_world,
                   const cVector3d& a_lightPos,
                   const cVector3d& a_lightLookat,
                   const cVector3d& a_lightUp,
                   const double a_lightFieldViewAngle,
                   const double a_distanceNear,
                   const double a_distanceFar,
                   const double a_mirrorH,
                   const double a_mirrorV);


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS:
    //--------------------------------------------------------------------------

protected:

    //! If __true__, then shadow map is enabled, __false__ otherwise.
    bool m_enabled;
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
