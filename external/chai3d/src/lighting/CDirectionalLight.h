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
#ifndef CDirectionalLightH
#define CDirectionalLightH
//------------------------------------------------------------------------------
#include "lighting/CGenericLight.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CDirectionalLight.h

    \brief
    Implements a directional light source.
*/
//==============================================================================

//==============================================================================
/*!
    \class      cDirectionalLight
    \ingroup    lighting

    \brief
    This class implements a directional light source

    \details
    This class implements a directional light source. A directional 
    light source is treated as though it is located infinitely far away from 
    the scene, therefore only the lighting direction needs to be defined.
    The effect of an infinite location is that the rays of light can be 
    considered parallel by the time they reach an object. An example of a 
    real-world directional light source is the sun.
*/
//==============================================================================
class cDirectionalLight : public virtual cGenericLight
{
    friend class cCamera;

    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cDirectionalLight.
    cDirectionalLight(cWorld* a_world);
    
    //! Destructor of cDirectionalLight.
    virtual ~cDirectionalLight();


    //-----------------------------------------------------------------------
    // PUBLIC METHODS:
    //-----------------------------------------------------------------------

public:

    //! This method sets the direction of the light beam.
    void setDir(const cVector3d& a_direction);

    //! This method sets the direction of the light beam.
    void setDir(const double a_x, const double a_y, const double a_z);

    //! This method returns the direction of the light beam.
    cVector3d getDir() const { return (m_globalRot.getCol0()); }


    //-----------------------------------------------------------------------
    // VIRTUAL PROTECTED METHODS:
    //-----------------------------------------------------------------------

protected:

    //! This method renders the lighting properties of this light source using OpenGL.
    virtual void renderLightSource(cRenderOptions& a_options);

    //! This method renders a graphical representation (display model) of the light source. (used for debugging purposes typically).
    virtual void render(cRenderOptions& a_options){}
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------

