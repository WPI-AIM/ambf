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
    \author    Dan Morris
    \author    Adnan Munawar
    \version   3.2.0 $Rev: 1869 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CTextureCubeMapH
#define CTextureCubeMapH
//------------------------------------------------------------------------------
#include "graphics/CColor.h"
#include "graphics/CImage.h"
#include "materials/CTexture1d.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file   CTexture2d.h

    \brief
    Implements 2D textures.
*/
//==============================================================================

//------------------------------------------------------------------------------
class cTextureCubeMap;
typedef std::shared_ptr<cTextureCubeMap> cTextureCubeMapPtr;
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \class      cTextureCubeMap
    \ingroup    materials

    \brief
    This class implements a 2D texture map.

    \details
    This class implements a 2D texture map.
*/
//==============================================================================
class cTextureCubeMap : public cTexture1d
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cTexture2d.
    cTextureCubeMap();

    //! Destructor of cTexture2d.
    virtual ~cTextureCubeMap();

    //! Shared cTextureCubeMap allocator.
    static cTextureCubeMapPtr create() { return (std::make_shared<cTextureCubeMap>()); }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

public:

    //! This method creates a copy of itself.
    cTextureCubeMapPtr copy();

    //! This method enables texturing and sets this texture as the current texture.
    virtual void renderInitialize(cRenderOptions& a_options);

    //! This method disable texture once triangles have been rendered.
    virtual void renderFinalize(cRenderOptions& a_options);

    //! This method set the texture wrap mode of __t__.
    virtual void setWrapModeT(const GLint& a_wrapModeT);

    //! This method set the texture wrap mode of __r__.
    virtual void setWrapModeR(const GLint& a_wrapModeR);

    //! This method returns the texture wrap mode of __t__.
    GLint getWrapModeT() const { return (m_wrapModeT); }

    //! This method returns the texture wrap mode of __r__.
    GLint getWrapModeR() const { return (m_wrapModeR); }

    //! Image object associated with texture (use this to get data about the texture itself).
    cImagePtr m_images[6];
    // RIGHT
    // LEFT
    // TOP
    // BOTTOM
    // FRONT
    // BACK


    //--------------------------------------------------------------------------
    // PROTECTED METHODS:
    //--------------------------------------------------------------------------

protected:

    //! This method resets all internal variables. This function should be called only by constructors.
    virtual void reset();

    //! This method updates this texture to GPU.
    virtual void update(cRenderOptions& a_options);


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS:
    //--------------------------------------------------------------------------

protected:

    //! Texture wrap parameter along __t__ (GL_REPEAT or GL_CLAMP).
    GLint m_wrapModeT;

    //! Texture wrap parameter along __r__ (GL_REPEAT or GL_CLAMP).
    GLint m_wrapModeR;
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
