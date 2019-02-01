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
// Texture Filter Parameters Explained:
//------------------------------------------------------------------------------
/*
    The value of parameter number three depends on whether you are setting a
    GL_TEXTURE_MIN_FILTER variable or a GL_TEXTURE_MAG_FILTER variable.
    Here are the possible combinations:

    When the second parameter is GL_TEXTURE_MIN_FILTER, parameter three can be:
        GL_NEAREST_MIPMAP_NEAREST
        GL_LINEAR_MIPMAP_NEAREST
        GL_NEAREST_MIPMAP_LINEAR
        GL_LINEAR_MIPMAP_LINEAR
        GL_NEAREST
        GL_LINEAR

    When the second parameter is GL_TEXTURE_MAG_FILTER, parameter three can be:
        GL_LINEAR
        GL_NEAREST

        The filter value set for GL_TEXTURE_MIN_FILTER is used whenever a surface is
        rendered with smaller dimensions than its corresponding texture bitmap
        (far away objects). Whereas the filter value for GL_TEXTURE_MAG_FILTER is used in
        the exact opposite case when a surface is bigger than the texture being applied (near objects).

        There are more options for the min filter because it can potentially have mipmapping.
        However, it wouldn't make sense to apply mipmapping to the mag filter since close-up
        objects don't need it in the first place. Here's a list of all the possible combinations
        and how they impact what is rendered (first constant in the left-most column is the near
        object filter [mag]; second constant is the far object filter [min]):


    Filter Combination                          Bilinear Filtering    Bilinear Filtering    Mipmapping
    (MAG_FILTER/MIN_FILTER)                     (Near)                (Far)
    GL_NEAREST / GL_NEAREST_MIPMAP_NEAREST      Off                   Off                   Standard
    GL_NEAREST / GL_LINEAR_MIPMAP_NEAREST       Off                   On                    Standard
    GL_NEAREST / GL_NEAREST_MIPMAP_LINEAR       Off                   Off                   Use trilinear filtering
    GL_NEAREST / GL_LINEAR_MIPMAP_LINEAR        Off                   On                    Use trilinear filtering
    GL_NEAREST / GL_NEAREST                     Off                   Off                   None
    GL_NEAREST / GL_LINEAR                      Off                   On                    None
    GL_LINEAR / GL_NEAREST_MIPMAP_NEAREST       On                    Off                   Standard
    GL_LINEAR / GL_LINEAR_MIPMAP_NEAREST        On                    On                    Standard
    GL_LINEAR / GL_NEAREST_MIPMAP_LINEAR        On                    Off                   Use trilinear filtering
    GL_LINEAR / GL_LINEAR_MIPMAP_LINEAR         On                    On                    Use trilinear filtering
    GL_LINEAR / GL_NEAREST                      On                    Off                   None
    GL_LINEAR / GL_LINEAR                       On                    On                    None
*/
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#ifndef CGenericTextureH
#define CGenericTextureH
//------------------------------------------------------------------------------
#include "graphics/CColor.h"
#include "graphics/CRenderOptions.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CGenericTexture.h
    
    \brief
    Implements a base class for textures.
*/
//==============================================================================

//------------------------------------------------------------------------------
class cGenericTexture;
typedef std::shared_ptr<cGenericTexture> cGenericTexturePtr;
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \class      cGenericTexture
    \ingroup    materials

    \brief
    This class implements a base class for textures.

    \details
    cGenericTexture implements a base class for OpenGL textures.
*/
//==============================================================================
class cGenericTexture
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cGenericTexture.
    cGenericTexture() { m_enabled = true; };

    //! Destructor of cGenericTexture.
    virtual ~cGenericTexture() {};

    //! Shared cGenericTexture allocator.
    static cGenericTexturePtr create() { return (std::make_shared<cGenericTexture>()); }


    //-----------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

public:

    //! This method loads a texture image file.
    virtual bool loadFromFile(const std::string& a_fileName) { return (false); }

    //! This method saves a texture image file.
    virtual bool saveToFile(const std::string& a_fileName) { return (false); }

    //! This method enables texturing and set this texture as the current texture.
    virtual void renderInitialize(cRenderOptions& a_options) {}

    //! This method disables texturing and should be called after triangles have been rendered.
    virtual void renderFinalize(cRenderOptions& a_options) {}

    //! This method marks this texture for GPU update.
    virtual void markForUpdate() {}

    //! This method marks this texture for GPU deletion and reinitialization.
    virtual void markForDeleteAndUpdate() {}

    //! This method enables or disables texture mapping.
    void setEnabled(bool a_enabled) { m_enabled = a_enabled; }

    //! This method returns __true__ if the texture map is enabled, __false__ otherwise.
    bool getEnabled() const { return(m_enabled); }


    //-----------------------------------------------------------------------
    // PROTECTED MEMBERS:
    //--------------------------------------------------------------------------

protected:

    //! If __true__, this texture is enabled, __false__ otherwise.
    bool m_enabled;
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
