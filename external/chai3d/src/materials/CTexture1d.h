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
#ifndef CTexture1dH
#define CTexture1dH
//------------------------------------------------------------------------------
#include "graphics/CColor.h"
#include "graphics/CImage.h"
#include "materials/CGenericTexture.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*! 
    \file   CTexture1d.h

    \brief
    Implements 1D textures.
*/
//==============================================================================

//------------------------------------------------------------------------------
class cTexture1d;
typedef std::shared_ptr<cTexture1d> cTexture1dPtr;
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \class      cTexture1d
    \ingroup    materials

    \brief
    This class implements a 1D texture map.

    \details
    This class implements a 1D texture map.
*/
//==============================================================================
class cTexture1d : public cGenericTexture
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cTexture1d.
    cTexture1d();

    //! Destructor of cTexture1d.
    virtual ~cTexture1d();

    //! Shared cTexture1d allocator.
    static cTexture1dPtr create() { return (std::make_shared<cTexture1d>()); }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

public:

    //! This method set the texture unit where a_textureUnit is GL_TEXTUREi_ARB, where 0i<GL_MAX_TEXTURE_UNITS_ARB.
    inline void setTextureUnit(const GLenum a_textureUnit) { m_textureUnit = a_textureUnit; }

    //! This method returns the texture unit value.
    inline GLenum getTextureUnit() const { return (m_textureUnit); }

    //! This method sets the texture ID.
    void setTextureId(const GLuint a_textureId) { m_textureID = a_textureId; }

    //! This method returns the texture ID.
    GLuint getTextureId() { return(m_textureID); }

    //! This method creates a copy of itself.
    cTexture1dPtr copy();

    //! This method load a texture image from a file.
    virtual bool loadFromFile(const std::string& a_fileName);

    //! This method save a texture image to a file.
    virtual bool saveToFile(const std::string& a_fileName);

    //! This method enables texturing and sets this texture as the current texture.
    virtual void renderInitialize(cRenderOptions& a_options);

    //! This method disables texture rendering. This method should be called after triangles have been rendered.
    virtual void renderFinalize(cRenderOptions& a_options);

    //! This method marks this texture for GPU update.
    virtual void markForUpdate();

    //! This method marks this texture for GPU deletion and reinitialization.
    virtual void markForDeleteAndUpdate();

    //! This method sets the environment mode (GL_MODULATE, GL_DECAL, GL_BLEND, GL_REPLACE).
    void setEnvironmentMode(const GLint& a_environmentMode) { m_environmentMode = a_environmentMode; }

    //! This method returns the environment mode status.
    GLint getEnvironmentMode() { return (m_environmentMode); }

    //! This method set the texture wrap mode of __s__.
    virtual void setWrapModeS(const GLint& a_wrapModeS);

    //! This method returns the texture wrap mode of __s__.
    GLint getWrapModeS() const { return (m_wrapModeS); }

    //! This method sets the magnification function value.
    void setMagFunction(const GLint a_magFunction);

    //! This method returns the current magnification function value.
    GLint getMagFunction() const { return (m_magFunction); }

    //! This method sets the minification function value.
    void setMinFunction(const GLint a_minFunction);

    //! This method returns the current minification function value.
    GLint getMinFunction() const { return (m_minFunction); }

    //! This method enables of disables the spherical mapping mode.
    void setSphericalMappingEnabled(const bool a_enabled) { m_useSphericalMapping = a_enabled; }

    //! This method returns thestatus of the spherical mapping mode.
    bool getSphericalMappingEnabled() const { return (m_useSphericalMapping); }

    //! This method enables or disables the use of mipmaps.
    void setUseMipmaps(bool a_useMipmaps);

    //! This returns the the status of mipmaps.
    bool getUseMipmaps() { return (m_useMipmaps); }

    //! This method assigns an image to this texture.
    bool setImage (cImagePtr a_image);


    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

public:

    //! Image object associated with texture (use this to get data about the texture itself).
    cImagePtr m_image;

    //! Environmental color.
    cColorf m_color;


    //--------------------------------------------------------------------------
    // PROTECTED METHODS:
    //--------------------------------------------------------------------------

protected:

    //! This method resets all internal variables. This function should be called only by constructors.
    virtual void reset();

    //! This method update the OpenGL texture to GPU.
    virtual void update(cRenderOptions& a_options);


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS:
    //--------------------------------------------------------------------------

protected:

    //! If __true__, texture bitmap needs to be deleted from GPU memory.
    bool m_deleteTextureFlag;

    //! If __true__, texture bitmap needs to be updated to GPU memory.
    bool m_updateTextureFlag;

    //! OpenGL texture ID number.
    GLuint m_textureID;

    //! Texture wrap parameter along __s__ (GL_REPEAT or GL_CLAMP).
    GLint m_wrapModeS;

    //! Texture magnification function value. (GL_NEAREST or GL_LINEAR).
    GLint m_magFunction;

    //! Texture minification function value. (GL_NEAREST or GL_LINEAR).
    GLint m_minFunction;

    //! If __true__, mipmaps are enabled.
    bool m_useMipmaps;

    //! If __true__, spherical mapping is enable.
    bool m_useSphericalMapping;

    //! OpenGL texture mode (GL_MODULATE, GL_DECAL, GL_BLEND, or GL_REPLACE).
    GLint m_environmentMode;

    //! Texture unit number.
    GLenum m_textureUnit;

    //! Texture magnification function when mipmapping is OFF.
    GLint m_magFunctionMipmapsOFF;

    //! Texture minification function when mipmapping is OFF.
    GLint m_minFunctionMipmapsOFF;

    //! Texture magnification function when mipmapping is ON.
    GLint m_magFunctionMipmapsON;

    //! Texture minification function when mipmapping is ON.
    GLint m_minFunctionMipmapsON;
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
