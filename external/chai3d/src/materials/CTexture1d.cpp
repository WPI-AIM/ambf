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
    \version   3.2.0 $Rev: 2162 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "materials/CTexture1d.h"
//------------------------------------------------------------------------------
using namespace std;
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    Constructor of cTexture1d.
*/
//==============================================================================
cTexture1d::cTexture1d()
{
    // set default texture unit
    m_textureUnit = GL_TEXTURE0;

    // create image
    m_image = cImage::create();

    // initialize internal variables
    reset();
}


//==============================================================================
/*!
    Destructor of cTexture1d.
*/
//==============================================================================
cTexture1d::~cTexture1d()
{
    if (m_textureID != 0)
    {
        #ifdef C_USE_OPENGL
        glDeleteTextures(1, &m_textureID);
        #endif
        
        m_textureID = 0;
    }
}


//==============================================================================
/*!
    This method creates a copy of itself.

    \return Pointer to new object.
*/
//==============================================================================
cTexture1dPtr cTexture1d::copy()
{
    // create new instance
    cTexture1dPtr obj = cTexture1d::create();

    // create copy of image data
    obj->m_image = m_image->copy();

    // copy all variables
    obj->m_enabled                  = m_enabled;
    obj->m_wrapModeS                = m_wrapModeS;
    obj->m_magFunction              = m_magFunction;
    obj->m_minFunction              = m_minFunction;
    obj->m_useMipmaps               = m_useMipmaps;
    obj->m_useSphericalMapping      = m_useSphericalMapping;
    obj->m_environmentMode          = m_environmentMode;

    // return
    return (obj);
}


//==============================================================================
/*!
    This method resets all internal variables. This methos must only be called
    by constructors.
*/
//==============================================================================
void cTexture1d::reset()
{
    // id number provided by OpenGL once texture is stored in graphics
    // card memory
    m_textureID = 0;

    // texture has not yet been rendered
    m_updateTextureFlag = true;
    m_deleteTextureFlag = false;

    // Tile the texture in X. (GL_REPEAT or GL_CLAMP)
    m_wrapModeS = GL_CLAMP_TO_EDGE;

    // set environmental mode (GL_MODULATE, GL_DECAL, GL_BLEND, GL_REPLACE)
    m_environmentMode = GL_MODULATE;

    // set environmental color
    m_color.set(1.0, 1.0, 1.0, 0.0);

    // set spherical mode
    m_useSphericalMapping = false;

    // use mipmaps
    m_useMipmaps = false;

    // default settings
    m_magFunctionMipmapsOFF = GL_LINEAR;
    m_minFunctionMipmapsOFF = GL_LINEAR;
    m_magFunctionMipmapsON = GL_LINEAR;
    m_minFunctionMipmapsON = GL_LINEAR_MIPMAP_LINEAR;

    // set the magnification function. (GL_NEAREST or GL_LINEAR)
    m_magFunction = m_magFunctionMipmapsOFF;

    // set the minifying function. (GL_NEAREST or GL_LINEAR)
    m_minFunction = m_minFunctionMipmapsOFF;
}


//==============================================================================
/*!
    This method enables texturing and sets this texture as the current texture.

    \param  a_options  Rendering options.
*/
//==============================================================================
void cTexture1d::renderInitialize(cRenderOptions& a_options)
{
#ifdef C_USE_OPENGL

    // check if shadow is enabled
    if (!m_enabled) { return; }

    // check if materials should be rendered
    if (!a_options.m_render_textures) { return; }

    // check image texture
    if (m_image->isInitialized() == 0) return;

    // Only check residency in memory if we weren't going to
    // update the texture anyway...
    if (m_updateTextureFlag == false)
    {
        if (m_textureID != 0)
        {
            if (glIsTexture(m_textureID) == false)
            {
                m_textureID = 0;
                m_updateTextureFlag = true;
            }
        }
    }

    // setup texture settings
    glActiveTexture(m_textureUnit);
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_COMBINE);

    glTexEnvi(GL_TEXTURE_ENV, GL_COMBINE_RGB, GL_REPLACE);
    glTexEnvi(GL_TEXTURE_ENV, GL_SOURCE0_RGB, GL_PREVIOUS);
    glTexEnvi(GL_TEXTURE_ENV, GL_OPERAND0_RGB, GL_SRC_COLOR);

    glTexEnvi(GL_TEXTURE_ENV, GL_COMBINE_ALPHA, GL_ADD_SIGNED);
    glTexEnvi(GL_TEXTURE_ENV, GL_SOURCE0_ALPHA, GL_PREVIOUS);
    glTexEnvi(GL_TEXTURE_ENV, GL_OPERAND0_ALPHA, GL_SRC_ALPHA);
    glTexEnvi(GL_TEXTURE_ENV, GL_SOURCE1_ALPHA, GL_TEXTURE);
    glTexEnvi(GL_TEXTURE_ENV, GL_OPERAND1_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // enable texturing
    glEnable(GL_TEXTURE_1D);

    // setup texture or update
    if (m_updateTextureFlag)
    {
        // update texture map
        update(a_options);
        m_updateTextureFlag = false;
    }
    else
    {
        // make this the current texture
        glBindTexture(GL_TEXTURE_1D, m_textureID);
    }

    // enable or disable spherical mapping
    if (m_useSphericalMapping)
    {
        glEnable(GL_TEXTURE_GEN_S);
        glTexGeni(GL_S, GL_TEXTURE_GEN_MODE, GL_SPHERE_MAP);
    }
    else
    {
        glDisable(GL_TEXTURE_GEN_S);
    }
    
    // Sets the wrap parameter for texture coordinate s to either
    // GL_CLAMP or GL_REPEAT.
    glTexParameteri(GL_TEXTURE_1D ,GL_TEXTURE_WRAP_S, m_wrapModeS);

    // Set the texture magnification function to either GL_NEAREST or GL_LINEAR.
    glTexParameteri(GL_TEXTURE_1D ,GL_TEXTURE_MAG_FILTER, m_magFunction);

    // Set the texture minifying function to either GL_NEAREST or GL_LINEAR.
    glTexParameteri(GL_TEXTURE_1D ,GL_TEXTURE_MIN_FILTER, m_minFunction);

    // set the environment mode (GL_MODULATE, GL_DECAL, GL_BLEND, GL_REPLACE)
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, m_environmentMode);

    // make this the current texture
    glBindTexture(GL_TEXTURE_1D, m_textureID);

    // set the environmental color
    glTexEnvfv(GL_TEXTURE_ENV, GL_TEXTURE_ENV_COLOR, &m_color.getData()[0]);

    // set default vertex color which combined with the texture (white).
    glColor4f(1.0, 1.0, 1.0, 1.0);

#endif
}


//==============================================================================
/*!
    This method is called to disable this texture once all triangles have 
    been rendered.

    \param  a_options  Rendering options.
*/
//==============================================================================
void cTexture1d::renderFinalize(cRenderOptions& a_options)
{
#ifdef C_USE_OPENGL
    
    if (m_enabled)
    {
        glActiveTexture(m_textureUnit);
        glDisable(GL_TEXTURE_1D);
    }
    
#endif
}


//==============================================================================
/*!
    This method loads a texture image file.

    \param  a_fileName  Filename.
*/
//==============================================================================
bool cTexture1d::loadFromFile(const string& a_fileName)
{
    return (m_image->loadFromFile(a_fileName));
}


//==============================================================================
/*!
    This method saves this texture to a file.

    \param  a_fileName  Filename.
*/
//==============================================================================
bool cTexture1d::saveToFile(const string& a_fileName)
{
    return (m_image->saveToFile(a_fileName));
}


//==============================================================================
/*!
    This method marks this texture for GPU update. This method should be called
    if the image data has changed at the CPU level and needs to be updated to
    the GPU memory.
*/
//==============================================================================
void cTexture1d::markForUpdate() 
{ 
    m_updateTextureFlag = true;
}


//==============================================================================
/*!
    This method marks the texture for deletion from GPU memory and update.
    This method should be called if the image format or size has changed.
*/
//==============================================================================
void cTexture1d::markForDeleteAndUpdate() 
{ 
    m_deleteTextureFlag = true;
    m_updateTextureFlag = true;
}


//==============================================================================
/*!
    This method updates the texture from memory data to GPU.

    \param  a_options  Rendering options.
*/
//==============================================================================
void cTexture1d::update(cRenderOptions& a_options)
{
#ifdef C_USE_OPENGL

    if (m_deleteTextureFlag)
    {
        if (m_textureID != 0)
        {
            glDeleteTextures(1, &m_textureID);
            m_textureID = 0;
        }

        m_deleteTextureFlag = false;
    }

    if (m_textureID == 0)
    {
        glGenTextures(1, &m_textureID);
        glBindTexture(GL_TEXTURE_1D, m_textureID);

        glTexParameteri(GL_TEXTURE_1D ,GL_TEXTURE_WRAP_S, m_wrapModeS);
        glTexParameteri(GL_TEXTURE_1D ,GL_TEXTURE_MAG_FILTER, m_magFunction);
        glTexParameteri(GL_TEXTURE_1D ,GL_TEXTURE_MIN_FILTER, m_minFunction);

        glPixelStorei(GL_UNPACK_ALIGNMENT,   1);
        glPixelStorei(GL_UNPACK_ROW_LENGTH,  0);
        glPixelStorei(GL_UNPACK_SKIP_ROWS,   0);
        glPixelStorei(GL_UNPACK_SKIP_PIXELS, 0);

        glTexImage1D(GL_TEXTURE_1D,
            0,
            GL_RGBA,
            (GLsizei)(m_image->getWidth() * m_image->getHeight()),
            0,
            m_image->getFormat(),
            m_image->getType(),
            m_image->getData());
    }
    else
    {
        glBindTexture(GL_TEXTURE_1D, m_textureID);

        glTexParameteri(GL_TEXTURE_1D ,GL_TEXTURE_WRAP_S, m_wrapModeS);
        glTexParameteri(GL_TEXTURE_1D ,GL_TEXTURE_MAG_FILTER, m_magFunction);
        glTexParameteri(GL_TEXTURE_1D ,GL_TEXTURE_MIN_FILTER, m_minFunction);

        glPixelStorei(GL_UNPACK_ALIGNMENT,   1);
        glPixelStorei(GL_UNPACK_ROW_LENGTH,  0);
        glPixelStorei(GL_UNPACK_SKIP_ROWS,   0);
        glPixelStorei(GL_UNPACK_SKIP_PIXELS, 0);

        glTexSubImage1D(GL_TEXTURE_1D, 
                        0, 
                        0, 
                        (GLsizei)m_image->getWidth(),
                        m_image->getFormat(),
                        m_image->getType(),
                        m_image->getData());
    }
    
    if (m_useMipmaps)
    {
        glEnable (GL_TEXTURE_1D);
        glGenerateMipmap(GL_TEXTURE_1D);
    }

#endif
}


//==============================================================================
/*!
    This method sets the wrap parameter for texture coordinate S to either 
    GL_CLAMP or GL_REPEAT. \n\n
    
    GL_CLAMP causes __s__ coordinates to be clamped to the range [0,1] and is 
    useful for preventing wrapping artifacts when mapping a single image onto 
    an object. \n\n

    GL_REPEAT causes the integer part of the __s__ coordinate to be ignored;
    OpenGL uses only the fractional part, thereby creating a repeating pattern.
    Border texture elements are accessed only if wrapping is set to GL_CLAMP.

    \param  a_wrapModeS  Value shall be either GL_REPEAT or GL_CLAMP.
*/
//==============================================================================
void cTexture1d::setWrapModeS(const GLint& a_wrapModeS)
{
    m_wrapModeS = a_wrapModeS;
}


//==============================================================================
/*!
    This method sets the texture magnification function to either GL_NEAREST 
    or GL_LINEAR. \n\n

    The texture magnification function is used when the pixel being 
    textured maps to an area less than or equal to one texture element.

    \param  a_magFunction  value shall be either GL_NEAREST or GL_LINEAR.
*/
//==========================================================================
void cTexture1d::setMagFunction(const GLint a_magFunction)
{
    m_magFunction = a_magFunction;

    if (m_useMipmaps)
    {
             
            m_magFunctionMipmapsON = m_magFunction;
    }
    else
    {
            m_magFunctionMipmapsOFF = m_magFunction;
    }
}


//==============================================================================
/*!
    This method sets the minifying value. \n\n

    The texture minifying function is used whenever the pixel being textured
    maps to an area greater than one texture element. There are six defined
    minifying functions. Two of them use the nearest one or nearest four
    texture elements to compute the texture value. The other four use mipmaps.
    A mipmap is an ordered set of arrays representing the same image at
    progressively lower resolutions. If the texture has dimensions 2nx2m
    there are max(n, m) + 1 mipmaps. The first mipmap is the original texture,
    with dimensions 2nx2m. Each subsequent mipmap has dimensions 2k1x2l1 where 2
    kx2l are the dimensions of the previous mipmap, until either k = 0 or l = 0.
    At that point, subsequent mipmaps have dimension 1x2l1 or 2k1x1 until the
    final mipmap, which has dimension 1x1. Mipmaps are defined using
    glTexImage1D or glTexImage2D with the level-of-detail argument indicating
    the order of the mipmaps. Level 0 is the original texture; level bold
    max(n, m) is the final 1x1 mipmap.

    \param  a_minFunction  value shall be either GL_NEAREST or GL_LINEAR.
*/
//==========================================================================
void cTexture1d::setMinFunction(const GLint a_minFunction)
{
    m_minFunction = a_minFunction;

    if (m_useMipmaps)
    {
             
            m_minFunctionMipmapsON = m_minFunction;
    }
    else
    {
            m_minFunctionMipmapsOFF = m_minFunction;
    }
}


//==============================================================================
/*!
    This method enables or disables mipmapping. Please note that this function 
    does not work with all graphics cards! (ATI graphic cards seem to have problems)

    \param  a_useMipmaps  Mipmaps status.
*/
//==============================================================================
void cTexture1d::setUseMipmaps(bool a_useMipmaps) 
{ 
    m_useMipmaps = a_useMipmaps;
    
    if (m_useMipmaps)
    {
        m_minFunction = m_minFunctionMipmapsON;
        m_magFunction = m_magFunctionMipmapsON;
    }
    else
    {
        m_minFunction = m_minFunctionMipmapsOFF;
        m_magFunction = m_magFunctionMipmapsOFF;
    }
}


//==============================================================================
/*!
    This method assign image associated with the texture.

    \param  a_image  Pointer to image.

    \return __true__ if operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cTexture1d::setImage(cImagePtr a_image)
{
    // sanity check
    if (!a_image) return (false);

    // store new image
    m_image = a_image;

    // mark texture for update
    markForUpdate();

    // success
    return (true);
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
