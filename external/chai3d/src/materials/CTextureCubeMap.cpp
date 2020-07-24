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
    \version   3.2.0 $Rev: 2162 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "materials/CTextureCubeMap.h"
//------------------------------------------------------------------------------
using namespace std;
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    Constructor of cTextureCubeMap.
*/
//==============================================================================
cTextureCubeMap::cTextureCubeMap()
{
    // set default texture unit
    m_textureUnit = GL_TEXTURE0;

    // initialize internal variables
    reset();
}


//==============================================================================
/*!
    Destructor of cTextureCubeMap.
*/
//==============================================================================
cTextureCubeMap::~cTextureCubeMap()
{
    return;
}


//==============================================================================
/*!
    This method creates a copy of itself.

    \return Pointer to new object.
*/
//==============================================================================
cTextureCubeMapPtr cTextureCubeMap::copy()
{
    // create new instance
    cTextureCubeMapPtr obj = cTextureCubeMap::create();

    // create copy of image data
    obj->m_image = m_image->copy();

    // copy all variables
    obj->m_enabled                  = m_enabled;
    obj->m_wrapModeS                = m_wrapModeS;
    obj->m_wrapModeT                = m_wrapModeT;
    obj->m_wrapModeR                = m_wrapModeR;
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
    This method resets all internal variables. This function should be called
    only by constructors.
*/
//==============================================================================
void cTextureCubeMap::reset()
{
    // id number provided by OpenGL once texture is stored in graphics
    // card memory
    m_textureID = 0;

    // texture has not yet been rendered
    m_updateTextureFlag = true;
    m_deleteTextureFlag = false;

    // Tile the texture in X. (GL_REPEAT or GL_CLAMP)
    m_wrapModeS = GL_CLAMP_TO_EDGE;

    // Tile the texture in Y. (GL_REPEAT or GL_CLAMP)
    m_wrapModeT = GL_CLAMP_TO_EDGE;

    // Tile the texture in R. (GL_REPEAT or GL_CLAMP)
    m_wrapModeR = GL_CLAMP_TO_EDGE;

    // set environmental mode (GL_MODULATE, GL_DECAL, GL_BLEND, GL_REPLACE)
    m_environmentMode = GL_MODULATE;

    // set environmental color
    m_color.set(1.0, 1.0, 1.0, 0.0);

    // set spherical mode
    m_useSphericalMapping = false;

    // mipmaps disabled
    m_useMipmaps = false;

    // default settings
    m_magFunctionMipmapsOFF = GL_LINEAR;
    m_minFunctionMipmapsOFF = GL_LINEAR;
    m_magFunctionMipmapsON  = GL_LINEAR;
    m_minFunctionMipmapsON  = GL_LINEAR_MIPMAP_LINEAR;

    // set the magnification function. (GL_NEAREST or GL_LINEAR)
    m_magFunction = m_magFunctionMipmapsOFF;

    // set the minifying function. (GL_NEAREST or GL_LINEAR)
    m_minFunction = m_minFunctionMipmapsOFF;
}


//==============================================================================
/*!
    This method enables texturing and set this texture as the current texture.

    \param  a_options  Rendering options.
*/
//==============================================================================
void cTextureCubeMap::renderInitialize(cRenderOptions& a_options)
{
#ifdef C_USE_OPENGL

    // check if texture is enabled
    if (!m_enabled) { return; }

    // check if materials should be rendered
    if (!a_options.m_render_textures) { return; }

    // Check skybox image textures
    for (int iI = 0 ; iI < 6 ; iI++){
        if (m_images[iI]->isInitialized() == 0){
            return;
        }

    }

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

    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

    // enable texturing
    glEnable(GL_TEXTURE_CUBE_MAP);

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
        glBindTexture(GL_TEXTURE_CUBE_MAP, m_textureID);
    }

    // enable or disable spherical mapping
    if (m_useSphericalMapping)
    {
        glEnable(GL_TEXTURE_GEN_S);
        glEnable(GL_TEXTURE_GEN_T);
        glEnable(GL_TEXTURE_GEN_R);
        glTexGeni(GL_S, GL_TEXTURE_GEN_MODE, GL_SPHERE_MAP);
        glTexGeni(GL_T, GL_TEXTURE_GEN_MODE, GL_SPHERE_MAP);
        glTexGeni(GL_R, GL_TEXTURE_GEN_MODE, GL_SPHERE_MAP);
    }
    else
    {
        glDisable(GL_TEXTURE_GEN_S);
        glDisable(GL_TEXTURE_GEN_T);
        glDisable(GL_TEXTURE_GEN_R);
    }

    // Sets the wrap parameter for texture coordinate s to either
    // GL_CLAMP or GL_REPEAT.
    glTexParameteri(GL_TEXTURE_CUBE_MAP ,GL_TEXTURE_WRAP_S, m_wrapModeS);
    glTexParameteri(GL_TEXTURE_CUBE_MAP ,GL_TEXTURE_WRAP_T, m_wrapModeT);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, m_wrapModeR);

    // Set the texture magnification function to either GL_NEAREST or GL_LINEAR.
    glTexParameteri(GL_TEXTURE_CUBE_MAP ,GL_TEXTURE_MAG_FILTER, m_magFunction);

    // Set the texture minifying function to either GL_NEAREST or GL_LINEAR.
    glTexParameteri(GL_TEXTURE_CUBE_MAP ,GL_TEXTURE_MIN_FILTER, m_minFunction);

    // set the environment mode (GL_MODULATE, GL_DECAL, GL_BLEND, GL_REPLACE)
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, m_environmentMode);

    // set the environmental color
    glTexEnvfv(GL_TEXTURE_ENV, GL_TEXTURE_ENV_COLOR, &m_color.getData()[0]);

    // set default vertex color which combined with the texture (white).
    glColor4f(1.0, 1.0, 1.0, 1.0);

#endif
}


//==============================================================================
/*!
    This method updates the texture from memory data to GPU.

    \param  a_options  Rendering options.
*/
//==============================================================================
void cTextureCubeMap::update(cRenderOptions& a_options)
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
        glBindTexture(GL_TEXTURE_CUBE_MAP, m_textureID);

        glTexParameteri(GL_TEXTURE_CUBE_MAP ,GL_TEXTURE_WRAP_S, m_wrapModeS);
        glTexParameteri(GL_TEXTURE_CUBE_MAP ,GL_TEXTURE_WRAP_T, m_wrapModeT);
        glTexParameteri(GL_TEXTURE_CUBE_MAP ,GL_TEXTURE_WRAP_R, m_wrapModeR);
        glTexParameteri(GL_TEXTURE_CUBE_MAP ,GL_TEXTURE_MAG_FILTER, m_magFunction);
        glTexParameteri(GL_TEXTURE_CUBE_MAP ,GL_TEXTURE_MIN_FILTER, m_minFunction);

        glPixelStorei(GL_UNPACK_ALIGNMENT,   1);
        glPixelStorei(GL_UNPACK_ROW_LENGTH,  0);
        glPixelStorei(GL_UNPACK_SKIP_ROWS,   0);
        glPixelStorei(GL_UNPACK_SKIP_PIXELS, 0);

        for (int i = 0 ; i < 6 ; i++){
            glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i,
                         0,
                         GL_RGBA,
                         m_images[i]->getWidth(),
                         m_images[i]->getHeight(),
                         0,
                         m_images[i]->getFormat(),
                         m_images[i]->getType(),
                         m_images[i]->getData()
                         );
        }
    }
    else
    {
        glBindTexture(GL_TEXTURE_CUBE_MAP, m_textureID);

        glTexParameteri(GL_TEXTURE_CUBE_MAP ,GL_TEXTURE_WRAP_S, m_wrapModeS);
        glTexParameteri(GL_TEXTURE_CUBE_MAP ,GL_TEXTURE_WRAP_T, m_wrapModeT);
        glTexParameteri(GL_TEXTURE_CUBE_MAP ,GL_TEXTURE_WRAP_T, m_wrapModeR);
        glTexParameteri(GL_TEXTURE_CUBE_MAP ,GL_TEXTURE_MAG_FILTER, m_magFunction);
        glTexParameteri(GL_TEXTURE_CUBE_MAP ,GL_TEXTURE_MIN_FILTER, m_minFunction);

        glPixelStorei(GL_UNPACK_ALIGNMENT,   1);
        glPixelStorei(GL_UNPACK_ROW_LENGTH,  0);
        glPixelStorei(GL_UNPACK_SKIP_ROWS,   0);
        glPixelStorei(GL_UNPACK_SKIP_PIXELS, 0);

        for (int i = 0 ; i < 6 ; i++){
            glTexSubImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i,
                            0,
                            0,
                            0,
                            (GLsizei)m_images[i]->getWidth(),
                            (GLsizei)m_images[i]->getHeight(),
                            (GLsizei)m_images[i]->getFormat(),
                            m_images[i]->getType(),
                            m_images[i]->getData());
        }
    }

    if (m_useMipmaps)
    {
        glEnable(GL_TEXTURE_CUBE_MAP);
        glGenerateMipmap(GL_TEXTURE_2D);
    }

#endif
}


//==============================================================================
/*!
    This method disable texture rendering  once triangles have been rendered.

    \param  a_options  Rendering options.
*/
//==============================================================================
void cTextureCubeMap::renderFinalize(cRenderOptions& a_options)
{
#ifdef C_USE_OPENGL
    if (m_enabled)
    {
        glActiveTexture(m_textureUnit);
        glDisable(GL_TEXTURE_CUBE_MAP);
    }
#endif
}


//==============================================================================
/*!
    This method Ssts the wrap parameter for texture coordinate __t__ to either
    GL_CLAMP or GL_REPEAT. \n\n

    GL_CLAMP causes __t__ coordinates to be clamped to the range [0,1] and is
    useful for preventing wrapping artifacts when mapping a single image onto
    an object. \n\n

    GL_REPEAT causes the integer part of the __t__ coordinate to be ignored;
    OpenGL uses only the fractional part, thereby creating a repeating pattern.
    Border texture elements are accessed only if wrapping is set to GL_CLAMP.

    \param  a_wrapModeT  value shall be either GL_REPEAT or GL_CLAMP.
*/
//==============================================================================
void cTextureCubeMap::setWrapModeT(const GLint& a_wrapModeT)
{
    m_wrapModeT = a_wrapModeT;
}



//==============================================================================
/*!
    This method Ssts the wrap parameter for texture coordinate __r__ to either
    GL_CLAMP or GL_REPEAT. \n\n

    GL_CLAMP causes __r__ coordinates to be clamped to the range [0,1] and is
    useful for preventing wrapping artifacts when mapping a single image onto
    an object. \n\n

    GL_REPEAT causes the integer part of the __r__ coordinate to be ignored;
    OpenGL uses only the fractional part, thereby creating a repeating pattern.
    Border texture elements are accessed only if wrapping is set to GL_CLAMP.

    \param  a_wrapModeR  value shall be either GL_REPEAT or GL_CLAMP.
*/
//==============================================================================
void cTextureCubeMap::setWrapModeR(const GLint& a_wrapModeR)
{
    m_wrapModeR = a_wrapModeR;
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
