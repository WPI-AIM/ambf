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
#include "display/CFrameBuffer.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    Constructor of cFrameBuffer.
*/
//==============================================================================
cFrameBuffer::cFrameBuffer()
{
    m_width             = -1;
    m_height            = -1;
    m_prevWidth         = -1;
    m_prevHeight        = -1;
    m_useImageBuffer    = false;
    m_useDepthBuffer    = false;
    m_camera            = NULL;
    m_fbo               = -1;
}


//==============================================================================
/*!
    Destructor of cFrameBuffer.
*/
//==============================================================================
cFrameBuffer::~cFrameBuffer()
{
    #ifdef C_USE_OPENGL

    if ((int)m_fbo >= 0 && glIsFramebuffer(m_fbo))
    {
        glDeleteFramebuffers(1, &m_fbo);
        m_fbo = -1;
    }

    #endif
}


//==============================================================================
/*!
    This method initializes the cFrameBuffer.

    \param  a_camera          Camera to be attached to framebuffer.
    \param  a_width           Width in pixels of framebuffer.
    \param  a_height          Height in pixels of framebuffer.
    \param  a_useImageBuffer  If __true__ then framebuffer shall store color information.
    \param  a_useDepthBuffer  If __true__ then framebuffer shall store depth information.
*/
//==============================================================================
void cFrameBuffer::setup(cCamera* a_camera,
                         const unsigned int a_width,
                         const unsigned int a_height,
                         const bool a_useImageBuffer,
                         const bool a_useDepthBuffer)
{
    // store settings
    m_useImageBuffer = a_useImageBuffer;
    m_useDepthBuffer = a_useDepthBuffer;
    m_camera = a_camera;

    // setup image buffer
    if (m_useImageBuffer)
    {
        m_imageBuffer = cTexture2d::create();
        m_imageBuffer->setTextureUnit(GL_TEXTURE2);
        m_imageBuffer->setWrapModeS(GL_CLAMP_TO_EDGE);
        m_imageBuffer->setWrapModeT(GL_CLAMP_TO_EDGE);
        m_imageBuffer->setMagFunction(GL_LINEAR);
        m_imageBuffer->setMinFunction(GL_LINEAR);
    }

    // setup depth buffer
    if (m_useDepthBuffer)
    {
        m_depthBuffer = cTexture2d::create();
        m_depthBuffer->setTextureUnit(GL_TEXTURE3);
        m_depthBuffer->setWrapModeS(GL_CLAMP_TO_EDGE);
        m_depthBuffer->setWrapModeT(GL_CLAMP_TO_EDGE);
        m_depthBuffer->setMagFunction(GL_LINEAR);
        m_depthBuffer->setMinFunction(GL_LINEAR);
    }

    // setup resolution
    setSize(a_width, a_height);
}


//==============================================================================
/*!
    This method sets the size of the framebuffer by defining the width and 
    height in pixel.

    \param  a_width   Width of frame buffer in pixels.
    \param  a_height  Height of frame buffer in pixels.
*/
//==============================================================================
void cFrameBuffer::setSize(const unsigned int a_width, const unsigned int a_height)
{
    // store dimensions in pixels
    m_width = a_width;
    m_height = a_height;

    // allocate image buffer
    if(m_useImageBuffer)
    {
        m_imageBuffer->m_image->allocate(m_width, m_height, GL_RGB, GL_UNSIGNED_BYTE);
    }

    // allocate depth buffer
    if(m_useDepthBuffer)
    {
        m_depthBuffer->m_image->allocate(m_width, m_height, GL_DEPTH_COMPONENT, GL_UNSIGNED_INT);
    }
}


//==============================================================================
/*!
    This method copies the OpenGL image buffer content to an image.

    \param  a_image           Destination image.
*/
//==============================================================================
void cFrameBuffer::copyImageBuffer(cImagePtr a_image)
{
#ifdef C_USE_OPENGL

    // check image structure
    if (a_image == nullptr) { return; }

    // check size
    if ((m_width  != a_image->getWidth()) ||
        (m_height != a_image->getHeight()))
    {

        a_image->allocate(m_width, m_height, GL_RGBA);
    }

    // bind texture
    glBindTexture(GL_TEXTURE_2D, m_imageBuffer->getTextureId());

    // settings
    glPixelStorei(GL_PACK_ALIGNMENT, 1);

    // copy pixel data if required
    glGetTexImage(GL_TEXTURE_2D,
                  0,
                  GL_RGBA,
                  GL_UNSIGNED_BYTE,
                  (GLvoid*)(a_image->getData())
                  );
#endif
}


//==============================================================================
/*!
    This method copies the OpenGL depth buffer content to an image.

    \param  a_image           Destination image.
*/
//==============================================================================
void cFrameBuffer::copyDepthBuffer(cImagePtr a_image)
{
#ifdef C_USE_OPENGL

    // check image structure
    if (a_image == nullptr) { return; }

    // check size
    if ((m_width  != a_image->getWidth()) ||
        (m_height != a_image->getHeight()))
    {

        a_image->allocate(m_width, m_height, GL_RGBA);
    }

    // bind texture
    glBindTexture(GL_TEXTURE_2D, m_depthBuffer->getTextureId());

    // settings
    glPixelStorei(GL_PACK_ALIGNMENT, 1);

    // copy pixel data if required
    glGetTexImage(GL_TEXTURE_2D,
                  0,
                  GL_DEPTH_COMPONENT,
                  GL_UNSIGNED_INT,
                  (GLvoid*)(a_image->getData())
                  );

#endif
}


//==============================================================================
/*!
    This method initializes the framebuffer so that rendering of the scene can 
    occur. This method is for advanced users only; in practice you should simply
    call method \ref renderView() to render the framebuffer.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cFrameBuffer::renderInitialize()
{
#ifdef C_USE_OPENGL

    // check for necessary OpenGL extensions
#ifdef GLEW_VERSION
    if(!(GLEW_ARB_depth_texture && GLEW_ARB_shadow && GLEW_ARB_framebuffer_object))
    {
        return (false);
    }
#endif

    /////////////////////////////////////////////////////////////////////////////////////
    // SIZE TESTING
    /////////////////////////////////////////////////////////////////////////////////////

    // verify size of frame buffer
    if (!((m_width > 0) && (m_height > 0) && (m_useImageBuffer || m_useDepthBuffer)))
    { 
        return (false);
    }


    /////////////////////////////////////////////////////////////////////////////////////
    // SETUP FRAMEBUFFER TO BE TEXTURE OBJECTS
    /////////////////////////////////////////////////////////////////////////////////////

    // as with the other objects in OpenGL (texture object, pixel buffer objects 
    // and vertex buffer object) before you can use a FBO you have to create a 
    // valid handle to it:

    if (glIsFramebuffer(m_fbo) == false)
    {
        glGenFramebuffers(1, &m_fbo);
    }

    // to perform any operations on a FBO you need to bind it, much like you 
    // would a VBO or texture, so that the operations can be performed on it, 
    // this is done via the following code
    glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);

    // reserve space for a texture
    glPushAttrib(GL_ALL_ATTRIB_BITS);


    /////////////////////////////////////////////////////////////////////////////////////
    // SETUP IMAGE BUFFER
    /////////////////////////////////////////////////////////////////////////////////////
    if (m_useImageBuffer)
    {
        // enable texture
        glActiveTexture(m_imageBuffer->getTextureUnit());

        // update parameters
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, m_imageBuffer->getMinFunction());
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, m_imageBuffer->getMagFunction());
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, m_imageBuffer->getWrapModeS());
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, m_imageBuffer->getWrapModeT());

        // create texture
        bool flagNew = false;
        if (m_imageBuffer->getTextureId() == 0)
        {
            GLuint textureId;
            glGenTextures(1, &textureId);
            m_imageBuffer->setTextureId(textureId);
            flagNew = true;
        }

        // bind texture
        glBindTexture(GL_TEXTURE_2D, m_imageBuffer->getTextureId());

        // setup properties
        if ((m_width != m_prevWidth) || (m_height != m_prevHeight) || (flagNew))
        {
            glTexImage2D(GL_TEXTURE_2D,
                0,
                GL_RGBA,
                m_imageBuffer->m_image->getWidth(),
                m_imageBuffer->m_image->getHeight(),
                0,
                m_imageBuffer->m_image->getFormat(),
                m_imageBuffer->m_image->getType(),
                0
                );
        }

        // connect framebuffer to texture
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_imageBuffer->getTextureId(), 0);

        GLuint status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
        if (status != GL_FRAMEBUFFER_COMPLETE) 
        {
            printf("Frame Buffer Not Complete\n");
            glPopAttrib();
            return (false);
        }
    }


    /////////////////////////////////////////////////////////////////////////////////////
    // SETUP DEPTH BUFFER
    /////////////////////////////////////////////////////////////////////////////////////
    if (m_useDepthBuffer)
    {
        // enable texture
        glActiveTexture(m_depthBuffer->getTextureUnit());

        // no drawing
        glDrawBuffer(GL_NONE);
        glReadBuffer(GL_NONE);
        
        // update parameters
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, m_depthBuffer->getMinFunction());
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, m_depthBuffer->getMagFunction());
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, m_depthBuffer->getWrapModeS());
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, m_depthBuffer->getWrapModeT());

        // create texture
        bool flagNew = false;
        if (m_depthBuffer->getTextureId() == 0)
        {
            GLuint textureId;
            glGenTextures(1, &textureId);
            m_depthBuffer->setTextureId(textureId);
            flagNew = true;
        }

        // bind texture
        glBindTexture(GL_TEXTURE_2D, m_depthBuffer->getTextureId());

        // setup properties
        if ((m_width != m_prevWidth) || (m_height != m_prevHeight) || (flagNew))
        {
            glTexImage2D(GL_TEXTURE_2D,
                0,
                GL_DEPTH_COMPONENT,
                m_depthBuffer->m_image->getWidth(),
                m_depthBuffer->m_image->getHeight(),
                0,
                m_depthBuffer->m_image->getFormat(),
                m_depthBuffer->m_image->getType(),
                0
                );
        }

        // connect framebuffer to texture
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, m_depthBuffer->getTextureId(), 0);
        
        GLuint status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
        if (status != GL_FRAMEBUFFER_COMPLETE) 
        {
            printf("Depth Buffer Not Complete\n");
            glPopAttrib();
            return (false);
        }
    }

    // update information
    m_prevWidth = m_width;
    m_prevHeight = m_height;

    return (true);
#else
    return (false);
#endif
}


//==============================================================================
/*!
    This method finalizes the rendering of the framebuffer. This method is for
    advanced users only; in practice you should simply call method 
    \ref renderView() to render the framebuffer.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cFrameBuffer::renderFinalize()
{

#ifdef C_USE_OPENGL
    // restore OpenGL settings
    glDisable(GL_ALPHA_TEST);
    glDisable(GL_POLYGON_OFFSET_FILL);
    glColorMask(true, true, true, true);
    glCullFace(GL_BACK);
    glShadeModel(GL_SMOOTH);
    glPopAttrib();
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
#endif

    return (true);
}


//==============================================================================
/*!
    This method render the scene to the framebuffer.

    \param  a_eyeMode         When using stereo mode C_STEREO_PASSIVE_DUAL_DISPLAY,
                              this argument specifies which eye view to render.
*/
//==============================================================================
void cFrameBuffer::renderView(const cEyeMode a_eyeMode)
{
    // initialization
    if (renderInitialize())
    {
        if (m_camera != NULL)
        {
            // render scene
            m_camera->renderView(m_width, m_height, C_STEREO_LEFT_EYE, false);
        }

        // finalization;
        renderFinalize();
    }
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
