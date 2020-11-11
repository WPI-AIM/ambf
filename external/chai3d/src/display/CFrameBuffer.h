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
#ifndef CFrameBufferH
#define CFrameBufferH
//------------------------------------------------------------------------------
#include "graphics/CImage.h"
#include "graphics/CColor.h"
#include "materials/CTexture2d.h"
#include "world/CWorld.h"
//------------------------------------------------------------------------------
#include <string>
#include <stdio.h>
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*! 
    \file       CFrameBuffer.h

    \brief
    Implementation of a framebuffer.
*/
//==============================================================================

//------------------------------------------------------------------------------
class cFrameBuffer;
typedef std::shared_ptr<cFrameBuffer> cFrameBufferPtr;
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \class      cFrameBuffer
    \ingroup    display
    
    \brief
    This class implements an OpenGL framebuffer.

    \details
    This class implements an OpenGL framebuffer.
*/
//==============================================================================
class cFrameBuffer
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cFrameBuffer.
    cFrameBuffer();

    //! Destructor of cFrameBuffer.
    virtual ~cFrameBuffer();

    //! Shared cFrameBuffer allocator.
    static cFrameBufferPtr create() { return (std::make_shared<cFrameBuffer>()); }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS
    //--------------------------------------------------------------------------

public:

    //! This method initializes the framebuffer.
    void setup(cCamera* a_camera = NULL,
               const unsigned int a_width = 0,
               const unsigned int a_height = 0,
               const bool a_enableImageBuffer = true,
               const bool a_enableDepthBuffer = true,
               const GLenum a_imageBufferInternalStorageType=GL_RGBA,
               const GLenum a_depthBufferInternalStorageType=GL_DEPTH_COMPONENT);

    //! This method renders the framebuffer view.
    void renderView(const cEyeMode a_eyeMode = C_STEREO_LEFT_EYE);

    //! This method copies the framebuffer content to an image.
    void copyImageBuffer(cImagePtr a_image, int a_type=GL_UNSIGNED_BYTE);

    //! This method copies the depth buffer content to an image.
    void copyDepthBuffer(cImagePtr a_image);


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - CAMERA
    //--------------------------------------------------------------------------

public:

    //! This method attaches a camera to the framebuffer.
    void setCamera(cCamera* a_camera) { m_camera = a_camera; }

    //! This method returns a pointer to the camera attached to the framebuffer.
    cCamera* getCamera() { return (m_camera); }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - RESOLUTION
    //--------------------------------------------------------------------------

public:


    //! This method returns the width in pixels of the framebuffer.
    inline unsigned int getWidth() const { return (m_width); }

    //! This method returns the height in pixels of the framebuffer.
    inline unsigned int getHeight() const { return (m_height); }

    //! Returns the internal format of Image Buffer's Texture Image
    inline GLenum getImageBufferInternalFormat(){return m_imageInternalFormat;}

    //! Returns the internal format of Depth Buffer's Texture Image
    inline GLenum getDepthBufferInternalFormat(){return m_depthInternalFormat;}

    //! This method sets the resolution of the framebuffer by defining its width and height in pixels.
    void setSize(const unsigned int a_width, const unsigned int a_height);


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - EXPERTS ONLY
    //--------------------------------------------------------------------------

public:

    //! This method initializes rendering to the framebuffer.
    bool renderInitialize();

    //! This method finalizes the rendering to the framebuffer.
    bool renderFinalize();


    //--------------------------------------------------------------------------
    // PUBLIC MEMBERS - BUFFERS
    //--------------------------------------------------------------------------

public:

    //! Image buffer.
    cTexture2dPtr m_imageBuffer;

    //! Depth buffer.
    cTexture2dPtr m_depthBuffer;


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS:
    //--------------------------------------------------------------------------

protected:

    //! If __true__ then image buffer is enabled, __false__ otherwise.
    bool m_useImageBuffer;

    //! If __true__ then image buffer is enabled, __false__ otherwise.
    bool m_useDepthBuffer;

    //! Width in pixels of the frame buffer.
    unsigned int m_width;

    //! Height in pixels of the frame buffer.
    unsigned int m_height;

    //! Last width value that was used when allocating texture size.
    unsigned int m_prevWidth;

    //! Last height value that was used when allocating texture size.
    unsigned int m_prevHeight;

    //! Camera associated with this framebuffer.
    cCamera* m_camera;

    //! OpenGL frame buffer object.
    GLuint m_fbo;

    //! Storage Type of Image Buffer. E.g. GL_RGBA, GL_RGBA12, GL_RGBA16 ...
    GLenum m_imageInternalFormat;

    //! Storage Type of Depth Buffer. E.g. GL_DEPTH_COMPONENT, GL_DEPTH)COMPONENT24 ...
    GLenum m_depthInternalFormat;
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------

