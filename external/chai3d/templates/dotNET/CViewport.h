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
    \version   3.2.0 $Rev: 1242 $
*/
//==============================================================================
#ifndef CViewportH
#define CViewportH
//---------------------------------------------------------------------------
#include <windows.h>
//---------------------------------------------------------------------------

//===========================================================================
/*!
      \file       CViewport.h
      \class      cViewport

      \brief
      cViewport describes a two-dimensional window for rendering an OpenGL scene.
      Basically this class encapsulates an OpenGL rendering context.  Creating
      a window is left to the application programmer, since that will depend
      on the development environment that you're using.  Once you have a window
      handle, use this class to bind it to an OpenGL context.

      Typically a cViewport is connected to a cCamera for rendering, and a cCamera
      is typically connected to a cWorld, where objects actually live.
      
*/
//===========================================================================
class cViewport
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------
    
public:

    //! Constructor of cViewport
    cViewport(HWND a_winHandle,
              PIXELFORMATDESCRIPTOR* a_pixelFormat = NULL,
              bool a_enableActiveStereo = false);

    //! Destructor of cViewport.
    ~cViewport();


    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

public:

    //! Get width of viewport.
    unsigned int getWidth() const { return (m_width); }

    //! Get height of viewport.
    unsigned int getHeight() const { return (m_height); }

    //! Returns the pixel format used by this viewport
    const PIXELFORMATDESCRIPTOR* getPixelFormat() { return (&m_pixelFormat); }

    //! Return a direct handle to the OpenGL viewing context
    HDC getGLDC() { return (m_glDC); }

    //! Clean up the current rendering context
    bool cleanup();

    //! Intitialized viewport for rendering.
    bool preRender();

    //! Finalize rendering with the viewport.
    bool postRender();


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS:
    //--------------------------------------------------------------------------

protected:

    //! If __true__ then viewport has been initialized.
    bool m_enabled;

    //! Viewport width.
    int m_width;

    //! Viewport height.
    int m_height;

    //! Handle to window display.
    HWND m_winHandle;

    //! OpenGL display context.
    HGLRC m_glContext;

    //! display context.
    HDC m_glDC;

    //! OpenGL status.
    bool m_glReady;

    //! OpenGL context descriptor.
    PIXELFORMATDESCRIPTOR m_pixelFormat;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------

