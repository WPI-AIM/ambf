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

//---------------------------------------------------------------------------
#include "CViewport.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    Constructor of cViewport.

    \param  a_winHandle  Handle to the window display.
    \param  a_pixelFormat  If non-zero, this custom pixel format is used 
              to initialize the viewport.
    \param  a_enableActiveStereo  If __true__ active stereo is enabled if
              available.
*/
//===========================================================================
cViewport::cViewport(HWND a_winHandle, PIXELFORMATDESCRIPTOR* a_pixelFormat, bool a_enableActiveStereo)
{
    // viewport isnot yet initialized
    m_enabled = false;

    // update wincontrol
    m_winHandle = a_winHandle;

    // initialize an OpenGL context
    m_glDC = GetDC(a_winHandle);

    // If the user requested a specific pixel format, use that as our
    // requested format for initializing the display context
    if (a_pixelFormat != 0)
    {
        m_pixelFormat = *a_pixelFormat;
    }

    // Otherwise use a default format descriptor...
    else
    {
		// basic settings
		DWORD dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER | PFD_TYPE_RGBA;
        
		// enable / disable stereo
		if (a_enableActiveStereo)
        {
			dwFlags = dwFlags | PFD_STEREO;
        }

        PIXELFORMATDESCRIPTOR pfd =
        {
            sizeof(PIXELFORMATDESCRIPTOR),       // size of this pfd
            1,                                   // version number
            dwFlags,							 // properties of the pixel buffer
            32,                                  // 32-bit color depth
            0, 0, 0, 0, 0, 0,                    // color bits ignored
            0,                                   // no alpha buffer
            0,                                   // shift bit ignored
            0,                                   // no accumulation buffer
            0, 0, 0, 0,                          // accum bits ignored
            32,                                  // 32-bit z-buffer
            0,                                   // no stencil buffer
            0,                                   // no auxiliary buffer
            PFD_MAIN_PLANE,                      // main layer
            0,                                   // reserved
            0, 0, 0                              // layer masks ignored
        };

        m_pixelFormat = pfd;
    }

    if (m_winHandle == 0)
    {
        return;
    }

    // find pixel format supported by the device context. If error return false.
    int formatIndex = ChoosePixelFormat(m_glDC, &m_pixelFormat);
    if (formatIndex == 0)
    {
        return;
    }

    // sets the specified device context's pixel format. If error return false
    if (!SetPixelFormat(m_glDC, formatIndex, &m_pixelFormat))
    {
        return;
    }

    formatIndex = GetPixelFormat (m_glDC);
    DescribePixelFormat (m_glDC, formatIndex, sizeof(PIXELFORMATDESCRIPTOR), &m_pixelFormat);

    // create display context
    m_glContext = wglCreateContext(m_glDC);
    if (m_glContext == 0)
    {        
        return;
    }

    // viewport has been succesfully initialized
    m_enabled = true;
}


//===========================================================================
/*!
    Destructor of cViewport.
*/
//===========================================================================
cViewport::~cViewport()
{
    cleanup();
}


//===========================================================================
/*!
    Clean up the current rendering context.
*/
//===========================================================================
bool cViewport::cleanup()
{
    bool result = true;

    // delete display context
    if (ReleaseDC(m_winHandle, m_glDC) == 0)
    {
        result = false;
    }

    if (wglDeleteContext(m_glContext) == 0) 
    {
        result = false;
    }

    m_glContext = 0;
    m_glDC = 0;
    m_glReady = false;

    return (result);
}


//===========================================================================
/*!
    Call this method before rendering the scene.
*/
//===========================================================================
bool cViewport::preRender()
{
    if (m_enabled)
    {
        // Make sure the viewport is really ready for rendering
        if (m_glReady == 0)
        {
            return (false);
        }

        RECT sizeWin;
        if (GetWindowRect(m_winHandle, &sizeWin) == 0) 
        { 
            return (false); 
        }

        unsigned int m_width   = sizeWin.right - sizeWin.left;
        unsigned int m_height  = sizeWin.bottom - sizeWin.top;

        // activate display context
        if (!wglMakeCurrent(m_glDC, m_glContext))
        {
            return(false);
        }

        // return success
        return (true);
    }
    else
    {
        return (false);
    }
}


//===========================================================================
/*!
    Call this method after rendering the scene.
*/
//===========================================================================
bool cViewport::postRender()
{
    if (m_enabled)
    {
        // swap buffers
        SwapBuffers(m_glDC);

        // deactivate display context
        wglMakeCurrent(m_glDC, 0);

        // return success
        return (true);
    }
    else
    {
        return (false);
    }
}
