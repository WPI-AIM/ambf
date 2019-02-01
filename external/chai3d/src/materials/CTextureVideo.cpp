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
    \author    Sebastien Grange
    \version   3.2.0 $Rev: 1869 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "materials/CTextureVideo.h"
using namespace std;
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------


//==============================================================================
/*!
    Constructor of cTextureVideo.
*/
//==============================================================================
cTextureVideo::cTextureVideo() : cTexture2d()
{
    // create video
    m_video = cVideo::create();
}


//==============================================================================
/*!
    Destructor of cTextureVideo.
*/
//==============================================================================
cTextureVideo::~cTextureVideo()
{
    return;
}


//==============================================================================
/*!
    This method enables texturing and set this video texture as the current
    texture.

    \param  a_options  Rendering options.
*/
//==============================================================================
void cTextureVideo::renderInitialize(cRenderOptions& a_options)
{
    // update video frame and mark texture for update if necessary
    if (m_video->getCurrentFramePointer(*(m_image.get())))
    {
        markForUpdate();
    }

    // call base renderer
    cTexture2d::renderInitialize(a_options);
}


//==============================================================================
/*!
    This method loads a texture video file.

    \param  a_fileName  Filename.
*/
//==============================================================================
bool cTextureVideo::loadFromFile(const string& a_fileName)
{
    if (m_video->loadFromFile(a_fileName) == false)
    {
        return (false);
    }

    return (true);
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
