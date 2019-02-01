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
#include "materials/CNormalMap.h"
using namespace std;
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    Constructor of cNormalMap.
*/
//==============================================================================
cNormalMap::cNormalMap()
{
    // set default texture unit
    m_textureUnit = GL_TEXTURE2;
}


//==============================================================================
/*!
    Destructor of cNormalMap.
*/
//==============================================================================
cNormalMap::~cNormalMap()
{
}


//==============================================================================
/*!
    This method creates a copy of itself.

    \return Pointer to new object.
*/
//==============================================================================
cNormalMapPtr cNormalMap::copy()
{
    // create new instance
    cNormalMapPtr obj = cNormalMap::create();

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
    This method creates a normal map from a image texture. The image gradient
    data is computed using the gray-scale level of each pixel.

    \param  a_texture  Input image.
*/
//==============================================================================
void cNormalMap::createMap(cTexture1dPtr a_texture)
{
    // sanity check
    if (a_texture != nullptr)
    {
        createMap(a_texture->m_image);
    }
}


//==============================================================================
/*!
    This method creates a normal map from a 2D image texture. The image gradient
    is computed using the gray-scale level of each pixel.

    \param  a_image  Input image.
*/
//==============================================================================
void cNormalMap::createMap(cImagePtr a_image)
{
    // sanity check
    if (m_image == nullptr)
    {
        m_image = cImage::create();
    }

    // compute gradient
    int w = a_image->getWidth();
    int h = a_image->getHeight();

    // allocate image size
    m_image->allocate(w, h, GL_RGB);

    for (int u=0; u<w; u++)
    {
        for (int v=0; v<h; v++)
        {
            cColorb colorU0V, colorU1V, colorUV0, colorUV1;

            // get color pixels
            a_image->getPixelColor(u, cClamp(v-1, 0, h-1), colorUV0);
            a_image->getPixelColor(u, cClamp(v+1, 0, h-1), colorUV1);
            a_image->getPixelColor(cClamp(u-1, 0, w-1), v, colorU0V);
            a_image->getPixelColor(cClamp(u+1, 0, w-1), v, colorU1V);

            // compute height from luminance value
            double levelUV0 = (double)(colorUV0.getLuminance());
            double levelUV1 = (double)(colorUV1.getLuminance());
            double levelU0V = (double)(colorU0V.getLuminance());
            double levelU1V = (double)(colorU1V.getLuminance());

            // compute normalized gradient
            double SCALE = 1.0/255.0;
            double deltaU =-SCALE * (levelU1V - levelU0V);
            double deltaV = SCALE * (levelUV1 - levelUV0);
            double deltaH = 1-cSqr(deltaU)-cSqr(deltaV);

            // compute length of vector
            double l = sqrt(deltaU*deltaU + deltaV*deltaV + deltaH*deltaH);

            // normalize, scale, and offset
            deltaU = 128 + 127 * (deltaU / l);
            deltaV = 128 + 127 * (deltaV / l);
            deltaH = 128 + 127 * (deltaH / l);

            // set gradient value at location (u,v)
            cColorb gradient;
            gradient.set((GLubyte)deltaU, (GLubyte)deltaV, (GLubyte)deltaH, 0xff);
            m_image->setPixelColor(u, v, gradient);
        }
    }
}


//==============================================================================
/*!
    This method flips normals along the U and/or V axis.

    \param  a_flipU  If __true__, normal texture is flipped along U axis.
    \param  a_flipV  If __true__, normal texture is flipped along V axis.
*/
//==============================================================================
void cNormalMap::flip(const bool a_flipU, const bool a_flipV)
{
    // sanity check
    if (m_image == nullptr) { return; }

    // flip normals
    int w = m_image->getWidth();
    for (int u=0; u<w; u++)
    {
        for (int v=0; v<w; v++)
        {
            cColorb color;
            m_image->getPixelColor(u, v, color);
            if (a_flipU)
            {
                color.setR(0xff - color.getR());
            }
            if (a_flipV)
            {
                color.setG(0xff - color.getG());
            }
        }
    }
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
