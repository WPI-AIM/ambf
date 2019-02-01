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
#include "widgets/CBitmap.h"
//------------------------------------------------------------------------------
using namespace std;
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    Constructor of cBitmap.
*/
//==============================================================================
cBitmap::cBitmap()
{
    // initialize zoom factors
    m_zoomWidth  = 1.0;
    m_zoomHeight = 1.0;

    // enabling transparency
    setUseTransparency(true);

    // create empty texture map
    m_texture = cTexture2d::create();
}


//==============================================================================
/*!
    Destructor of cBitmap.
*/
//==============================================================================
cBitmap::~cBitmap()
{
}


//==============================================================================
/*!
    This method sets the zoom factors for the horizontal and vertical directions.

    \param  a_zoomWidth   Zoom factor along the horizontal direction.
    \param  a_zoomHeight  Zoom factor along the vertical direction.
*/
//==============================================================================
void cBitmap::setZoom(const float a_zoomWidth, 
                      const float a_zoomHeight)
{
    // update zoom factors
    m_zoomWidth = a_zoomWidth;
    m_zoomHeight = a_zoomHeight;

    // update panel mesh
    updateBitmapMesh();
}


//==============================================================================
/*!
    This method updates the mesh model of this bitmap widget.
*/
//==============================================================================
void cBitmap::updateBitmapMesh()
{
    if (m_texture == nullptr)
    {
        m_width = 0.0;
        m_height = 0.0;
    }
    else
    {
        m_width = m_zoomWidth * m_texture->m_image->getWidth();
        m_height = m_zoomHeight * m_texture->m_image->getHeight();
    }

    set(m_width, m_height, 0, 0, 0, 0);
}


//==============================================================================
/*!
    This method loads a texture image. Depending of the size of the image a 1D 
    or 2D texture map is created.

    \param  a_filename  Filename.
*/
//==============================================================================
bool cBitmap::loadFromFile(string a_filename)
{
    // create image if not yet allocated
    cImagePtr image = cImage::create();
    
    // load file
    bool success = image->loadFromFile(a_filename);
    if (!success) { return (false); }

    // retrieve dimension and decide if a 1D or 2D should be created
    unsigned int w = image->getWidth();
    unsigned int h = image->getHeight();

    // create 1D or 2D texture depending of image size
    if ((w == 1) || (h == 1))
    {
        m_texture = cTexture1d::create();
    }
    else
    {
        m_texture = cTexture2d::create();
    }

    // copy image data
    m_texture->m_image->allocate(image->getWidth(), image->getHeight(), image->getFormat());
    image->copyTo(m_texture->m_image);

    // enable texture
    setUseTexture(true);

    // enable vertex colors
    setUseVertexColors(false);
    setUseMaterial(false);

    // update bitmap mesh
    updateBitmapMesh();

    // success
    return (true);
}


//==============================================================================
/*!
    This method loads a texture image as background. Depending of the size of 
    the image a 1D or 2D texture map is created.

    \param  a_image  Input image.
*/
//==============================================================================
bool cBitmap::loadFromImage(cImagePtr a_image)
{
    // retrieve dimension and decide if a 1D or 2D should be created
    unsigned int w = a_image->getWidth();
    unsigned int h = a_image->getHeight();

    // create 1D or 2D texture depending of image size
    if ((w == 1) || (h == 1))
    {
        m_texture = cTexture1d::create();
    }
    else
    {
        m_texture = cTexture2d::create();
    }

    // copy image data
    /*
    m_texture->m_image->allocate(a_image->getWidth(), a_image->getHeight(), a_image->getFormat());
    a_image->copyTo(m_texture->m_image);
    */

    m_texture->m_image = a_image;

    // enable texture
    setUseTexture(true);

    // enable vertex colors
    setUseVertexColors(false);
    setUseMaterial(false);

    // update bitmap mesh
    updateBitmapMesh();

    // success
    return (true);
}


//==============================================================================
/*!
    This method creates a copy of itself.

    \param  a_duplicateMaterialData   If __true__, material (if available) is duplicated, otherwise it is shared.
    \param  a_duplicateTextureData    If __true__, texture data (if available) is duplicated, otherwise it is shared.
    \param  a_duplicateMeshData       If __true__, mesh data (if available) is duplicated, otherwise it is shared.
    \param  a_buildCollisionDetector  If __true__, collision detector (if available) is duplicated, otherwise it is shared.

    \return Pointer to new object.
*/
//==============================================================================
cBitmap* cBitmap::copy(const bool a_duplicateMaterialData,
                       const bool a_duplicateTextureData, 
                       const bool a_duplicateMeshData,
                       const bool a_buildCollisionDetector)
{
    // create new instance
    cBitmap* obj = new cBitmap();

    // copy cBitmap properties
    copyBitmapProperties(obj, 
        a_duplicateMaterialData, 
        a_duplicateTextureData,
        a_duplicateMeshData,
        a_buildCollisionDetector);

    // return
    return (obj);
}


//==============================================================================
/*!
    This method copies all properties of this object to another.

    \param  a_obj                     Destination object where properties are copied to.
    \param  a_duplicateMaterialData   If __true__, material (if available) is duplicated, otherwise it is shared.
    \param  a_duplicateTextureData    If __true__, texture data (if available) is duplicated, otherwise it is shared.
    \param  a_duplicateMeshData       If __true__, mesh data (if available) is duplicated, otherwise it is shared.
    \param  a_buildCollisionDetector  If __true__, collision detector (if available) is duplicated, otherwise it is shared.
*/
//==============================================================================
void cBitmap::copyBitmapProperties(cBitmap* a_obj,
    const bool a_duplicateMaterialData,
    const bool a_duplicateTextureData, 
    const bool a_duplicateMeshData,
    const bool a_buildCollisionDetector)
{
    // copy properties of cPanel
    copyPanelProperties(a_obj, 
        a_duplicateMaterialData, 
        a_duplicateTextureData,
        a_duplicateMeshData,
        a_buildCollisionDetector);

    // copy properties cBitmap
    a_obj->setZoom((float)m_zoomWidth, (float)m_zoomHeight);
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
