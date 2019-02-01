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
#include "widgets/CBackground.h"
using namespace std;
//------------------------------------------------------------------------------
#include "display/CCamera.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    Constructor of cBackground.
*/
//==============================================================================
cBackground::cBackground()
{
    // create a panel without rounded corners (radius = 0). The default width 
    // and height values are updated in the render() method to span the full 
    // window display.

    set(200,   // default width
        100,   // default height
          0,
          0,
          0,
          0);

    m_fixedAspectRatio = true;

    // use display lists
    setUseDisplayList(false);
}


//==============================================================================
/*!
    This method renders the background object using OpenGL.

    \param  a_options  Rendering options.
*/
//==============================================================================
void cBackground::render(cRenderOptions& a_options)
{
    // update the width and height of background in case window size has changed
    int w = a_options.m_camera->getDisplayWidth();
    int h = a_options.m_camera->getDisplayHeight();
    update(0, 0, w, h);  

    // render background
    cMesh::render(a_options);
}


//==============================================================================
/*!
    This method updates the dimensions of the background based on the viewport
    dimensions passed as argument.

    \param  a_bottomLeftX  X coordinate of bottom left point.
    \param  a_bottomLeftY  Y coordinate of bottom left point.
    \param  a_topRightX    X coordinate of top right point.
    \param  a_topRightY    Y coordinate of top right point.
*/
//==============================================================================
void cBackground::update(const unsigned int a_bottomLeftX,
                         const unsigned int a_bottomLeftY,
                         const unsigned int a_topRightX,
                         const unsigned int a_topRightY)
{
    // sanity check
    int numVertices = getNumVertices();
    if (numVertices != 4)
    {
        return;
    }

    // vertex indices
    unsigned int vertexTL = 3;
    unsigned int vertexTR = 2;
    unsigned int vertexBL = 0;
    unsigned int vertexBR = 1;

    // update position of vertices
    m_vertices->setLocalPos(vertexTL, a_bottomLeftX, a_topRightY,    0.0);
    m_vertices->setLocalPos(vertexTR, a_topRightX,   a_topRightY,    0.0);
    m_vertices->setLocalPos(vertexBL, a_bottomLeftX, a_bottomLeftY,  0.0);
    m_vertices->setLocalPos(vertexBR, a_topRightX,   a_bottomLeftY,  0.0);

    // update texture coordinates
    if (dynamic_pointer_cast<cTexture2d>(m_texture))
    {
        // 2D texture
        if (m_fixedAspectRatio)
        {
            // temp variables
            double ratioImage  = 1.0;
            double ratioWindow = 1.0;
            double wW, hW;
            double wI = 1.0;
            double hI = 1.0;

            // compute window ratio 
            wW = (double)(a_topRightX - a_bottomLeftX);
            hW = (double)(a_topRightY - a_bottomLeftY);

            if (hW > 0)
            {
                ratioWindow = wW/hW;
            }

            // compute image ratio
            cImagePtr image = dynamic_pointer_cast<cTexture2d>(m_texture)->m_image;
            if (image != nullptr)
            {
                wI = (double)(image->getWidth());
                hI = (double)(image->getHeight());

                if (hI > 0)
                {
                    ratioImage = wI/hI;
                }
            }

            // compute texture coordinates so that image ratio is maintained
            if (ratioWindow > ratioImage)
            {
                double h = (hW / wW) * wI;
                double t = 0.5 * (hI - h) / hI;

                m_vertices->setTexCoord(vertexTL, 0.0, 1.0-t, 0.0);
                m_vertices->setTexCoord(vertexTR, 1.0, 1.0-t, 0.0);
                m_vertices->setTexCoord(vertexBL, 0.0, t, 0.0);
                m_vertices->setTexCoord(vertexBR, 1.0, t, 0.0);
            }
            else
            {
                double w = (wW / hW) * hI;
                double t = 0.5 * (wI - w) / wI;

                m_vertices->setTexCoord(vertexTL, t, 1.0, 0.0);
                m_vertices->setTexCoord(vertexTR, 1.0-t, 1.0, 0.0);
                m_vertices->setTexCoord(vertexBL, t, 0.0, 0.0);
                m_vertices->setTexCoord(vertexBR, 1.0-t, 0.0, 0.0);
            }
        }
        else
        {
            m_vertices->setTexCoord(vertexTL, 0.0, 1.0, 0.0);
            m_vertices->setTexCoord(vertexTR, 1.0, 1.0, 0.0);
            m_vertices->setTexCoord(vertexBL, 0.0, 0.0, 0.0);
            m_vertices->setTexCoord(vertexBR, 1.0, 0.0, 0.0);
        }
    }
    
    else if (dynamic_pointer_cast<cTexture1d>(m_texture))
    {
        // horizontal 1D texture
        if (m_texture->m_image->getHeight() == 1)
        {
            m_vertices->setTexCoord(vertexTL, 0.0, 0.0, 0.0);
            m_vertices->setTexCoord(vertexTR, 0.0, 1.0, 0.0);
            m_vertices->setTexCoord(vertexBL, 0.0, 0.0, 0.0);
            m_vertices->setTexCoord(vertexBR, 0.0, 1.0, 0.0);
        }

        // vertical 1D texture
        else
        {
            m_vertices->setTexCoord(vertexTL, 1.0, 0.0, 0.0);
            m_vertices->setTexCoord(vertexTR, 1.0, 0.0, 0.0);
            m_vertices->setTexCoord(vertexBL, 0.0, 0.0, 0.0);
            m_vertices->setTexCoord(vertexBR, 0.0, 0.0, 0.0);
        }
    }

    // update size
    m_width = a_topRightX - a_bottomLeftX;
    m_height = a_topRightY - a_bottomLeftY;
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
cBackground* cBackground::copy(const bool a_duplicateMaterialData,
                       const bool a_duplicateTextureData, 
                       const bool a_duplicateMeshData,
                       const bool a_buildCollisionDetector)
{
    // create new instance
    cBackground* obj = new cBackground();

    // copy cBitmap properties
    copyBackgroundProperties(obj, 
        a_duplicateMaterialData, 
        a_duplicateTextureData,
        a_duplicateMeshData,
        a_buildCollisionDetector);

    // return
    return (obj);
}


//==============================================================================
/*!
    This method copies properties from this object to another passed as argument.

    \param  a_obj                     Destination object where properties are copied to.
    \param  a_duplicateMaterialData   If __true__, material (if available) is duplicated, otherwise it is shared.
    \param  a_duplicateTextureData    If __true__, texture data (if available) is duplicated, otherwise it is shared.
    \param  a_duplicateMeshData       If __true__, mesh data (if available) is duplicated, otherwise it is shared.
    \param  a_buildCollisionDetector  If __true__, collision detector (if available) is duplicated, otherwise it is shared.
*/
//==============================================================================
void cBackground::copyBackgroundProperties(cBackground* a_obj,
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

    // copy properties of cBackground
    a_obj->m_fixedAspectRatio = m_fixedAspectRatio;
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
