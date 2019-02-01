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
    \version   3.2.0 $Rev: 2178 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "widgets/CPanel.h"
//------------------------------------------------------------------------------
#include "graphics/CPrimitives.h"
//------------------------------------------------------------------------------
#include <vector>
//------------------------------------------------------------------------------
using namespace std;
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    Constructor of cPanel.
*/
//==============================================================================
cPanel::cPanel()
{
    // use vertex colors
    m_useVertexColors = true;

    // disable material properties
    m_useMaterialProperty = false;

    // set default colors for each corner
    m_panelColorTopLeft.setGrayLevel(0.40f);
    m_panelColorTopRight.setGrayLevel(0.40f);
    m_panelColorBottomLeft.setGrayLevel(0.40f);
    m_panelColorBottomRight.setGrayLevel(0.40f);

    // set default radius values
    m_panelRadiusTopLeft =       0.0;
    m_panelRadiusTopRight =      0.0;
    m_panelRadiusBottomLeft =    0.0;
    m_panelRadiusBottomRight =   0.0;

    // set default margin values
    m_marginTop    = 0.0;
    m_marginBottom = 0.0;
    m_marginLeft   = 0.0;
    m_marginRight  = 0.0;

    // number of line segments used to render each corner
    m_numPanelSegmentsPerCorner = 8;

    // enable panel for display
    m_showPanel = true;

    // use display lists
    setUseDisplayList(true);
}


//==============================================================================
/*!
    This method sets the width and height of this panel, including the radius
    at each corner.

    \param  a_width              Width of Panel.
    \param  a_height             Height of Panel.
    \param  a_radiusTopLeft      Radius of top left corner.
    \param  a_radiusTopRight     Radius of top right corner.
    \param  a_radiusBottomLeft   Radius of bottom left corner.
    \param  a_radiusBottomRight  Radius of bottom right corner.
*/
//==============================================================================
void cPanel::set(const double& a_width, 
    const double& a_height,
    const double& a_radiusTopLeft,
    const double& a_radiusTopRight,
    const double& a_radiusBottomLeft,
    const double& a_radiusBottomRight)
{
    // set new values for each corner
    m_panelRadiusTopLeft = fabs(a_radiusTopLeft);
    m_panelRadiusTopRight = fabs(a_radiusTopRight);
    m_panelRadiusBottomLeft = fabs(a_radiusBottomLeft);
    m_panelRadiusBottomRight = fabs(a_radiusBottomRight);

    // set new values for width and height. 
    double w = cMax(a_radiusTopLeft, a_radiusBottomLeft) + 
               cMax(a_radiusTopRight, a_radiusBottomRight);
    m_width = cMax(a_width, w);

    double h = cMax(a_radiusTopLeft, a_radiusTopRight) + 
               cMax(a_radiusBottomLeft, a_radiusBottomRight);
    m_height = cMax(a_height, h);
    
    // update model of panel
    updatePanelMesh();
}


//==============================================================================
/*!
    This method sets the width and height of the panel.

    \param  a_width   Width of panel.
    \param  a_height  Height of panel.
*/
//==============================================================================
void cPanel::setSize(const double& a_width, 
                     const double& a_height)
{
    // set new values for width and height.
    double w = cMax(m_panelRadiusTopLeft, m_panelRadiusBottomLeft) + 
               cMax(m_panelRadiusTopRight, m_panelRadiusBottomRight);
    m_width = cMax(a_width, w);

    double h = cMax(m_panelRadiusTopLeft, m_panelRadiusTopRight) + 
               cMax(m_panelRadiusBottomLeft, m_panelRadiusBottomRight);
    m_height = cMax(a_height, h);

    // update model of panel
    updatePanelMesh();
}


//==============================================================================
/*!
    This method sets the radius os each corner of the panel.

    \param  a_radiusTopLeft      Radius of top left corner.
    \param  a_radiusTopRight     Radius of top right corner.
    \param  a_radiusBottomLeft   Radius of bottom left corner.
    \param  a_radiusBottomRight  Radius of bottom right corner.
*/
//==============================================================================
void cPanel::setCornerRadius(const double& a_radiusTopLeft,
    const double& a_radiusTopRight,
    const double& a_radiusBottomLeft,
    const double& a_radiusBottomRight)
{
    // set new values
    m_panelRadiusTopLeft = fabs(a_radiusTopLeft);
    m_panelRadiusTopRight = fabs(a_radiusTopRight);
    m_panelRadiusBottomLeft = fabs(a_radiusBottomLeft);
    m_panelRadiusBottomRight = fabs(a_radiusBottomLeft);
    m_panelRadiusBottomRight = fabs(a_radiusBottomRight);

    // check dimension for width and height. adjust accordingly
    double w = cMax(m_panelRadiusTopLeft, m_panelRadiusBottomLeft) + 
               cMax(m_panelRadiusTopRight, m_panelRadiusBottomRight);
    m_width = cMax(m_width, w);

    double h = cMax(m_panelRadiusTopLeft, m_panelRadiusTopRight) + 
               cMax(m_panelRadiusBottomLeft, m_panelRadiusBottomRight);
    m_height = cMax(m_height, h);

    // update model of panel
    updatePanelMesh();
}


//==============================================================================
/*!
    This method sets the color of the panel.

    \param  a_panelColor  Color of panel.
*/
//==============================================================================
void cPanel::setColor(const cColorf& a_panelColor)
{
    // assign new color
    m_panelColorTopLeft     = a_panelColor;
    m_panelColorTopRight    = a_panelColor;
    m_panelColorBottomLeft  = a_panelColor;
    m_panelColorBottomRight = a_panelColor;

    // update model of panel
    updatePanelMesh();
}


//==============================================================================
/*!
    This method sets a color for each of the four corners of the panel.

    \param  a_panelColorTopLeft      Color of top left corner.
    \param  a_panelColorTopRight     Color of top right corner.
    \param  a_panelColorBottomLeft   Color of bottom left corner.
    \param  a_panelColorBottomRight  Color of bottom right corner.
*/
//==============================================================================
void cPanel::setCornerColors(const cColorf& a_panelColorTopLeft, 
    const cColorf& a_panelColorTopRight, 
    const cColorf& a_panelColorBottomLeft, 
    const cColorf& a_panelColorBottomRight)
{
    // assign new colors
    m_panelColorTopLeft     = a_panelColorTopLeft;
    m_panelColorTopRight    = a_panelColorTopRight;
    m_panelColorBottomLeft  = a_panelColorBottomLeft;
    m_panelColorBottomRight = a_panelColorBottomRight;

    // update model of panel
    updatePanelMesh();
}


//==============================================================================
/*!
    This method defines a vertical gradient colored panel. The gradient is 
    defined by the top and bottom colors.

    \param  a_topColor     Top color.
    \param  a_bottomColor  Bottom color.
*/
//==============================================================================
void cPanel::setPanelColorVerticalGradient(cColorf a_topColor, cColorf a_bottomColor)
{
    // assign new colors
    m_panelColorTopLeft     = a_topColor;
    m_panelColorTopRight    = a_topColor;
    m_panelColorBottomLeft  = a_bottomColor;
    m_panelColorBottomRight = a_bottomColor;

    // update model of panel
    updatePanelMesh();
}


//==============================================================================
/*!
    This method defines a horizontal gradient colored background. The gradient
    is defined by the left and right colors.

    \param  a_leftColor   Left color.
    \param  a_rightColor  Right color.
*/
//==============================================================================
void cPanel::setHorizontalLinearGradient(cColorf a_leftColor, cColorf a_rightColor)
{
    // assign new colors
    m_panelColorTopLeft     = a_leftColor;
    m_panelColorTopRight    = a_rightColor;
    m_panelColorBottomLeft  = a_leftColor;
    m_panelColorBottomRight = a_rightColor;

    // update model of panel
    updatePanelMesh();
}


//==============================================================================
/*!
    This method assigns a transparency level for this panel.

    \param  a_level            Level of transparency ranging from 0.0 to 1.0.
    \param  a_applyToVertices  If __true__, then apply changes to vertex colors.
    \param  a_applyToTextures  If __true__, then apply changes to texture.
    \param  a_affectChildren   If __true__, then children are updated too.
*/
//==============================================================================
void cPanel::setTransparencyLevel(const float a_level,
    const bool a_applyToVertices,
    const bool a_applyToTextures,
    const bool a_affectChildren)
{
    // if the transparency level is equal to 1.0, then do not apply transparency
    // otherwise enable it.
    if (a_level < 1.0)
    {
        setUseTransparency(true);
    }
    else
    {
        setUseTransparency(false);
    }

    // apply new value to material
    if (m_material != nullptr)
    {
        m_material->setTransparencyLevel(a_level);
    }

    // apply new value to texture
    if (m_texture != nullptr)
    {
        if (m_texture->m_image != nullptr)
        {
            unsigned char level = (unsigned char)(255.0 * a_level);
            m_texture->m_image->setTransparency(level);
        }
    }

    // assign transparency level to vertex colors
    m_panelColorTopLeft.setA(a_level);
    m_panelColorTopRight.setA(a_level);
    m_panelColorBottomLeft.setA(a_level);
    m_panelColorBottomRight.setA(a_level);

    // update panel.
    updatePanelMesh();

    // apply change to children
    if (a_affectChildren)
    {
        vector<cGenericObject*>::iterator it;
        for (it = m_children.begin(); it < m_children.end(); it++)
        {
            (*it)->setTransparencyLevel(a_level,
                                        a_applyToVertices,
                                        a_applyToTextures,
                                        true);
        }
    }
}

//==============================================================================
/*!
    This message assign values to all four margins of the widget, namely the
    __top__, __bottom__, __left__, and __right__ margins.

    Margins are interpreted by the widget render method when displaying images, 
    text, or data inside them. The rule is to always make sure that nothing 
    exceeds the margin limits.

    \param  a_marginTop  Top margin.
    \param  a_marginBottom  Bottom margin.
    \param  a_marginLeft  Left margin.
    \param  a_marginRight  Right margin.
*/
//==============================================================================
void cPanel::setMargins(const double a_marginTop,
    const double a_marginBottom,
    const double a_marginLeft,
    const double a_marginRight)
{
    m_width  = 0.0;
    m_height = 0.0;
    m_marginTop    = cMax(0.0, a_marginTop);
    m_marginBottom = cMax(0.0, a_marginBottom);
    m_marginLeft   = cMax(0.0, a_marginLeft);
    m_marginRight  = cMax(0.0, a_marginRight);
}


//==============================================================================
/*!
    This method assign a top margin value.

    \param  a_marginTop  Top margin.
*/
//==============================================================================
void cPanel::setMarginTop(const double a_marginTop)
{ 
    m_marginTop = cMax(0.0, a_marginTop); 
    setSize(m_width, m_height);
}


//==============================================================================
/*!
    This method assign a bottom margin value.

    \param  a_marginBottom  Bottom margin.
*/
//==============================================================================
void cPanel::setMarginBottom(const double a_marginBottom)
{ 
    m_marginBottom = cMax(0.0, a_marginBottom);
    setSize(m_width, m_height);
}


//==============================================================================
/*!
    This method assign a left margin value.

    \param  a_marginLeft  Left margin.
*/
//==============================================================================
void cPanel::setMarginLeft(const double a_marginLeft)
{ 
    m_marginLeft = cMax(0.0, a_marginLeft);
    setSize(m_width, m_height);
}


//==============================================================================
/*!
    This method assign a right margin value.

    \param  a_marginRight  Right margin.
*/
//==============================================================================
void cPanel::setMarginRight(const double a_marginRight)
{ 
    m_marginRight = cMax(0.0, a_marginRight);
    setSize(m_width, m_height);
}


//==============================================================================
/*!
    This method updates the mesh model this panel.
*/
//==============================================================================
void cPanel::updatePanelMesh()
{
    // clear all triangles
    clear();

    // exit 
    if (m_showPanel)
    {
        // create new model
        cCreatePanel2(this, 
                      m_width,
                      m_height,
                      m_panelRadiusTopLeft,
                      m_panelRadiusTopRight,
                      m_panelRadiusBottomLeft,
                      m_panelRadiusBottomRight,
                      m_numPanelSegmentsPerCorner,
                      cVector3d(0.5 * m_width,
                                0.5 * m_height,
                                0.0),
                      cIdentity3d(),
                      m_panelColorTopLeft,
                      m_panelColorTopRight,
                      m_panelColorBottomLeft,
                      m_panelColorBottomLeft
                      );
    }

    // mark for update
    markForUpdate(false);
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
cPanel* cPanel::copy(const bool a_duplicateMaterialData,
    const bool a_duplicateTextureData, 
    const bool a_duplicateMeshData,
    const bool a_buildCollisionDetector)
{
    // create new instance
    cPanel* obj = new cPanel();

    // copy properties of cPanel
    copyPanelProperties(obj, 
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
void cPanel::copyPanelProperties(cPanel* a_obj,
    const bool a_duplicateMaterialData,
    const bool a_duplicateTextureData, 
    const bool a_duplicateMeshData,
    const bool a_buildCollisionDetector)
{
    // copy properties of cGenericWidget
    copyGenericWidgetProperties(a_obj, 
        a_duplicateMaterialData, 
        a_duplicateTextureData,
        a_duplicateMeshData,
        a_buildCollisionDetector);

    // copy color properties
    a_obj->m_panelColorTopLeft = m_panelColorTopLeft;
    a_obj->m_panelColorTopRight = m_panelColorTopRight;
    a_obj->m_panelColorBottomLeft = m_panelColorBottomLeft;
    a_obj->m_panelColorBottomRight = m_panelColorBottomRight;

    // copy properties of cPanel
    a_obj->set(m_width, 
        m_height,
        m_panelRadiusTopLeft,
        m_panelRadiusTopRight,
        m_panelRadiusBottomLeft,
        m_panelRadiusBottomRight);
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
