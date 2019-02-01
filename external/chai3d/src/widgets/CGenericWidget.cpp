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
    \version   3.2.0 $Rev: 2167 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "widgets/CGenericWidget.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    Constructor of cGenericWidget.
*/
//==============================================================================
cGenericWidget::cGenericWidget()
{
    // set dimension
    m_width  = 0.0;
    m_height = 0.0;

    // we create a small position offset along the x-axis so that when this widget is
    // added as a child to another widget, it is automatically placed 
    // slightly in front ot it.
    m_localPos.set(0.0, 0.0, 0.1);

    // create brute force collision detector
    createBruteForceCollisionDetector();
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
cGenericWidget* cGenericWidget::copy(const bool a_duplicateMaterialData,
                                     const bool a_duplicateTextureData, 
                                     const bool a_duplicateMeshData,
                                     const bool a_buildCollisionDetector)
{
    // create new instance
    cGenericWidget* obj = new cGenericWidget();

    // copy properties of cGenericObject
    copyGenericWidgetProperties(obj, 
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
void cGenericWidget::copyGenericWidgetProperties(cGenericWidget* a_obj,
    const bool a_duplicateMaterialData,
    const bool a_duplicateTextureData, 
    const bool a_duplicateMeshData,
    const bool a_buildCollisionDetector)
{
    // copy properties of cGenericObject
    copyGenericObjectProperties(a_obj, 
        a_duplicateMaterialData, 
        a_duplicateTextureData,
        a_duplicateMeshData,
        a_buildCollisionDetector);

    // copy properties of cGenericWidget
    a_obj->m_width = m_width;
    a_obj->m_height = m_height;
}


//==============================================================================
/*!
    This method rotate the widget around its center point with an angle defined
    in radians. The center point of the widget is defined by the width and height
    values of the widget.

    \param  a_angleRad  Angle of rotation in radians.
*/
//==============================================================================
void cGenericWidget::rotateWidgetAroundCenterRad(double a_angleRad)
{
    cVector3d posCenter0 = m_localRot * cVector3d(0.5 * m_width, 0.5 * m_height, 0);
    rotateWidgetRad(a_angleRad);
    cVector3d posCenter1 = m_localRot * cVector3d(0.5 * m_width, 0.5 * m_height, 0);
    setLocalPos(m_localPos + (posCenter0 - posCenter1));
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
