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
#include "widgets/CLevel.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    Constructor of cLevel.
*/
//==============================================================================
cLevel::cLevel()
{
    // use vertex colors for line segments
    m_useVertexColors = true;

    // disable material properties
    m_useMaterialProperty = false;

    // use display list to accelerate rendering
    m_useDisplayList = true;

    // set the default number of increments
    m_numIncrements = 100;

    // set display mode
    m_flagSingleIncrementDisplay = false;

    // set color for activated increments
    m_colorActive.setBlueCornflower();

    // set color for activated increments
    m_colorInactive = m_colorActive;
    m_colorInactive.mul(0.3f);

    // define default value
    m_value = 50;

    // set range of input values
    setRange(0, 100);

    // default width
    setWidth(40);
}


//==============================================================================
/*!
    This method sets the width of the cLevel object.

    \param  a_width  Width size.
*/
//==============================================================================
void cLevel::setWidth(const double a_width)
{
    // sanity test
    m_width = fabs(a_width);

    // update level
    updateLevelMesh();

    // set value
    setValue(m_value);
}


//==============================================================================
/*!
    This method sets the number of increments. This value can vary between 2 
    and 200.

    \param  a_numIncrements  Number of increments.
*/
//==============================================================================
void cLevel::setNumIncrements(const int a_numIncrements)
{
    // clamp value
    m_numIncrements = cClamp(a_numIncrements, 2, 200);

    // update level
    updateLevelMesh();

    // set value
    setValue(m_value);
}


//==============================================================================
/*!
    This method sets the range of input values which are used to display the 
    highlighted segments of cLevel.

    \param  a_minValue  Minimum value.
    \param  a_maxValue  Maximum value.
*/
//==============================================================================
void cLevel::setRange(const double a_minValue, 
                      const double a_maxValue)
{
    // sanity check
    if (a_minValue == a_maxValue)
    {
        return;
    }

    // store values
    m_minValue = cMin(a_minValue, a_maxValue);
    m_maxValue = cMax(a_minValue, a_maxValue);

    // set value
    setValue(m_value);
}


//==============================================================================
/*!
    This method sets a new value. The value is compared with the range defined by 
    m_minValue and m_maxValue. The segment are then highlighted accordingly.

    \param  a_value  New value.
*/
//==============================================================================
void cLevel::setValue(const double a_value)
{
    // sanity check
    if (m_numIncrements > (int)(4 * m_vertices->getNumElements()))
    {
        updateLevelMesh();
    }

    // init variables
    int j=0;

    // clamp value within range
    double value = cClamp(a_value, m_minValue, m_maxValue);
    
    // compute level based on value and range
    int level = (int)((double)(m_numIncrements) * (value - m_minValue) / (m_maxValue - m_minValue));
    if (level > (m_numIncrements-1))
    {
        level = m_numIncrements-1;
    }
    
    // process all increments
    for (int i=0; i<m_numIncrements; i++)
    {
        if ((i == level) || ((i < level) && (!m_flagSingleIncrementDisplay)))
        {
            m_vertices->setColor(j, m_colorActive); j++;
            m_vertices->setColor(j, m_colorActive); j++;
            m_vertices->setColor(j, m_colorActive); j++;
            m_vertices->setColor(j, m_colorActive); j++;
        }
        else
        {
            m_vertices->setColor(j, m_colorInactive); j++;
            m_vertices->setColor(j, m_colorInactive); j++;
            m_vertices->setColor(j, m_colorInactive); j++;
            m_vertices->setColor(j, m_colorInactive); j++;
        }
    }

    // mark for update
    markForUpdate(false);
}


//==============================================================================
/*!
    This method updates the mesh of this object.
*/
//==============================================================================
void cLevel::updateLevelMesh()
{
    // clear all triangles
    clear();

    // set dimensions
    double incrementHeight = (double)((int)(m_width) / 40) + 1;
    double h2 = 2 * incrementHeight;
    double y = 0;
    cVector3d n(0,0,1);

    // create polygons
    for (int i=0; i<m_numIncrements; i++)
    {
        // compute position of vertices
        cVector3d v0(0, y, 0.0);
        cVector3d v1(m_width, y, 0.0);
        cVector3d v2(m_width, y+incrementHeight, 0.0);
        cVector3d v3(0, y+incrementHeight, 0.0);

        // create new vertices
        int vertexIndex0 = newVertex(v0);
        int vertexIndex1 = newVertex(v1);
        int vertexIndex2 = newVertex(v2);
        int vertexIndex3 = newVertex(v3);

        // define vertex normal
        m_vertices->setNormal(vertexIndex0, n);
        m_vertices->setNormal(vertexIndex1, n);
        m_vertices->setNormal(vertexIndex2, n);
        m_vertices->setNormal(vertexIndex3, n);

        // create triangles
        newTriangle(vertexIndex0, vertexIndex1, vertexIndex2);
        newTriangle(vertexIndex0, vertexIndex2, vertexIndex3);

        // increment offset
        y = y + h2;
    }

    m_height = y;
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
cLevel* cLevel::copy(const bool a_duplicateMaterialData,
    const bool a_duplicateTextureData, 
    const bool a_duplicateMeshData,
    const bool a_buildCollisionDetector)
{
    // create new instance
    cLevel* obj = new cLevel();

    // copy properties of cLevel
    copyLevelProperties(obj, 
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
void cLevel::copyLevelProperties(cLevel* a_obj,
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

    // copy properties of cLevel
    a_obj->m_minValue = m_minValue;
    a_obj->m_maxValue = m_maxValue;
    a_obj->m_value = m_value;
    a_obj->m_colorActive = m_colorActive;
    a_obj->m_colorInactive = m_colorInactive;
    a_obj->m_flagSingleIncrementDisplay = m_flagSingleIncrementDisplay;
    a_obj->m_numIncrements = m_numIncrements;

    a_obj->updateLevelMesh();
    a_obj->setValue(m_value);
}

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
