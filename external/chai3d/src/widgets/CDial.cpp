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
    \version   3.2.0 $Rev: 2163 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "widgets/CDial.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    Constructor of cDial.
*/
//==============================================================================
cDial::cDial()
{
    // initialization
    m_value0 = 0.0;
    m_value1 = 0.0;

    // set the default number of increments
    m_numIncrements = 72;

    // set display mode
    m_flagSingleIncrementDisplay = false;

    // use vertex colors for line segments
    m_useVertexColors = true;

    // disable material properties
    m_useMaterialProperty = false;

    // use display list to accelerate rendering
    m_useDisplayList = true;

    // set color for activated increments
    m_colorActive.setBlueCornflower();

    // set color for activated increments
    m_colorInactive = m_colorActive;
    m_colorInactive.mul(0.3f);

    // define default values
    m_value0 = 0;
    m_value1 = 50;

    // set range of input values
    setRange(0, 100);

    // default width
    setSize(100);
}


//==============================================================================
/*!
    This method sets the number of displayed increments. This value can vary 
    between 2 and 200.

    \param  a_numIncrements  Number of increments.
*/
//==============================================================================
void cDial::setNumIncrements(const int a_numIncrements)
{
    // clamp value
    m_numIncrements = cClamp(a_numIncrements, 2, 360);

    // update level
    updateDialMesh();

    // set value
    setValues(m_value0, m_value1);
}


//==============================================================================
/*!
    This method sets the range of input values which are used to command the 
    highlighted segments of cLevel.

    \param  a_minValue  Minimum value.
    \param  a_maxValue  Maximum value.
*/
//==============================================================================
void cDial::setRange(const double a_minValue,
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
    setValues(m_value0, m_value1);
}

//==============================================================================
/*!
    This method sets a new value. The value is compared to the 
    range defined by m_minValue and m_maxValue. The segment are then highlighted 
    accordingly.

    \param  a_value  New value.
*/
//==============================================================================
void cDial::setValue(const double a_value)
{
    // sanity check
    if (m_numIncrements > (int)(4 * m_vertices->getNumElements()))
    {
        updateDialMesh();
    }

    // init variables
    int j=0;

    // clamp value within range
    m_value0 = m_minValue;
    m_value1 = cClamp(a_value, m_minValue, m_maxValue);

    // compute level based on value and range
    int level = (int)((double)(m_numIncrements) * (m_value1 - m_minValue) / (m_maxValue - m_minValue));
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
    This method sets a new value 0. The value is compared with the range defined 
    by m_minValue and m_maxValue. The segment are then highlighted accordingly.

    \param  a_value  New value 0.
*/
//==============================================================================
void cDial::setValue0(const double a_value)
{
    // sanity check
    if (m_numIncrements > (int)(4 * m_vertices->getNumElements()))
    {
        updateDialMesh();
    }

    // init variables
    int j=0;

    // clamp value within range
    m_value0 = cClamp(a_value, m_minValue, m_maxValue);
    
    // compute level based on value and range
    int level0 = (int)((double)(m_numIncrements) * (m_value0 - m_minValue) / (m_maxValue - m_minValue));
    int level1 = (int)((double)(m_numIncrements) * (m_value1 - m_minValue) / (m_maxValue - m_minValue));

    // process increments
    if (m_flagSingleIncrementDisplay)
    {
        for (int i=0; i<m_numIncrements; i++)
        {
            if ((i == level0) || (i == level1))
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
    }
    else
    {
        if (level1 >= level0)
        {
            for (int i=0; i<m_numIncrements; i++)
            {
                if ((i >= level0) && (i <= level1))
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
        }
        else
        {
            for (int i=0; i<m_numIncrements; i++)
            {
                if ((i <= level1) || (i >= level0))
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
        }
    }

    // mark for update
    markForUpdate(false);
}


//==============================================================================
/*!
    This method sets a new value 1. The value is compared with the range 
    defined by m_minValue and m_maxValue. The segment are then highlighted
    accordingly.

    \param  a_value  New value 1.
*/
//==============================================================================
void cDial::setValue1(const double a_value)
{
    // clamp value within range
    m_value1 = cClamp(a_value, m_minValue, m_maxValue);

    setValue0(m_value0);
}


//==============================================================================
/*!
    This method sets both values 0 and 1. The display segments are then 
    highlighted accordingly.

    \param  a_value0  Value 0.
    \param  a_value1  Value 1.
*/
//==============================================================================
void cDial::setValues(const double a_value0,
                      const double a_value1)
{
    // clamp value within range
    m_value0 = cClamp(a_value0, m_minValue, m_maxValue);
    m_value1 = cClamp(a_value1, m_minValue, m_maxValue);

    // update increments
    setValue0(m_value0);
}


//==============================================================================
/*!
    This method sets the overall dimension of the dial.

    \param  a_size  Size value.
*/
//==============================================================================
void cDial::setSize(const double a_size)
{
    // sanity test
    m_width = fabs(a_size);
    m_height = fabs(a_size);

    // update mesh
    updateDialMesh();

    // set value
    setValues(m_value0, m_value1);
}


//==============================================================================
/*!
    This method updates the mesh of this dial.
*/
//==============================================================================
void cDial::updateDialMesh()
{
    // clear all triangles
    clear();

    // set dimension
    double r0 = 0.7 * (0.5 * m_width);
    double r1 = 1.0 * (0.5 * m_width);

    double a = (double)(2*C_PI) / (double)(m_numIncrements);
    cVector3d n(0,0,1);
    double angle = 0;

    double cos0 = 1.0;
    double sin0 = 0.0;

    // create polygons
    for (int i=0; i<m_numIncrements; i++)
    {
        // compute position of vertices

        double cos1 = cos(angle+a);
        double sin1 = sin(angle+a);

        cVector3d v0(-r0 * sin0, -r0 * cos0, 0.0);
        cVector3d v1(-r1 * sin0, -r1 * cos0, 0.0);
        cVector3d v2(-r0 * sin1, -r0 * cos1, 0.0);
        cVector3d v3(-r1 * sin1, -r1 * cos1, 0.0);

        cos0 = cos1;
        sin0 = sin1;

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
        newTriangle(vertexIndex2, vertexIndex1, vertexIndex3);

        // increment offset
        angle = angle + a;
    }

    // update boundary box
    updateBoundaryBox();
}

//==============================================================================
/*!
    This method updates the bounding box of this object.
 */
//==============================================================================
void cDial::updateBoundaryBox()
{
    double s = 0.5 * m_width;
    m_boundaryBoxMin.set(-s, -s, 0.0);
    m_boundaryBoxMax.set( s,  s, 0.0);
}


//==============================================================================
/*!
    This method create a copy of itself.

    \param  a_duplicateMaterialData   If __true__, material (if available) is duplicated, otherwise it is shared.
    \param  a_duplicateTextureData    If __true__, texture data (if available) is duplicated, otherwise it is shared.
    \param  a_duplicateMeshData       If __true__, mesh data (if available) is duplicated, otherwise it is shared.
    \param  a_buildCollisionDetector  If __true__, collision detector (if available) is duplicated, otherwise it is shared.

    \return Pointer to new object.
*/
//==============================================================================
cDial* cDial::copy(const bool a_duplicateMaterialData,
    const bool a_duplicateTextureData, 
    const bool a_duplicateMeshData,
    const bool a_buildCollisionDetector)
{
    // create new instance
    cDial* obj = new cDial();

    // copy properties of cDial
    copyDialProperties(obj, 
        a_duplicateMaterialData, 
        a_duplicateTextureData,
        a_duplicateMeshData,
        a_buildCollisionDetector);

    // return
    return (obj);
}


//==============================================================================
/*!
    This method copy properties of this object to another.

    \param  a_obj                     Destination object where properties are copied to.
    \param  a_duplicateMaterialData   If __true__, material (if available) is duplicated, otherwise it is shared.
    \param  a_duplicateTextureData    If __true__, texture data (if available) is duplicated, otherwise it is shared.
    \param  a_duplicateMeshData       If __true__, mesh data (if available) is duplicated, otherwise it is shared.
    \param  a_buildCollisionDetector  If __true__, collision detector (if available) is duplicated, otherwise it is shared.
*/
//==============================================================================
void cDial::copyDialProperties(cDial* a_obj,
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
    a_obj->m_colorActive = m_colorActive;
    a_obj->m_colorInactive = m_colorInactive;
    a_obj->m_flagSingleIncrementDisplay = m_flagSingleIncrementDisplay;
    a_obj->m_numIncrements = m_numIncrements;

    a_obj->updateDialMesh();
    a_obj->setValues(m_value0, m_value1);
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
