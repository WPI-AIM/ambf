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
    \version   3.2.0 $Rev: 2050 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "widgets/CScope.h"
//------------------------------------------------------------------------------
#include "graphics/CPrimitives.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    Constructor of cScope.
*/
//==============================================================================
cScope::cScope()
{
    // clear signals
    clearSignals();

    // set default radius values
    m_panelRadiusTopLeft =       10;
    m_panelRadiusTopRight =      10;
    m_panelRadiusBottomLeft =    10;
    m_panelRadiusBottomRight =   10;

    // set default color signals
    m_colorSignal0.setBlueCornflower();
    m_colorSignal1.setGreenMediumSea();
    m_colorSignal2.setPinkDeep();
    m_colorSignal3.setYellowLemonChiffon();

    // default line width
    m_lineWidth = 1.0;

    // default panel background color settings
    m_panelColorTopLeft.setGrayLevel(0.3f);
    m_panelColorTopRight.setGrayLevel(0.3f);
    m_panelColorBottomLeft.setGrayLevel(0.2f);
    m_panelColorBottomRight.setGrayLevel(0.2f);

    // initialize values
    m_scopeWidth = 0.0;
    m_scopeHeight = 0.0;
    m_scopePosition.set(0.0, 0.0, 0.0);

    // set a default size
    setSize(600, 200);
}


//==============================================================================
/*!
    This method sets values for signals 0, 1, 2 and 3.

    \param  a_signalValue0  Value for signal 0.
    \param  a_signalValue1  Value for signal 1.
    \param  a_signalValue2  Value for signal 2.
    \param  a_signalValue3  Value for signal 3.
*/
//==============================================================================
void cScope::setSignalValues(const double a_signalValue0,
                             const double a_signalValue1,
                             const double a_signalValue2,
                             const double a_signalValue3)
{
    m_index1 = (m_index1 + 1) % C_SCOPE_MAX_SAMPLES;

    if (m_index1 == m_index0)
    {
        m_index0 = (m_index0 + 1) % C_SCOPE_MAX_SAMPLES;
    }
    

    // set signal 0
    if (m_signalEnabled[0])
    {
        double value = cClamp(a_signalValue0, m_minValue, m_maxValue);
        m_signals[0][m_index1] = (int)((double)(m_scopeHeight) * (value - m_minValue) / (m_maxValue - m_minValue));
    }
    else
    {
        m_signals[0][m_index1] = 0;
    }

    // set signal 1
    if (m_signalEnabled[1])
    {
        double value = cClamp(a_signalValue1, m_minValue, m_maxValue);
        m_signals[1][m_index1] = (int)((double)(m_scopeHeight) * (value - m_minValue) / (m_maxValue - m_minValue));
    }
    else
    {
        m_signals[1][m_index1] = 0;
    }

    // set signal 2
    if (m_signalEnabled[2])
    {
        double value = cClamp(a_signalValue2, m_minValue, m_maxValue);
        m_signals[2][m_index1] = (int)((double)(m_scopeHeight) * (value - m_minValue) / (m_maxValue - m_minValue));
    }
    else
    {
        m_signals[2][m_index1] = 0;
    }

    // set signal 3
    if (m_signalEnabled[3])
    {
        double value = cClamp(a_signalValue3, m_minValue, m_maxValue);
        m_signals[3][m_index1] = (int)((double)(m_scopeHeight) * (value - m_minValue) / (m_maxValue - m_minValue));
    }
    else
    {
        m_signals[3][m_index1] = 0;
    }

    // initialization case
    if ((m_index0 == 0) && (m_index1 == 1))
    {
        for (int i=0; i<4; i++)
        m_signals[i][0] = m_signals[i][1];
    }
}


//==============================================================================
/*!
    This method enables or disable the display of signals.

    \param  a_signalEnabled0  Status for signal 0.
    \param  a_signalEnabled1  Status for signal 1.
    \param  a_signalEnabled2  Status for signal 2.
    \param  a_signalEnabled3  Status for signal 3.
*/
//==============================================================================
void cScope::setSignalEnabled(const bool a_signalEnabled0,
                              const bool a_signalEnabled1,
                              const bool a_signalEnabled2,
                              const bool a_signalEnabled3)
{
    m_signalEnabled[0] = a_signalEnabled0;
    m_signalEnabled[1] = a_signalEnabled1;
    m_signalEnabled[2] = a_signalEnabled2;
    m_signalEnabled[3] = a_signalEnabled3;
}


//==============================================================================
/*!
    This method resets all signals from scope.
*/
//==============================================================================
void cScope::clearSignals()
{
    m_index0 = 0;
    m_index1 = 0;
}


//==============================================================================
/*!
    This method sets the range of values that can be displayed by the scope.

    \param  a_minValue  Minimum value.
    \param  a_maxValue  Maximum value.
*/
//==============================================================================
void cScope::setRange(const double a_minValue, 
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
}


//==============================================================================
/*!
    This method sets the size of the scope by defining its width and height.

    \param  a_width   Width of scope.
    \param  a_height  Height of scope.
*/
//==============================================================================
void cScope::setSize(const double& a_width, const double& a_height)
{
    // set width
    m_width = a_width;
    m_height = a_height;

    // compute max radius
    double radius = cMax(m_panelRadiusTopLeft, m_panelRadiusBottomLeft) +
                    cMax(m_panelRadiusTopRight, m_panelRadiusBottomRight);

    // adjust margins if needed
    m_marginTop = cMax(m_marginTop, radius);
    m_marginBottom = cMax(m_marginBottom, radius);
    m_marginLeft = cMax(m_marginLeft, radius);
    m_marginRight = cMax(m_marginRight, radius);

    // update model of panel
    updatePanelMesh();

    // set dimension of scope.
    m_scopeWidth = cMax(0.0, m_width - m_marginLeft - m_marginRight);
    m_scopeHeight = cMax(0.0, m_height - m_marginTop - m_marginBottom);

    // set position of scope within panel
    m_scopePosition.set(m_marginLeft, m_marginBottom, 0.0);
}


//==============================================================================
/*!
    This method renders the scope using OpenGL.

    \param  a_options  Rendering options.
*/
//==============================================================================
void cScope::render(cRenderOptions& a_options)
{
#ifdef C_USE_OPENGL

    // render background panel
    if (m_showPanel)
    {
        cMesh::render(a_options);
    }

    /////////////////////////////////////////////////////////////////////////
    // Render parts that are always opaque
    /////////////////////////////////////////////////////////////////////////
    if (SECTION_RENDER_OPAQUE_PARTS_ONLY(a_options))
    {
        if (m_index1 == m_index0) { return; }

        // disable lighting
        glDisable(GL_LIGHTING);

        // set line width
        glLineWidth((GLfloat)m_lineWidth);

        // position scope within panel
        glPushMatrix();
        glTranslated(m_scopePosition(0), m_scopePosition(1), 0.0);

        // render signal 0
        for (int i=0; i<4; i++)
        {
            if (m_signalEnabled[i])
            {
                switch (i)
                {
                    case 0: m_colorSignal0.render(); break;
                    case 1: m_colorSignal1.render(); break;
                    case 2: m_colorSignal2.render(); break;
                    case 3: m_colorSignal3.render(); break;
                }

                int x = (int)m_scopeWidth;
                unsigned int i0 = m_index1;
                int i1 = i0-1;
                if (i1 < 0)
                {
                    i1 = C_SCOPE_MAX_SAMPLES-1;
                }

                glBegin(GL_LINES);
                while ((i0 != m_index0) && (x>0))
                {
                    glVertex3d(x, m_signals[i][i0], 0.0);
                    glVertex3d(x-1, m_signals[i][i1], 0.0);  
                    i0 = i1;
                    i1--;
                    if (i1 < 0)
                    {
                        i1 = C_SCOPE_MAX_SAMPLES-1;
                    }
                    x--;
                }
                glEnd();
            }
        }

        // restore OpenGL settings
        glPopMatrix();
        glEnable(GL_LIGHTING);
    }

#endif
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
cScope* cScope::copy(const bool a_duplicateMaterialData,
    const bool a_duplicateTextureData, 
    const bool a_duplicateMeshData,
    const bool a_buildCollisionDetector)
{
    // create new instance
    cScope* obj = new cScope();

    // copy properties of cGenericObject
    copyScopeProperties(obj, 
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
void cScope::copyScopeProperties(cScope* a_obj,
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

    // copy properties of cScope
    a_obj->m_minValue = m_minValue;
    a_obj->m_maxValue = m_maxValue;
    a_obj->m_lineWidth = m_lineWidth;
    a_obj->m_colorSignal0 = m_colorSignal0;
    a_obj->m_colorSignal1 = m_colorSignal1;
    a_obj->m_colorSignal2 = m_colorSignal2;
    a_obj->m_colorSignal3 = m_colorSignal3;
    a_obj->m_signalEnabled[0] = m_signalEnabled[0];
    a_obj->m_signalEnabled[1] = m_signalEnabled[1];
    a_obj->m_signalEnabled[2] = m_signalEnabled[2];
    a_obj->m_signalEnabled[3] = m_signalEnabled[3];

    a_obj->setSize(m_width, m_height);
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
