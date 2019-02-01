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
    \version   3.2.0 $Rev: 2153 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "CLabel.h"
//------------------------------------------------------------------------------
using namespace std;
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    Constructor of cLabel.
*/
//==============================================================================
cLabel::cLabel(cFontPtr a_font)
{
    // set label properties
    m_fontScale = 1.0;
    m_letterSpacing = 1.0;
    m_lineSpacing = 1.0;
    m_text = "";
    m_font = a_font;

    // set panel properties
    m_panelColorTopLeft.setGrayLevel(0.40f);
    m_panelColorTopRight.setGrayLevel(0.40f);
    m_panelColorBottomLeft.setGrayLevel(0.40f);
    m_panelColorBottomRight.setGrayLevel(0.40f);
    m_panelRadiusTopLeft = 0.0;
    m_panelRadiusTopRight = 0.0;
    m_panelRadiusBottomLeft = 0.0;
    m_panelRadiusBottomRight = 0.0;
    m_numPanelSegmentsPerCorner = 8;
    m_showPanel = false;
}


//==============================================================================
/*!
    Destructor of cLabel.
*/
//==============================================================================
cLabel::~cLabel()
{
}


//==============================================================================
/*!
    This method renders the label using OpenGL.

    \param  a_options  Rendering options.
*/
//==============================================================================
void cLabel::render(cRenderOptions& a_options)
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
        // disable lighting  properties
        glDisable(GL_LIGHTING);

        // render font color
        m_fontColor.render();

        // render string at desired location
        glPushMatrix();
        glTranslated(m_marginLeft, m_marginBottom, 0.0);
        m_font->renderText(m_text, m_fontColor, m_fontScale, m_letterSpacing, m_lineSpacing, a_options);
        glPopMatrix();

        // enable lighting  properties
        glEnable(GL_LIGHTING);
    }

#endif
}


//==============================================================================
/*!
    This method assigns a text string to rendered by this label.

    \param  a_text  Input text string.
*/
//==============================================================================
void cLabel::setText(const string a_text)
{
    // copy string
    m_text = a_text;

    // get width and height of string
    double w = getTextWidth();
    double h = getTextHeight();

    // add margins
    w = w + m_marginLeft + m_marginRight;
    h = h + m_marginTop + m_marginBottom;

    // set size of panel
    setSize(w, h);

    // adjust size of boundary box
    updateBoundaryBox();
}


//==============================================================================
/*!
    This method sets the font scale factor.

    \param  a_scale  Scale factor.
*/
//==============================================================================
void cLabel::setFontScale(const double a_scale) 
{ 
    // update scale factor
    m_fontScale = fabs(a_scale);

    // adjust size of boundary box
    updateBoundaryBox();
}


//==============================================================================
/*!
    This method returns the width of the current text string in pixels.

    \return Length of current text string in pixels.
*/
//==============================================================================
double cLabel::getTextWidth() const
{
    if (m_font == NULL)
    {
        return (0);
    }
    else
    {
        return (m_fontScale * m_font->getTextWidth(m_text, m_letterSpacing));
    }
}


//==============================================================================
/*!
    This method returns the height of the current text string in pixels.

    \return Height of text string in pixels.
*/
//==============================================================================
double cLabel::getTextHeight() const
{
    if (m_font == NULL)
    {
        return (0);
    }
    else
    {
        return (m_fontScale * m_font->getTextHeight(m_text, m_lineSpacing));
    }
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
cLabel* cLabel::copy(const bool a_duplicateMaterialData,
    const bool a_duplicateTextureData, 
    const bool a_duplicateMeshData,
    const bool a_buildCollisionDetector)
{
    cLabel* obj = new cLabel(m_font);

    // copy cLabel properties
    copyLabelProperties(obj, 
        a_duplicateMaterialData, 
        a_duplicateTextureData,
        a_duplicateMeshData,
        a_buildCollisionDetector);

    // return object
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
void cLabel::copyLabelProperties(cLabel* a_obj,
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

    // copy properties of cLabel
    a_obj->m_fontScale = m_fontScale;
    a_obj->m_fontColor = m_fontColor;

    a_obj->setText(m_text);
}


//==============================================================================
/*!
    This method determines whether a given segment intersects with this label.
    The segment is described by a start \p a_segmentPointA and end 
    point \p a_segmentPointB. 

    \param  a_segmentPointA  Start point of segment.
    \param  a_segmentPointB  End point of segment.
    \param  a_recorder       Recorder which stores all collision events.
    \param  a_settings       Contains collision settings information.

    \return __true__ if a collision has occurred, __false__ otherwise.
*/
//==============================================================================
bool cLabel::computeOtherCollisionDetection(cVector3d& a_segmentPointA,
                                            cVector3d& a_segmentPointB,
                                            cCollisionRecorder& a_recorder,
                                            cCollisionSettings& a_settings)
{
    ////////////////////////////////////////////////////////////////////////////
    // COMPUTE COLLISION
    ////////////////////////////////////////////////////////////////////////////

    // check if panel is enabled. if enabled, ignore as collision will be computed
    // with the triangles composing the panel.
    if (m_showPanel) { return (false); }

    // no collision has occurred yet
    bool hit = false;

    // temp variable to store collision data
    cVector3d collisionPoint;
    cVector3d collisionNormal;
    double c01, c02;
    double collisionDistanceSq = 0.0;

    // compute collision detection between segment and sphere
    if (cIntersectionSegmentTriangle(a_segmentPointA,
                                     a_segmentPointB,
                                     cVector3d(0.0, 0.0, 0.0),
                                     cVector3d(m_width, 0.0, 0.0),
                                     cVector3d(m_width, m_height, 0.0),
                                     true,
                                     true,
                                     collisionPoint,
                                     collisionNormal,
                                     c01,
                                     c02))
    {
        hit = true;
        collisionDistanceSq = cDistanceSq(a_segmentPointA, collisionPoint);
    }
    else
    {
        if (cIntersectionSegmentTriangle(a_segmentPointA,
                                         a_segmentPointB,
                                         cVector3d(0.0, 0.0, 0.0),
                                         cVector3d(m_width, m_height, 0.0),
                                         cVector3d(0.0, m_height, 0.0),
                                         true,
                                         true,
                                         collisionPoint,
                                         collisionNormal,
                                         c01,
                                         c02))
        {
            hit = true;
            collisionDistanceSq = cDistanceSq(a_segmentPointA, collisionPoint);
        }
    }


    ////////////////////////////////////////////////////////////////////////////
    // REPORT COLLISION
    ////////////////////////////////////////////////////////////////////////////

    // here we finally report the new collision to the collision event handler.
    if (hit)
    {
        // we verify if anew collision needs to be created or if we simply
        // need to update the nearest collision.
        if (a_settings.m_checkForNearestCollisionOnly)
        {
            // no new collision event is create. We just check if we need
            // to update the nearest collision
            if(collisionDistanceSq <= a_recorder.m_nearestCollision.m_squareDistance)
            {
                // report basic collision data
                a_recorder.m_nearestCollision.m_type = C_COL_SHAPE;
                a_recorder.m_nearestCollision.m_object = this;
                a_recorder.m_nearestCollision.m_localPos = collisionPoint;
                a_recorder.m_nearestCollision.m_localNormal = collisionNormal;
                a_recorder.m_nearestCollision.m_squareDistance = collisionDistanceSq;
                a_recorder.m_nearestCollision.m_adjustedSegmentAPoint = a_segmentPointA;


                // report advanced collision data
                if (!a_settings.m_returnMinimalCollisionData)
                {
                    a_recorder.m_nearestCollision.m_globalPos = cAdd(getGlobalPos(),
                        cMul(getGlobalRot(),
                        a_recorder.m_nearestCollision.m_localPos));
                    a_recorder.m_nearestCollision.m_globalNormal = cMul(getGlobalRot(),
                        a_recorder.m_nearestCollision.m_localNormal);
                }
            }
        }
        else
        {
            cCollisionEvent newCollisionEvent;

            // report basic collision data
            newCollisionEvent.m_type = C_COL_SHAPE;
            newCollisionEvent.m_object = this;
            newCollisionEvent.m_triangles = nullptr;
            newCollisionEvent.m_localPos = collisionPoint;
            newCollisionEvent.m_localNormal = collisionNormal;
            newCollisionEvent.m_squareDistance = collisionDistanceSq;
            newCollisionEvent.m_adjustedSegmentAPoint = a_segmentPointA;

            // report advanced collision data
            if (!a_settings.m_returnMinimalCollisionData)
            {
                newCollisionEvent.m_globalPos = cAdd(getGlobalPos(),
                    cMul(getGlobalRot(),
                    newCollisionEvent.m_localPos));
                newCollisionEvent.m_globalNormal = cMul(getGlobalRot(),
                    newCollisionEvent.m_localNormal);
            }

            // add new collision even to collision list
            a_recorder.m_collisions.push_back(newCollisionEvent);

            // check if this new collision is a candidate for "nearest one"
            if(collisionDistanceSq <= a_recorder.m_nearestCollision.m_squareDistance)
            {
                a_recorder.m_nearestCollision = newCollisionEvent;
            }
        }
    }

    // return result
    return (hit);
}

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
