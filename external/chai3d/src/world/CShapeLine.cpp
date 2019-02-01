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
    \version   3.2.0 $Rev: 2174 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "world/CShapeLine.h"
//------------------------------------------------------------------------------
#include "graphics/CDraw3D.h"
#include "shaders/CShaderProgram.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    Constructor of cShapeLine.
*/
//==============================================================================
cShapeLine::cShapeLine()
{
    // default line settings
    reset();

    // update line point
    updateLinePoints();
};


//==============================================================================
/*!
    Constructor of cShapeLine. The line is constructed by passing two points
    expressed in the local reference frame of the line.

    \param  a_pointA  Point A of line.
    \param  a_pointB  Point B of line.
*/
//==============================================================================
cShapeLine::cShapeLine(const cVector3d& a_pointA, const cVector3d& a_pointB)
{
    // default line settings
    reset();

    // initialize line with start and end points.
    m_pointA.copyfrom(a_pointA);
    m_pointB.copyfrom(a_pointB);

    // update line point
    updateLinePoints();
};


//==============================================================================
/*!
    Constructor of cShapeLine. The line is constructed by passing two 
    object of type cGenericobjects. The line connects the reference frame 
    origin of each object.

    \param  a_objectA  Object A to which line is attached.
    \param  a_objectB  Object B to which line is attached.
*/
//==============================================================================
cShapeLine::cShapeLine(cGenericObject* a_objectA, cGenericObject* a_objectB)
{
    // default line settings
    reset();

    // set objects to which line is attached
    m_objectA = a_objectA;
    m_objectB = a_objectB;

    // update line point
    updateLinePoints();
};


//==============================================================================
/*!
    Constructor of cShapeLine. The line is constructed by passing two
    object of type cGenericobjects. The line connects the a_pointA and a_pointB
    expressed in the local reference frame of each object.

    \param  a_objectA  Object A to which line is attached.
    \param  a_pointA   Point A of line.
    \param  a_objectB  Object B to which line is attached.
    \param  a_pointB   Point B of line.
*/
//==============================================================================
cShapeLine::cShapeLine(cGenericObject* a_objectA, const cVector3d& a_pointA,
                       cGenericObject* a_objectB, const cVector3d& a_pointB)
{
    // default line settings
    reset();

    // set objects to which line is attached
    m_objectA = a_objectA;
    m_objectB = a_objectB;

    // initialize line with start and end points.
    m_pointA.copyfrom(a_pointA);
    m_pointB.copyfrom(a_pointB);

    // update line point
    updateLinePoints();
};


//==============================================================================
/*!
    Initialize line data.
*/
//==============================================================================
void cShapeLine::reset()
{
    // default line width
    m_lineWidth = 1.0;

    // initialize line with start and end points.
    m_pointA.zero();
    m_pointB.zero();

    // set color properties
    m_colorPointA.set(0.7f, 0.7f, 0.7f, 1.0f);
    m_colorPointB.set(0.7f, 0.7f, 0.7f, 1.0f);

    // initialize stippling settings
    m_stippleFactor = 1;
    m_stipplePattern = 0xFFFF;

    // objects
    m_objectA = NULL;
    m_objectB = NULL;
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
cShapeLine* cShapeLine::copy(const bool a_duplicateMaterialData,
                             const bool a_duplicateTextureData, 
                             const bool a_duplicateMeshData,
                             const bool a_buildCollisionDetector)
{
    // create new instance
    cShapeLine* obj = new cShapeLine(m_pointA, m_pointB);

    // copy properties of cShapeLine
    copyShapeLineProperties(obj,
        a_duplicateMaterialData,
        a_duplicateTextureData, 
        a_duplicateMeshData,
        a_buildCollisionDetector);

    // return
    return (obj);
}


//==============================================================================
/*!
    This method copies all properties of this multi-mesh to another.

    \param  a_obj                     Destination object where properties are copied to.
    \param  a_duplicateMaterialData   If __true__, material (if available) is duplicated, otherwise it is shared.
    \param  a_duplicateTextureData    If __true__, texture data (if available) is duplicated, otherwise it is shared.
    \param  a_duplicateMeshData       If __true__, mesh data (if available) is duplicated, otherwise it is shared.
    \param  a_buildCollisionDetector  If __true__, collision detector (if available) is duplicated, otherwise it is shared.
*/
//==============================================================================
void cShapeLine::copyShapeLineProperties(cShapeLine* a_obj,
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

    // copy properties of cShapeLine
    a_obj->m_pointA = m_pointA;
    a_obj->m_pointB = m_pointB;
    a_obj->m_colorPointA = m_colorPointA;
    a_obj->m_colorPointB = m_colorPointB;
    a_obj->m_stippleFactor = m_stippleFactor;
    a_obj->m_stipplePattern = m_stipplePattern;
    a_obj->m_lineWidth = m_lineWidth;
    a_obj->m_objectA = m_objectA;
    a_obj->m_objectB = m_objectB;
    a_obj->m_linePointA = m_linePointA;
    a_obj->m_linePointB = m_linePointB;
}


//==============================================================================
/*!
    This method renders the object using OpenGL

    \param  a_options  Render options.
*/
//==============================================================================
void cShapeLine::render(cRenderOptions& a_options)
{
#ifdef C_USE_OPENGL

    /////////////////////////////////////////////////////////////////////////
    // ENABLE SHADER
    /////////////////////////////////////////////////////////////////////////
    if ((m_shaderProgram != nullptr) && (!a_options.m_creating_shadow_map))
    {
        // enable shader
        m_shaderProgram->use(this, a_options);
    }


    /////////////////////////////////////////////////////////////////////////
    // Render parts that are always opaque
    /////////////////////////////////////////////////////////////////////////
    if (SECTION_RENDER_OPAQUE_PARTS_ONLY(a_options))
    {
        // update line points
        updateLinePoints();

        // disable lighting
        glDisable(GL_LIGHTING);

        // set line width
        glLineWidth((GLfloat)m_lineWidth);

        // set stipple settings
        if (m_stipplePattern == 0xFFFF)
        {
            glDisable(GL_LINE_STIPPLE);
        }
        else
        {
            glLineStipple(m_stippleFactor, m_stipplePattern);
            glEnable(GL_LINE_STIPPLE);
        }

        // draw line
        glBegin(GL_LINES);
            m_colorPointA.render();
            glVertex3dv(&m_linePointA(0));
            m_colorPointB.render();
            glVertex3dv(&m_linePointB(0));
        glEnd();

        // restore OpenGL to default value
        glEnable(GL_LIGHTING);
        glDisable(GL_LINE_STIPPLE);
    }


    /////////////////////////////////////////////////////////////////////////
    // DISABLE SHADER
    /////////////////////////////////////////////////////////////////////////
    if ((m_shaderProgram != nullptr) && (!a_options.m_creating_shadow_map))
    {
        // disable shader
        m_shaderProgram->disable();
    }

#endif
}


//==============================================================================
/*!
    This method uses the position of the tool and searches for the nearest point
    located at the surface of the current object and identifies if the point is
    located inside or outside of the object.

    \param  a_toolPos  Position of the tool.
    \param  a_toolVel  Velocity of the tool.
    \param  a_IDN      Identification number of the force algorithm.
*/
//==============================================================================
void cShapeLine::computeLocalInteraction(const cVector3d& a_toolPos,
                                         const cVector3d& a_toolVel,
                                         const unsigned int a_IDN)
{
    // the tool can never be inside the line
    m_interactionInside = false;

    // if both point are equal
    m_interactionPoint = cProjectPointOnSegment(a_toolPos,
                                                m_linePointA,
                                                m_linePointB);

    // compute normal
    cVector3d normal = a_toolPos - m_interactionPoint;
    if (normal.lengthsq() > 0.0)
    {
        normal.normalize();
        m_interactionNormal = normal;
    }
    else
    {
        m_interactionNormal.set(0,0,1);
    }
}


//==============================================================================
/*!
    This method updates the boundary box of this object.
*/
//==============================================================================
void cShapeLine::updateBoundaryBox()
{
    m_boundaryBoxMin.set(cMin(m_linePointA(0) , m_linePointB(0) ),
                         cMin(m_linePointA(1) , m_linePointB(1) ),
                         cMin(m_linePointA(2) , m_linePointB(2) ));

    m_boundaryBoxMax.set(cMax(m_linePointA(0) , m_linePointB(0) ),
                         cMax(m_linePointA(1) , m_linePointB(1) ),
                         cMax(m_linePointA(2) , m_linePointB(2) ));
}


//==============================================================================
/*!
    This method scales the size of this object with given scale factor.

    \param  a_scaleFactor  Scale factor.
*/
//==============================================================================
void cShapeLine::scaleObject(const double& a_scaleFactor)
{
    if (m_objectA != NULL)
        m_pointA.mul(a_scaleFactor);

    if (m_objectB != NULL)
        m_pointB.mul(a_scaleFactor);

    updateLinePoints();

    updateBoundaryBox();
}


//==============================================================================
/*!
    This method specifies the line properties \n\n

    a_stippleFactor Specifies a multiplier for each bit in the line stipple 
    pattern. If factor is 3, for example, each bit in the pattern is used 
    three times before the next bit in the pattern is used. factor is 
    clamped to the range [1, 256] and defaults to 1.\n\n

    a_stipplePattern specifies a 16-bit integer whose bit pattern determines
    which fragments of a line will be drawn when the line is rasterized.
    Bit zero is used first; the default pattern is all 1's.

    \param  a_stippleFactor   Line stipple factor.
    \param  a_stipplePattern  Line stipple pattern.
*/
//==============================================================================
void cShapeLine::setLineStipple(const GLint a_stippleFactor, 
    const GLushort a_stipplePattern)
{
    m_stippleFactor = a_stippleFactor;
    m_stipplePattern = a_stipplePattern;
}


//==============================================================================
/*!
    This method updates the position of line extremities according to if the
    line is attched to an object or simply efined in the local coordinate
    system of the line itself
*/
//==============================================================================
void cShapeLine::updateLinePoints()
{
    cTransform T = this->getGlobalTransform();
    T.invert();

    if (m_objectA != NULL)
    {
        m_linePointA = T * m_objectA->getGlobalTransform() * m_pointA;
    }
    else
    {
        m_linePointA = m_pointA;
    }

    if (m_objectB != NULL)
    {
        m_linePointB = T * m_objectB->getGlobalTransform() * m_pointB;
    }
    else
    {
        m_linePointB = m_pointB;
    }
}


//==============================================================================
/*!
    This method determines whether a given segment intersects this object. \n
    The segment is described by a start point \p a_segmentPointA and end 
    point \p a_segmentPointB. \n
    All detected collisions are reported in the collision recorder passed 
    by argument \p a_recorder. \n
    Specifications about the type of collisions reported are specified by 
    argument \p a_settings.

    \param  a_segmentPointA  Start point of segment.
    \param  a_segmentPointB  End point of segment.
    \param  a_recorder       Recorder which stores all collision events.
    \param  a_settings       Collision settings information.

    \return __true__ if a collision has occurred, __false__ otherwise.
*/
//==============================================================================
bool cShapeLine::computeOtherCollisionDetection(cVector3d& a_segmentPointA,
                                                cVector3d& a_segmentPointB,
                                                cCollisionRecorder& a_recorder,
                                                cCollisionSettings& a_settings)
{
    ////////////////////////////////////////////////////////////////////////////
    // COMPUTE COLLISION
    ////////////////////////////////////////////////////////////////////////////

    // ignore shape if requested
    if (a_settings.m_ignoreShapes) { return (false); }

    // no collision has occurred yet
    bool hit = false;

    // temp variable to store collision data
    cVector3d collisionPoint0, collisionPoint1;
    cVector3d collisionNormal0, collisionNormal1;
    double c0, c1;
    double collisionDistanceSq = 0.0;

    // compute collision detection between segment and line
    if (cIntersectionSegmentToplessCylinder(a_segmentPointA,
                                   a_segmentPointB,
                                   m_linePointA,
                                   m_linePointB,
                                   0.005 * cDistance(m_linePointB, m_linePointB),
                                   collisionPoint0,
                                   collisionNormal0,
                                   c0,
                                   collisionPoint1,
                                   collisionNormal1,
                                   c1) > 0)
    {
        hit = true;
        collisionDistanceSq = cDistanceSq(a_segmentPointA, collisionPoint0);
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
                a_recorder.m_nearestCollision.m_localPos = collisionPoint0;
                a_recorder.m_nearestCollision.m_localNormal = collisionNormal0;
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
            newCollisionEvent.m_localPos = collisionPoint0;
            newCollisionEvent.m_localNormal = collisionNormal0;
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
