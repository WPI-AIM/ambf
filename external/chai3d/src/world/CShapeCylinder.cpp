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
    \author    Sebastien Grange
    \version   3.2.0 $Rev: 2174 $
 */
//==============================================================================

//------------------------------------------------------------------------------
#include <algorithm>
//------------------------------------------------------------------------------
#include "world/CShapeCylinder.h"
#include "shaders/CShaderProgram.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    Constructor of cShapeCylinder.

    \param  a_baseRadius  Base radius of cylinder
    \param  a_topRadius   Top radius of cylinder.
    \param  a_height      Height of cylinder.
    \param  a_material    Material property to be applied to object.
 */
//==============================================================================.
cShapeCylinder::cShapeCylinder(const double a_baseRadius,
                               const double a_topRadius,
                               const double a_height,
                               cMaterialPtr a_material)
{
    // enable display list
    m_useDisplayList = true;

    // initialize cylinder parameters
    m_baseRadius = fabs(a_baseRadius);
    m_topRadius  = fabs(a_topRadius);
    m_height     = fabs(a_height);

    // set material properties
    if (a_material == nullptr)
    {
        m_material = cMaterial::create();
        m_material->setShininess(100);
        m_material->m_ambient.set ((float)0.3, (float)0.3, (float)0.3);
        m_material->m_diffuse.set ((float)0.1, (float)0.7, (float)0.8);
        m_material->m_specular.set((float)1.0, (float)1.0, (float)1.0);
    }
    else
    {
        m_material = a_material;
    }

    // allocate a new OpenGL quadric object for rendering
    #ifdef C_USE_OPENGL
    m_quadric = gluNewQuadric ();
    #else
    m_quadric = NULL;
    #endif

    // initialize boundary box
    updateBoundaryBox ();
};


//==============================================================================
/*!
    Destructor of cShapeCylinder.
 */
//==============================================================================.
cShapeCylinder::~cShapeCylinder()
{
    // deallocate OpenGL quadric object used for rendering
    #ifdef C_USE_OPENGL
    gluDeleteQuadric(m_quadric);
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
cShapeCylinder* cShapeCylinder::copy(const bool a_duplicateMaterialData,
                                     const bool a_duplicateTextureData, 
                                     const bool a_duplicateMeshData,
                                     const bool a_buildCollisionDetector)
{
    // create new instance
    cShapeCylinder* obj = new cShapeCylinder(m_baseRadius, m_topRadius, m_height);

    // copy generic properties
    copyShapeCylinderProperties(obj, 
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
void cShapeCylinder::copyShapeCylinderProperties(cShapeCylinder* a_obj,
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

    // copy properties of cShapeCylinder
    a_obj->m_baseRadius = m_baseRadius;
    a_obj->m_topRadius = m_topRadius;
    a_obj->m_height = m_height;
}


//==============================================================================
/*!
    This method renders the object using OpenGL

    \param  a_options  Render options.
 */
//==============================================================================
void cShapeCylinder::render(cRenderOptions& a_options)
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
    // Render parts that use material properties
    /////////////////////////////////////////////////////////////////////////
    if (SECTION_RENDER_PARTS_WITH_MATERIALS(a_options, m_useTransparency))
    {
        // render material properties
        if (m_useMaterialProperty)
        {
            m_material->render(a_options);
        }

        // set rendering style
        gluQuadricDrawStyle (m_quadric, GLU_FILL);

        // set normal-rendering mode
        gluQuadricNormals (m_quadric, GLU_SMOOTH);

        // render texture property if defined
        if ((m_texture != nullptr) && (m_useTextureMapping))
        {
            // activate texture
            m_texture->renderInitialize(a_options);

            // generate texture coordinates
            gluQuadricTexture(m_quadric, GL_TRUE);
        }

        // render a cylinder
        gluCylinder(m_quadric, m_baseRadius, m_topRadius, m_height, 36, 36);

        // turn off texture rendering
        if ((m_texture != nullptr) && (m_useTextureMapping))
        {
            m_texture->renderFinalize(a_options);
        }

        // close the cylinder
        gluQuadricOrientation (m_quadric, GLU_INSIDE);
        gluDisk               (m_quadric, 0.0, m_baseRadius, 36, 36);
        glTranslated          (0.0, 0.0, m_height);
        gluQuadricOrientation (m_quadric, GLU_OUTSIDE);
        gluDisk               (m_quadric, 0.0, m_topRadius, 36, 36);
        glTranslated          (0.0, 0.0, -m_height);
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
void cShapeCylinder::computeLocalInteraction(const cVector3d& a_toolPos,
                                             const cVector3d& a_toolVel,
                                             const unsigned int a_IDN)
{
    const cVector3d axis(0.0, 0.0, 1.0);
    const cVector3d base(0.0, 0.0, 0.0);
    const cVector3d top (0.0, 0.0, m_height);

    // compute closest vertical segment on surface of cylinder
    cVector3d projAxis = cProjectPointOnLine(a_toolPos, base, axis);
    double    radius   = m_baseRadius + projAxis(2) /m_height*(m_topRadius - m_baseRadius);
    cVector3d dir      = cSub(a_toolPos, projAxis);
    double    dirLen   = dir.length();
    dir.normalize();
    cVector3d pointBase = base + cMul (m_baseRadius, dir);
    cVector3d pointTop  = top  + cMul (m_topRadius,  dir);

    // compute projection on both (top and base) disks and segment
    cVector3d projBase    = cProjectPointOnDiskXY(a_toolPos, 0.0, m_baseRadius);
    cVector3d projTop     = cProjectPointOnDiskXY(a_toolPos, m_height, m_topRadius);
    cVector3d projSurface = cProjectPointOnSegment(a_toolPos, pointBase, pointTop);

    // pick closest to toolPos
    double baseLen = cSub(a_toolPos,projBase).length();
    double topLen  = cSub(a_toolPos,projTop).length();
    double projLen = cSub(a_toolPos,projSurface).length();
    
    if (baseLen < topLen && baseLen < projLen) 
    {
       m_interactionPoint = projBase;
       m_interactionNormal.set(0.0, 0.0, 1.0);
    }
    
    else if (topLen  < baseLen && topLen  < projLen) 
    {
        m_interactionPoint = projTop;
        m_interactionNormal.set(0.0, 0.0, 1.0);
    }
    
    else
    {
        m_interactionPoint = projSurface;
        m_interactionNormal.set(projSurface.x(), projSurface.y(), 0.0);
        if (m_interactionNormal.lengthsq() > 0.0)
        {
            m_interactionNormal.normalize();
        }
        else
        {
            m_interactionNormal.set(0.0, 0.0, 1.0);
        }
    }

    // determine inside or out
    if (dirLen > radius || a_toolPos(2) > m_height || a_toolPos(2)  < 0.0) 
    {
        m_interactionInside = false;
    }
    else
    {
        m_interactionInside = true;
    }
}


//==============================================================================
/*!
    This method updates the boundary box of this object.
 */
//==============================================================================
void cShapeCylinder::updateBoundaryBox()
{
    double rad = std::max (m_baseRadius, m_topRadius);

    m_boundaryBoxMin.set(-rad, -rad, 0.0);
    m_boundaryBoxMax.set( rad,  rad, m_height);
}


//==============================================================================
/*!
    This method scales the size of this object with given scale factor.

    \param  a_scaleFactor  Scale factor.
*/
//==============================================================================
void cShapeCylinder::scaleObject(const double& a_scaleFactor)
{
    // update dimensions
    m_baseRadius *= a_scaleFactor;
    m_topRadius  *= a_scaleFactor;
    m_height     *= a_scaleFactor;

    // update bounding box
    updateBoundaryBox();

    // mark for update
    markForUpdate(false);
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
bool cShapeCylinder::computeOtherCollisionDetection(cVector3d& a_segmentPointA,
                                                  cVector3d& a_segmentPointB,
                                                  cCollisionRecorder& a_recorder,
                                                  cCollisionSettings& a_settings)
{
    ////////////////////////////////////////////////////////////////////////////
    // COMPUTE COLLISION
    ////////////////////////////////////////////////////////////////////////////

    // ignore shape if requested
 //   if (a_settings.m_ignoreShapes) { return (false); }

    // no collision has occurred yet
    bool hit = false;

    // temp variable to store collision data
    cVector3d collisionPoint;
    cVector3d collisionNormal;
    double collisionDistanceSq = 0.0;

    // compute collision detection between segment and sphere
    if (cIntersectionSegmentCylinder(a_segmentPointA,
                                     a_segmentPointB,
                                     m_baseRadius,
                                     m_topRadius,
                                     m_height,
                                     collisionPoint,
                                     collisionNormal) > 0)
    {
        hit = true;
        collisionDistanceSq = cDistanceSq(a_segmentPointA, collisionPoint);
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
