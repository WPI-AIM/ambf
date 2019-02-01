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
#include "world/CShapeSphere.h"
//------------------------------------------------------------------------------
#include "shaders/CShaderProgram.h"
//------------------------------------------------------------------------------
#ifdef C_USE_OPENGL
#ifdef MACOSX
#include "OpenGL/glu.h"
#else
#include "GL/glu.h"
#endif
#endif
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    Constructor of cShapeSphere.

    \param  a_radius    Radius of sphere
    \param  a_material  Material property to be applied to object.
*/
//==============================================================================
cShapeSphere::cShapeSphere(const double& a_radius, 
                           cMaterialPtr a_material)
{
    // initialize radius of sphere
    m_radius = fabs(a_radius);

    // set material properties
    if (a_material == nullptr)
    {
        m_material = cMaterial::create();
        m_material->setShininess(100);
        m_material->setWhite();
    }
    else
    {
        m_material = a_material;
    }

    // allocate a new OpenGL quadric object for rendering
#ifdef C_USE_OPENGL
    m_quadric = gluNewQuadric();
#else
    m_quadric = NULL;
#endif
};


//==============================================================================
/*!
    Destructor of cShapeSphere.
*/
//==============================================================================.
cShapeSphere::~cShapeSphere()
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
cShapeSphere* cShapeSphere::copy(const bool a_duplicateMaterialData,
                                 const bool a_duplicateTextureData, 
                                 const bool a_duplicateMeshData,
                                 const bool a_buildCollisionDetector)
{
    // enable display list
    m_useDisplayList = true;

    // create new instance
    cShapeSphere* obj = new cShapeSphere(m_radius);

    // copy properties of cShapeSphere
    copyShapeSphereProperties(obj, 
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
void cShapeSphere::copyShapeSphereProperties(cShapeSphere* a_obj,
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

    // copy properties of cShapeSphere
    a_obj->m_radius = m_radius;
}


//==============================================================================
/*!
    This method sets the radius of this sphere.

    \param  a_radius  Radius of sphere.
*/
//==============================================================================
void cShapeSphere::setRadius(const double& a_radius) 
{ 
    // set new radius
    m_radius = fabs(a_radius);

    // update bounding box
    updateBoundaryBox();

    // invalidate display list
    markForUpdate(false);
}


//==============================================================================
/*!
    This method renders the object using OpenGL

    \param  a_options  Render options.
*/
//==============================================================================
void cShapeSphere::render(cRenderOptions& a_options)
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

        // render a sphere
        gluSphere(m_quadric, m_radius, 36, 36);

        // turn off texture rendering
        if ((m_texture != nullptr) && (m_useTextureMapping))
        {
            m_texture->renderFinalize(a_options);
        }
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
void cShapeSphere::computeLocalInteraction(const cVector3d& a_toolPos,
                                          const cVector3d& a_toolVel,
                                          const unsigned int a_IDN)
{
    // compute distance from center of sphere to tool
    double distance = a_toolPos.length();

    // from the position of the tool, search for the nearest point located
    // on the surface of the sphere
    if (distance > 0)
    {
        m_interactionPoint = cMul( (m_radius/distance), a_toolPos);
        m_interactionNormal = m_interactionPoint;
        m_interactionNormal.normalize();
    }
    else
    {
        m_interactionPoint = a_toolPos;
        m_interactionNormal.set(0,0,1);
    }

    // check if tool is located inside or outside of the sphere
    if (distance <= m_radius)
    {
        m_interactionInside = true;
    }
    else
    {
        m_interactionInside = false;
    }
}


//==============================================================================
/*!
    This method updates the boundary box of this object.
*/
//==============================================================================
void cShapeSphere::updateBoundaryBox()
{
    m_boundaryBoxMin.set(-m_radius, -m_radius, -m_radius);
    m_boundaryBoxMax.set( m_radius,  m_radius,  m_radius);
}


//==============================================================================
/*!
    This method scales the size of this object with given scale factor.

    \param  a_scaleFactor  Scale factor.
*/
//==============================================================================
void cShapeSphere::scaleObject(const double& a_scaleFactor)
{
    // update radius
    m_radius *= a_scaleFactor;

    // update bounding box
    updateBoundaryBox();

    // invalidate display list
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
bool cShapeSphere::computeOtherCollisionDetection(cVector3d& a_segmentPointA,
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
    double collisionDistanceSq = 0.0;

    // compute collision detection between segment and sphere
    if (cIntersectionSegmentSphere(a_segmentPointA,
                                   a_segmentPointB,
                                   cVector3d(0.0, 0.0, 0.0),
                                   m_radius,
                                   collisionPoint0,
                                   collisionNormal0,
                                   collisionPoint1,
                                   collisionNormal1) > 0)
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
