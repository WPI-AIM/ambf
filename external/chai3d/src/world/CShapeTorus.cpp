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
#include "world/CShapeTorus.h"
//------------------------------------------------------------------------------
#include "shaders/CShaderProgram.h"
//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    Constructor of cShapeTorus.

    \param  a_innerRadius  Inside radius of torus.
    \param  a_outerRadius  Outside radius of torus.
    \param  a_material     Material property to be applied to object.
*/
//==============================================================================
cShapeTorus::cShapeTorus(const double& a_innerRadius, 
                         const double& a_outerRadius, 
                         cMaterialPtr a_material)
{
    // enable display list
    m_useDisplayList = true;

    // initialize dimensions of torus
    setSize(a_innerRadius, a_outerRadius);

    // set resolution of graphical model
    m_resolution = 64;

    // set material properties
    if (a_material == nullptr)
    {
        m_material = cMaterial::create();
        m_material->setShininess(100);
        m_material->m_ambient.set((float)0.3, (float)0.3, (float)0.3);
        m_material->m_diffuse.set((float)0.7, (float)0.7, (float)0.7);
        m_material->m_specular.set((float)1.0, (float)1.0, (float)1.0);
        m_material->setStiffness(100.0);
    }
    else
    {
        m_material = a_material;
    }
};


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
cShapeTorus* cShapeTorus::copy(const bool a_duplicateMaterialData,
                               const bool a_duplicateTextureData, 
                               const bool a_duplicateMeshData,
                               const bool a_buildCollisionDetector)
{
    // create new instance
    cShapeTorus* obj = new cShapeTorus(m_innerRadius, m_outerRadius); 

    // copy generic object properties
    copyShapeTorusProperties(obj,
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
void cShapeTorus::copyShapeTorusProperties(cShapeTorus* a_obj,
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

    // copy properties of cShapeTorus
    a_obj->m_innerRadius = m_innerRadius;
    a_obj->m_outerRadius = m_outerRadius;
    a_obj->m_resolution = m_resolution;
}


//==============================================================================
/*!
    This method sets the dimensions of the torus.

    \param  a_innerRadius  Inner radius of the torus.
    \param  a_outerRadius  Outer radius of the torus.
*/
//==============================================================================
 void cShapeTorus::setSize(const double& a_innerRadius, 
                           const double& a_outerRadius) 
 {
     // set new dimensions
     m_innerRadius = fabs(a_innerRadius);
     m_outerRadius = fabs(a_outerRadius);
     
     // update bounding box
     updateBoundaryBox(); 

     // update display list
    markForUpdate(false);
 }


//==============================================================================
/*!
    This method renders the object using OpenGL

    \param  a_options  Rendering options.
*/
//==============================================================================
void cShapeTorus::render(cRenderOptions& a_options)
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

        // render texture property if defined
        if ((m_texture != nullptr) && (m_useTextureMapping))
        {
            // the torus can only be rendered with texture if environmental mapping
            // is used. This limitation comes from the fact that the current implementation of
            // cDrawSolidTorus() does not generate any texture coordinates. 
            // If you wish to create a textured torus, you may build it using a cMesh class and 
            // the mesh primitives available in file CPrimitives.h
            if (m_texture->getSphericalMappingEnabled())
            {
                // activate texture
                m_texture->renderInitialize(a_options);
            }
        }

        // draw torus
        if (!m_displayList.render(m_useDisplayList))
        {
            // create display list if requested
            m_displayList.begin(m_useDisplayList);

            // draw object
            cDrawSolidTorus(m_innerRadius, m_outerRadius, m_resolution, m_resolution);
            
            // finalize display list
            m_displayList.end(true);
        }
  
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
void cShapeTorus::computeLocalInteraction(const cVector3d& a_toolPos,
                                          const cVector3d& a_toolVel,
                                          const unsigned int a_IDN)
{
    cVector3d toolProjection = a_toolPos;
    toolProjection.z(0.0);
    m_interactionNormal.set(0,0,1);

    // search for the nearest point on the torus medial axis
    if (a_toolPos.lengthsq() > C_SMALL)
    {
        cVector3d pointAxisTorus = cMul(m_outerRadius, cNormalize(toolProjection));

        // compute eventual penetration of tool inside the torus
        cVector3d vectTorusTool = cSub(a_toolPos, pointAxisTorus);

        double distance = vectTorusTool.length();

        // normal
        if (distance > 0.0)
        {
            m_interactionNormal = vectTorusTool;
            m_interactionNormal.normalize();
        }

        // tool is located inside the torus
        if ((distance < m_innerRadius) && (distance > 0.001))
        {
            m_interactionInside = true;
        }

        // tool is located outside the torus
        else
        {
            m_interactionInside = false;
        }

        // compute surface point
        double dist = vectTorusTool.length();
        if (dist > 0)
        {
            vectTorusTool.mul(1/dist);
        }
        vectTorusTool.mul(m_innerRadius);
        pointAxisTorus.addr(vectTorusTool, m_interactionPoint);
    }
    else
    {
        m_interactionInside = false;
        m_interactionPoint = a_toolPos;
    }
}


//==============================================================================
/*!
    This method updates the boundary box of this object.
*/
//==============================================================================
void cShapeTorus::updateBoundaryBox()
{
    double width = m_outerRadius + m_innerRadius;
    m_boundaryBoxMin.set(-width, -width,-m_innerRadius);
    m_boundaryBoxMax.set( width,  width, m_innerRadius);
}


//==============================================================================
/*!
    This method scales the size of this object with given scale factor.

    \param  a_scaleFactor  Scale factor.
*/
//==============================================================================
void cShapeTorus::scaleObject(const double& a_scaleFactor)
{
    // update dimensions
    m_outerRadius *= a_scaleFactor;
    m_innerRadius *= a_scaleFactor;

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
bool cShapeTorus::computeOtherCollisionDetection(cVector3d& a_segmentPointA,
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
    cVector3d collisionPoint;
    cVector3d collisionNormal;
    double collisionDistanceSq = 0.0;

    // compute collision detection between segment and line
    if (cIntersectionSegmentTorus(a_segmentPointA,
                                   a_segmentPointB,
                                   m_innerRadius,
                                   m_outerRadius,
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
