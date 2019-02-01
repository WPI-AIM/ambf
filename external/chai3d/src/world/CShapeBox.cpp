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
    POSSIBILITY OF SUCH DAMAGE. .

    \author    <http://www.chai3d.org>
    \author    Francois Conti
    \version   3.2.0 $Rev: 2174 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "world/CShapeBox.h"
//------------------------------------------------------------------------------
#include "shaders/CShaderProgram.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    Constructor of cShapeBox.

    \param  a_sizeX     Size of box along axis X.
    \param  a_sizeY     Size of box along axis Y.
    \param  a_sizeZ     Size of box along axis Z.
    \param  a_material  Material property to be applied to object.
*/
//==============================================================================
cShapeBox::cShapeBox(const double& a_sizeX, 
                     const double& a_sizeY, 
                     const double& a_sizeZ,
                     cMaterialPtr a_material)
{
    // enable display list
    m_useDisplayList = true;

    // initialize dimension
    setSize(a_sizeX, a_sizeY, a_sizeZ);

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
cShapeBox* cShapeBox::copy(const bool a_duplicateMaterialData,
                           const bool a_duplicateTextureData, 
                           const bool a_duplicateMeshData,
                           const bool a_buildCollisionDetector)
{
    // create new instance
    cShapeBox* obj = new cShapeBox(2.0 * m_hSizeX, 
                                   2.0 * m_hSizeY, 
                                   2.0 * m_hSizeZ);

    // copy generic properties
    copyShapeBoxProperties(obj, 
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
void cShapeBox::copyShapeBoxProperties(cShapeBox* a_obj,
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

    // copy properties of cShapeBox
    a_obj->m_hSizeX = m_hSizeX;
    a_obj->m_hSizeY = m_hSizeY;
    a_obj->m_hSizeZ = m_hSizeZ;
}


//==============================================================================
/*!
    This method sets the dimensions of the box along each axis.

    \param  a_sizeX  Size of box along axis X.
    \param  a_sizeY  Size of box along axis Y.
    \param  a_sizeZ  Size of box along axis Z.
*/
//==============================================================================
void cShapeBox::setSize(const double& a_sizeX, const double& a_sizeY, const double& a_sizeZ)
{
    // set dimensions
    m_hSizeX = 0.5 * fabs(a_sizeX);
    m_hSizeY = 0.5 * fabs(a_sizeY);
    m_hSizeZ = 0.5 * fabs(a_sizeZ);

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
void cShapeBox::render(cRenderOptions& a_options)
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


        if (!m_displayList.render(m_useDisplayList))
        {
            // create display list if requested
            m_displayList.begin(m_useDisplayList);

            // render box
            glBegin(GL_POLYGON);
                glNormal3d(1.0, 0.0, 0.0);
                glVertex3d(m_hSizeX, m_hSizeY, m_hSizeZ);
                glVertex3d(m_hSizeX,-m_hSizeY, m_hSizeZ);
                glVertex3d(m_hSizeX,-m_hSizeY,-m_hSizeZ);
                glVertex3d(m_hSizeX, m_hSizeY,-m_hSizeZ);
            glEnd();
        
            glBegin(GL_POLYGON);
                glNormal3d(-1.0, 0.0, 0.0);
                glVertex3d(-m_hSizeX, m_hSizeY,-m_hSizeZ);
                glVertex3d(-m_hSizeX,-m_hSizeY,-m_hSizeZ);
                glVertex3d(-m_hSizeX,-m_hSizeY, m_hSizeZ);
                glVertex3d(-m_hSizeX, m_hSizeY, m_hSizeZ);
            glEnd();

            glBegin(GL_POLYGON);
                glNormal3d(0.0, 1.0, 0.0);
                glVertex3d( m_hSizeX, m_hSizeY,-m_hSizeZ);
                glVertex3d(-m_hSizeX, m_hSizeY,-m_hSizeZ);
                glVertex3d(-m_hSizeX, m_hSizeY, m_hSizeZ);
                glVertex3d( m_hSizeX, m_hSizeY, m_hSizeZ);
            glEnd();
        
            glBegin(GL_POLYGON);
                glNormal3d(0.0,-1.0, 0.0);
                glVertex3d( m_hSizeX,-m_hSizeY, m_hSizeZ);
                glVertex3d(-m_hSizeX,-m_hSizeY, m_hSizeZ);
                glVertex3d(-m_hSizeX,-m_hSizeY,-m_hSizeZ);
                glVertex3d( m_hSizeX,-m_hSizeY,-m_hSizeZ);
            glEnd();

            glBegin(GL_POLYGON);
                glNormal3d(0.0, 0.0, 1.0);
                glVertex3d( m_hSizeX, m_hSizeY, m_hSizeZ);
                glVertex3d(-m_hSizeX, m_hSizeY, m_hSizeZ);
                glVertex3d(-m_hSizeX,-m_hSizeY, m_hSizeZ);
                glVertex3d( m_hSizeX,-m_hSizeY, m_hSizeZ);
            glEnd();
            glBegin(GL_POLYGON);
                glNormal3d(0.0, 0.0,-1.0);
                glVertex3d( m_hSizeX,-m_hSizeY,-m_hSizeZ);
                glVertex3d(-m_hSizeX,-m_hSizeY,-m_hSizeZ);
                glVertex3d(-m_hSizeX, m_hSizeY,-m_hSizeZ);
                glVertex3d( m_hSizeX, m_hSizeY,-m_hSizeZ);
            glEnd();

            // finalize display list
            m_displayList.end(true);
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
void cShapeBox::computeLocalInteraction(const cVector3d& a_toolPos,
                                          const cVector3d& a_toolVel,
                                          const unsigned int a_IDN)
{
    // temp variables
    bool inside;
    cVector3d projectedPoint;
    cVector3d normal(1,0,0);

    // sign
    double signX = cSign(a_toolPos(0) );
    double signY = cSign(a_toolPos(1) );
    double signZ = cSign(a_toolPos(2) );

    // check if tool is located inside box
    double tx = (signX * a_toolPos(0) );
    double ty = (signY * a_toolPos(1) );
    double tz = (signZ * a_toolPos(2) );
    
    inside = false;
    if (cContains(tx, 0.0, m_hSizeX))
    {
        if (cContains(ty, 0.0, m_hSizeY))
        {
            if (cContains(tz, 0.0, m_hSizeZ))
            {
                inside = true;
            }
        }
    }

    if (inside)
    {
        // tool is located inside box, compute distance from tool to surface
        double m_distanceX = m_hSizeX - (signX * a_toolPos(0) );
        double m_distanceY = m_hSizeY - (signY * a_toolPos(1) );
        double m_distanceZ = m_hSizeZ - (signZ * a_toolPos(2) );

        // search nearest surface
        if (m_distanceX < m_distanceY)
        {
            if (m_distanceX < m_distanceZ)
            {
                projectedPoint(0)  = signX * m_hSizeX;
                projectedPoint(1)  = a_toolPos(1) ;
                projectedPoint(2)  = a_toolPos(2) ;
                normal.set(signX * 1.0, 0.0, 0.0);
            }
            else
            {
                projectedPoint(0)  = a_toolPos(0) ;
                projectedPoint(1)  = a_toolPos(1) ;
                projectedPoint(2)  = signZ * m_hSizeZ;
                normal.set(0.0, 0.0, signZ * 1.0);
            }
        }
        else
        {
            if (m_distanceY < m_distanceZ)
            {
                projectedPoint(0)  = a_toolPos(0) ;
                projectedPoint(1)  = signY * m_hSizeY;
                projectedPoint(2)  = a_toolPos(2) ;
                normal.set(0.0, signY * 1.0, 0.0);
            }
            else
            {
                projectedPoint(0)  = a_toolPos(0) ;
                projectedPoint(1)  = a_toolPos(1) ;
                projectedPoint(2)  = signZ * m_hSizeZ;
                normal.set(0.0, 0.0, signZ * 1.0);
            }
        }
    }
    else
    {
        projectedPoint(0)  = cClamp(a_toolPos(0) , -m_hSizeX, m_hSizeX);
        projectedPoint(1)  = cClamp(a_toolPos(1) , -m_hSizeY, m_hSizeY);
        projectedPoint(2)  = cClamp(a_toolPos(2) , -m_hSizeZ, m_hSizeZ);
    }

    // return results
    m_interactionPoint = projectedPoint;

    cVector3d n = a_toolPos - projectedPoint;
    if (n.lengthsq() > 0.0)
    {
        m_interactionNormal = n;
        m_interactionNormal.normalize();
    }

    m_interactionInside = inside;
}


//==============================================================================
/*!
    This method updates the boundary box of this object.
*/
//==============================================================================
void cShapeBox::updateBoundaryBox()
{
    // compute half size lengths
    m_boundaryBoxMin.set(-m_hSizeX,-m_hSizeY,-m_hSizeZ);
    m_boundaryBoxMax.set( m_hSizeX, m_hSizeY, m_hSizeZ);
}


//==============================================================================
/*!
    This method scales the size of this object with given scale factor.

    \param  a_scaleFactor  Scale factor.
*/
//==============================================================================
void cShapeBox::scaleObject(const double& a_scaleFactor)
{
    // update dimensions
    m_hSizeX *= a_scaleFactor;
    m_hSizeY *= a_scaleFactor;
    m_hSizeZ *= a_scaleFactor;

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
bool cShapeBox::computeOtherCollisionDetection(cVector3d& a_segmentPointA,
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

    // compute distance between both point composing segment
    double collisionDistanceSq = 0.0;

    cVector3d min(-m_hSizeX,-m_hSizeY,-m_hSizeZ);
    cVector3d max( m_hSizeX, m_hSizeY, m_hSizeZ);

    // compute collision detection between segment and box
    if (cIntersectionSegmentBox(a_segmentPointA,
                                a_segmentPointB,
                                min,
                                max,
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
