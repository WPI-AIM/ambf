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
    \version   3.2.0 $Rev: 2181 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "world/CMultiMesh.h"
//------------------------------------------------------------------------------
#include "collisions/CGenericCollision.h"
#include "collisions/CCollisionBrute.h"
#include "collisions/CCollisionAABB.h"
#include "files/CFileModel3DS.h"
#include "files/CFileModelOBJ.h"
#include "files/CFileModelSTL.h"
#include "math/CMaths.h"
//------------------------------------------------------------------------------
#include <float.h>
#include <algorithm>
#include <set>
//------------------------------------------------------------------------------
using namespace std;
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    Constructor of cMultiMesh.
*/
//==============================================================================
cMultiMesh::cMultiMesh()
{
    // create array of mesh primitives
    m_meshes = new vector<cMesh*>;
}


//==============================================================================
/*!
    Destructor of cMultiMesh.
*/
//==============================================================================
cMultiMesh::~cMultiMesh()
{
    // delete all meshes
    deleteAllMeshes();

    // delete mesh vector
    delete m_meshes;
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
cMultiMesh* cMultiMesh::copy(const bool a_duplicateMaterialData,
                             const bool a_duplicateTextureData, 
                             const bool a_duplicateMeshData,
                             const bool a_buildCollisionDetector)
{
    // create multimesh object
    cMultiMesh* obj = new cMultiMesh();

    // copy multimesh properties
    copyMultiMeshProperties(obj,
        a_duplicateMaterialData,
        a_duplicateTextureData, 
        a_duplicateMeshData,
        a_buildCollisionDetector);

    // return result
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
void cMultiMesh::copyMultiMeshProperties(cMultiMesh* a_obj,
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

    // create properties of cMultiMesh
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        cMesh* mesh = (*it)->copy(a_duplicateMaterialData,
            a_duplicateTextureData, 
            a_duplicateMeshData,
            a_buildCollisionDetector);

        a_obj->addMesh(mesh);
    }
}


//==============================================================================
/*!
    This method enables or disables this object.\n

    When an object is disabled, haptic and graphic rendering are no longer 
    performed through the scenegraph and the object is simply ignored. Other 
    operations however will still be active. \n

    Enabling or disabling an object will not affect child objects, 
    unless explicitly specified.

    \param  a_enabled         If __true__ then object is enabled, __false__ otherwise.
    \param  a_affectChildren  If __true__, then children are updated too.
*/
//==============================================================================
void cMultiMesh::setEnabled(bool a_enabled,
                            const bool a_affectChildren)
{
    // updated meshes
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->setEnabled(a_enabled, true);
    }

    // update current object and possibly children
    cGenericObject::setEnabled(a_enabled, a_affectChildren);
}


//==============================================================================
/*!
    This method enables or disables the object to be felt haptically. \n

    If argument \p a_affectChildren is set to __true__ then all children are
    updated with the new value.

    \param  a_hapticEnabled   If __true__ then the object can be touched when visible.
    \param  a_affectChildren  If __true__ then all children are updated too.
*/
//==============================================================================
void cMultiMesh::setHapticEnabled(const bool a_hapticEnabled, 
                                  const bool a_affectChildren)
{
    // update current object and possibly children
    cGenericObject::setHapticEnabled(a_hapticEnabled, a_affectChildren);

    // updated meshes
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->setHapticEnabled(a_hapticEnabled, true);
    }
}


//==============================================================================
/*!
     This method sets the haptic stiffness for this object, optionally recursively 
     affecting children.

     \param  a_stiffness       The stiffness to apply to this object.
     \param  a_affectChildren  If __true__, then children are updated too.
*/
//==============================================================================
void cMultiMesh::setStiffness(const double a_stiffness, 
                              const bool a_affectChildren)
{
    // update current object and possibly children
    cGenericObject::setStiffness(a_stiffness, a_affectChildren);

    // updated meshes
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->setStiffness(a_stiffness, true);
    }
}


//==============================================================================
/*!
    This method sets the static and dynamic friction properties for this object,
    optionally recursively affecting children.

    \param  a_staticFriction   The static friction to apply to this object.
    \param  a_dynamicFriction  The dynamic friction to apply to this object.
    \param  a_affectChildren   If __true__ then children are updated too.
*/
//==============================================================================
void cMultiMesh::setFriction(double a_staticFriction, 
                             double a_dynamicFriction, 
                             const bool a_affectChildren)
{
    // update current object and possibly children
    cGenericObject::setFriction(a_staticFriction, a_dynamicFriction, a_affectChildren);

    // updated meshes
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->setFriction(a_staticFriction, a_dynamicFriction, true);
    }
}


//==============================================================================
/*!
    This method graphically shows or hides this object, optionally recursively 
    affecting children.

    \param  a_show            If __true__ then object is visible.
    \param  a_affectChildren  If __true__ then children are updated too.
*/
//==============================================================================
void cMultiMesh::setShowEnabled(const bool a_show, 
                                const bool a_affectChildren)
{
    // update current object and possibly children
    cGenericObject::setShowEnabled(a_show, a_affectChildren);

    // updated meshes
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->setShowEnabled(a_show, true);
    }
}


//==============================================================================
/*!
    This method specifies whether transparency is enabled. If transparency is 
    enabled then make sure that multi-pass rendering is enabled too. For more 
    information, see class \ref cCamera.

    \param  a_useTransparency  If __true__ then transparency is enabled.
    \param  a_affectChildren   If __true__ then children are updated too.
*/
//==============================================================================
void cMultiMesh::setUseTransparency(const bool a_useTransparency,
                                    const bool a_affectChildren)
{
    // update current object and possibly children
    cGenericObject::setUseTransparency(a_useTransparency, a_affectChildren);

    // updated meshes
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->setUseTransparency(a_useTransparency, true);
    }
}


//==============================================================================
/*!
    This method sets the alpha value to all components of the object,
    optionally propagating the operation to children. \n

    Using the 'apply to textures' option causes the actual texture
    alpha values to be over-written in my texture, if it exists. \n

    \param  a_level            Level of transparency ranging from 0.0 to 1.0.
    \param  a_applyToVertices  If __true__, then apply changes to vertex colors.
    \param  a_applyToTextures  If __true__, then apply changes to texture.
    \param  a_affectChildren   If __true__, then children are updated too.
*/
//==============================================================================
void cMultiMesh::setTransparencyLevel(const float a_level,
    const bool a_applyToVertices,
    const bool a_applyToTextures,
    const bool a_affectChildren)
{
    // update current object and possibly children
    cGenericObject::setTransparencyLevel(a_level, a_applyToVertices, a_applyToTextures, a_affectChildren);

    // updated meshes
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->setTransparencyLevel(a_level,
                                    a_applyToVertices,
                                    a_applyToTextures,
                                    true);
    }
}


//==============================================================================
/*!
    This method enables or disables wireframe rendering, optionally propagating
    the operation to children.

    \param  a_showWireMode    If __true__ then wireframe mode is used.
    \param  a_affectChildren  If __true__ then children are updated too.
*/
//==============================================================================
void cMultiMesh::setWireMode(const bool a_showWireMode, 
                             const bool a_affectChildren)
{
    // update current object and possibly children
    cGenericObject::setWireMode(a_showWireMode, a_affectChildren);

    // updated meshes
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->setWireMode(a_showWireMode, true);
    }
}


//==============================================================================
/*!
    This method enables or disables back face culling. \n
    Rendering in OpenGL is much faster with culling enabled.

    \param  a_useCulling      If __true__ then back faces are culled.
    \param  a_affectChildren  If __true__ then then children are updated too.
*/
//==============================================================================
void cMultiMesh::setUseCulling(const bool a_useCulling, 
                               const bool a_affectChildren)
{
    // update current object and possibly children
    cGenericObject::setUseCulling(a_useCulling, a_affectChildren);

    // updated meshes
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->setUseCulling(a_useCulling, true);
    }
}


//==============================================================================
/*!
    This method enables or disables the use of per-vertex color information of
    when rendering the mesh.

    \param  a_useColors       If __true__ then then vertex color information is 
                              applied.
    \param  a_affectChildren  If __true__ then children are updated too.
*/
//==============================================================================
void cMultiMesh::setUseVertexColors(const bool a_useColors, 
                                    const bool a_affectChildren)
{
    // update current object and possibly children
    cGenericObject::setUseVertexColors(a_useColors, a_affectChildren);

    // updated meshes
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->setUseVertexColors(a_useColors, true);
    }
}


//==============================================================================
/*!
    This method creates a backup of the material color properties of this object,
    optionally recursively affecting children.

    \param  a_affectChildren  If __true__ then children are updated too.
*/
//==============================================================================
void cMultiMesh::backupMaterialColors(const bool a_affectChildren)
{
    // update current object and possibly children
    cGenericObject::backupMaterialColors(a_affectChildren);

    // updated meshes
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->backupMaterialColors(true);
    }
}

 
//==============================================================================
/*!
    This method restores material color properties for this object, optionally 
    recursively affecting children.

    \param  a_affectChildren  If __true__ then children are updated too.
*/
//==============================================================================
void cMultiMesh::restoreMaterialColors(const bool a_affectChildren)
{
    // update current object and possibly children
    cGenericObject::restoreMaterialColors(a_affectChildren);

    // updated meshes
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->restoreMaterialColors(true);
    }
}


//==============================================================================
/*!
    This method enables the use of display lists for mesh rendering. 
    Display lists significantly speed up rendering for large meshes, but it 
    means that any changes that are made on the object (e.g changing vertex 
    positions) will not take effect until you invalidate the existing display list
    by calling \ref markForUpdate().

    \param  a_useDisplayList  If __true__ then a display list is created (cMesh).
    \param  a_affectChildren  If __true__ then children are updated too.
*/
//==============================================================================
void cMultiMesh::setUseDisplayList(const bool a_useDisplayList,
                                   const bool a_affectChildren)
{
    // update current object and possibly children
    cGenericObject::setUseDisplayList(a_useDisplayList, a_affectChildren);

    // updated meshes
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->setUseDisplayList(a_useDisplayList, true);
    }
}


//==============================================================================
/*!
    This method invalidates any existing display lists. You should call this on 
    if you're using display lists and you modify mesh options, vertex positions,
    etc.

    \param  a_affectChildren  If __true__ then children are updated too.
*/
//==============================================================================
void cMultiMesh::markForUpdate(const bool a_affectChildren)
{
     // update current object and possibly children
    cGenericObject::markForUpdate(a_affectChildren);

    // updated meshes
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->markForUpdate(a_affectChildren);
    }
}


//==============================================================================
/*!
     This method enables or disables the use of material properties.

     \param  a_useMaterial     If __true__ then material properties are used for rendering.
     \param  a_affectChildren  If __true__ then children are updated too.
*/
//==============================================================================
void cMultiMesh::setUseMaterial(const bool a_useMaterial, 
                                const bool a_affectChildren)
{
    // update current object and possibly children
    cGenericObject::setUseMaterial(a_useMaterial, a_affectChildren);

    // updated meshes
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->setUseMaterial(a_useMaterial, true);
    }
}


//==============================================================================
/*!
    This method copies all material properties defined in \p a_material to the 
    material structure of this object.\n

    Note that this does not affect whether material rendering is enabled;
    it sets the material that will be rendered _if_ material rendering is
    enabled.  Call method \ref setUseMaterial() to enable or disable material
    rendering.

    \param  a_material        The material to apply to this object.
    \param  a_affectChildren  If __true__ then children are updated too.
*/
//==============================================================================
void cMultiMesh::setMaterial(cMaterialPtr a_material,
                             const bool a_affectChildren)
{
    // update current object and possibly children
    cGenericObject::setMaterial(a_material, a_affectChildren);

    // updated meshes
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->setMaterial(a_material, true);
    }
}


//==============================================================================
/*!
    This method copies all material properties defined in \p a_material to the 
    material structure of this object.\n

    Note that this does not affect whether material rendering is enabled;
    it sets the material that will be rendered _if_ material rendering is
    enabled.  Call method \ref setUseMaterial() to enable or disable material
    rendering.

    \param  a_material        The material to apply to this object
    \param  a_affectChildren  If __true__ then children are updated too.
*/
//==============================================================================
void cMultiMesh::setMaterial(cMaterial& a_material,
                             const bool a_affectChildren)
{
    // update current object and possibly children
    cGenericObject::setMaterial(a_material, a_affectChildren);

    // updated meshes
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->setMaterial(a_material, true);
    }
}


//==============================================================================
/*!
    This method enables or disables texture-mapping, optionally recursively
    affecting children.

    \param  a_useTexture      If __true__ then texture mapping is used.
    \param  a_affectChildren  If __true__ then children are updated too.
*/
//==============================================================================
void cMultiMesh::setUseTexture(const bool a_useTexture, 
                               const bool a_affectChildren)
{
    // update current object and possibly children
    cGenericObject::setUseTexture(a_useTexture, a_affectChildren);

    // updated meshes
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->setUseTexture(a_useTexture, true);
    }
}


//==============================================================================
/*!
    This method sets the current texture for this mesh, optionally recursively
    affecting children. \n

    Note that this does not affect whether texturing is enabled; it sets
    the texture that will be rendered _if_ texturing is enabled.  Call
    method \ref setUseTexture() to enable or disable texturing.

    \param  a_texture         The texture to apply to this object.
    \param  a_affectChildren  If __true__ then children are updated too.
*/
//==============================================================================
void cMultiMesh::setTexture(cTexture1dPtr a_texture,
                            const bool a_affectChildren)
{
    // update current object and possibly children
    cGenericObject::setTexture(a_texture, a_affectChildren);

    // updated meshes
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->setTexture(a_texture, true);
    }
}


//==============================================================================
/*!
    This method assigns a shader program to this object. \n

    If \p a_affectChildren is set to __true__ then all children are assigned
    with the shader program.

    \param  a_shaderProgram   Shader program to be assigned to object.
    \param  a_affectChildren  If __true__ then children are updated too.
*/
//==============================================================================
void cMultiMesh::setShaderProgram(cShaderProgramPtr a_shaderProgram,
                                  const bool a_affectChildren)
{
    m_shaderProgram = a_shaderProgram;

    // updated meshes
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->setShaderProgram(a_shaderProgram, true);
    }

    // apply change to children
    if (a_affectChildren)
    {
        vector<cGenericObject*>::iterator it;
        for (it = m_children.begin(); it < m_children.end(); it++)
        {
            (*it)->setShaderProgram(a_shaderProgram, true);
        }
    }
}


//==============================================================================
/*!
    This method enables or disables the graphic representation of the 
    boundary box of this object. \n

    If \p a_affectChildren is set to __true__ then all children are updated
    with the new value.

    \param  a_showBoundaryBox  If __true__ boundary box is displayed.
    \param  a_affectChildren   If __true__ then children are updated too.
*/
//==============================================================================
void cMultiMesh::setShowBoundaryBox(const bool a_showBoundaryBox, 
                                    const bool a_affectChildren)
{
    // update current object and possibly children
    cGenericObject::setShowBoundaryBox(a_showBoundaryBox, a_affectChildren);

    // updated meshes
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->setShowBoundaryBox(a_showBoundaryBox, true);
    }
}


//==============================================================================
/*!
    This method compute the axis-aligned boundary box that encloses all
    triangles in this mesh.
*/
//==============================================================================
void cMultiMesh::updateBoundaryBox()
{
    // initialization
    bool empty = true;
    cVector3d minBox, maxBox;
    minBox.set( C_LARGE, C_LARGE, C_LARGE);
    maxBox.set(-C_LARGE,-C_LARGE,-C_LARGE);

    // compute the bounding box of all my children
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        cGenericObject* object = (*it);

        // compute boundary box of child
        object->computeBoundaryBox(true);

        // if boundary box is not empty, then include update
        if (!object->getBoundaryBoxEmpty())
        {
            // since child is not empty, parent is no longer empty too.
            empty = false;

            // retrieve position and orientation of child.
            cVector3d pos = object->getLocalPos();
            cMatrix3d rot = object->getLocalRot();

            // compute position of corners of child within parent reference frame
            double xMin = object->getBoundaryMin().x();
            double yMin = object->getBoundaryMin().y();
            double zMin = object->getBoundaryMin().z();
            double xMax = object->getBoundaryMax().x();
            double yMax = object->getBoundaryMax().y();
            double zMax = object->getBoundaryMax().z();

            cVector3d corners[8];
            corners[0] = pos + rot * cVector3d(xMin, yMin, zMin);
            corners[1] = pos + rot * cVector3d(xMin, yMin, zMax);
            corners[2] = pos + rot * cVector3d(xMin, yMax, zMin);
            corners[3] = pos + rot * cVector3d(xMin, yMax, zMax);
            corners[4] = pos + rot * cVector3d(xMax, yMin, zMin);
            corners[5] = pos + rot * cVector3d(xMax, yMin, zMax);
            corners[6] = pos + rot * cVector3d(xMax, yMax, zMin);
            corners[7] = pos + rot * cVector3d(xMax, yMax, zMax);

            // update boundary box by taking into account child boundary box
            for (int i=0; i<8; i++)
            {
                minBox(0) = cMin(corners[i](0),  minBox(0));
                minBox(1) = cMin(corners[i](1),  minBox(1));
                minBox(2) = cMin(corners[i](2),  minBox(2));
                maxBox(0) = cMax(corners[i](0),  maxBox(0));
                maxBox(1) = cMax(corners[i](1),  maxBox(1));
                maxBox(2) = cMax(corners[i](2),  maxBox(2));
            }
        }
    }

    if (empty)
    {
        m_boundaryBoxMin.zero();
        m_boundaryBoxMax.zero();
        m_boundaryBoxEmpty = true;
    }
    else
    {
        m_boundaryBoxMin = minBox;
        m_boundaryBoxMax = maxBox;
        m_boundaryBoxEmpty = false;
    }
}


//==============================================================================
/*!
    This method deletes any existing collision detector and sets the current
    collision detector to __null__. \n

    It's fine for an object to have a null collision detector (that's the
    default for a new object, in fact), it just means that no collisions 
    will be found.

    \param  a_affectChildren  If __true__ then children are updated too.
*/
//==============================================================================
void cMultiMesh::deleteCollisionDetector(const bool a_affectChildren)
{
    // update current object and possibly children
    cGenericObject::deleteCollisionDetector(a_affectChildren);

    // updated meshes
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->deleteCollisionDetector(true);
    }
}


//==============================================================================
/*!
    This method determines whether a given segment intersects this object or any
    of its descendants.\n
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

    \return __true__ if one or more collisions have occurred, __false__ otherwise.
*/
//==============================================================================
bool cMultiMesh::computeCollisionDetection(const cVector3d& a_segmentPointA,
                                           const cVector3d& a_segmentPointB,
                                           cCollisionRecorder& a_recorder,
                                           cCollisionSettings& a_settings)
{
    ///////////////////////////////////////////////////////////////////////////
    // INITIALIZATION
    ///////////////////////////////////////////////////////////////////////////

    // check if node is a ghost. If yes, then ignore call
    if (m_ghostEnabled) { return (false); }

    // temp variable
    bool hit = false;

    // get the transpose of the local rotation matrix
    cMatrix3d transLocalRot;
    m_localRot.transr(transLocalRot);

    // convert first endpoint of the segment into local coordinate frame
    cVector3d localSegmentPointA = a_segmentPointA;
    localSegmentPointA.sub(m_localPos);
    transLocalRot.mul(localSegmentPointA);

    // convert second endpoint of the segment into local coordinate frame
    cVector3d localSegmentPointB = a_segmentPointB;
    localSegmentPointB.sub(m_localPos);
    transLocalRot.mul(localSegmentPointB);


    ///////////////////////////////////////////////////////////////////////////
    // MOTION ADJUSTMENT
    ///////////////////////////////////////////////////////////////////////////

    // adjust the first segment endpoint so that it is in the same position
    // relative to the moving object as it was at the previous haptic iteration
    cVector3d localSegmentPointAadjusted;
    if (a_settings.m_adjustObjectMotion)
    {
        adjustCollisionSegment(localSegmentPointA, localSegmentPointAadjusted);
    }
    else
    {
        localSegmentPointAadjusted = localSegmentPointA;
    }



    ///////////////////////////////////////////////////////////////////////////
    // CHECK COLLISIONS
    ///////////////////////////////////////////////////////////////////////////

    // check for a collision with this object if:
    // (1) it has a collision detector
    // (2) if other settings (visible and haptic enabled) are activated
    if ((m_enabled) &&
        ((a_settings.m_checkVisibleObjects && m_showEnabled) ||
         (a_settings.m_checkHapticObjects && m_hapticEnabled)))
    {
        if (m_collisionDetector != NULL)
        {
            // call the collision detector's collision detection function
            if (m_collisionDetector->computeCollision(this,
                                                      localSegmentPointAadjusted,
                                                      localSegmentPointB,
                                                      a_recorder,
                                                      a_settings))
            {
                // record that there has been a collision
                hit = true;
            }
        }

        // compute any other collisions. This is a virtual function that can be extended for
        // classes that may contain other objects (sibling) for which collision detection may
        // need to be computed.
        hit = hit || computeOtherCollisionDetection(localSegmentPointAadjusted,
                                                    localSegmentPointB,
                                                    a_recorder,
                                                    a_settings);
    }

    ///////////////////////////////////////////////////////////////////////////
    // CHECK MESHES
    ///////////////////////////////////////////////////////////////////////////

    // check for collisions with all meshes of this object
    for (unsigned int i=0; i<m_meshes->size(); i++)
    {
        // call this child's collision detection function to see if it (or any
        // of its descendants) are intersected by the segment
        bool hitMesh = m_meshes->at(i)->computeCollisionDetection(localSegmentPointA,
                                                                  localSegmentPointB,
                                                                  a_recorder,
                                                                  a_settings);

        // update if a hit occurred
        hit = hit | hitMesh;
    }



    ///////////////////////////////////////////////////////////////////////////
    // CHECK CHILDREN
    ///////////////////////////////////////////////////////////////////////////

    // check for collisions with all children of this object
    for (unsigned int i=0; i<m_children.size(); i++)
    {
        // call this child's collision detection function to see if it (or any
        // of its descendants) are intersected by the segment
        bool hitChild = m_children[i]->computeCollisionDetection(localSegmentPointA,
                                                                 localSegmentPointB,
                                                                 a_recorder,
                                                                 a_settings);

        // update if a hit occurred
        hit = hit | hitChild;
    }


    ///////////////////////////////////////////////////////////////////////////
    // FINALIZE
    ///////////////////////////////////////////////////////////////////////////

    // return whether there was a collision between the segment and this world
    return (hit);
}


//==============================================================================
/*!
    This method enables or disables graphic representation of the collision 
    detector at this node. \n

    If argument \p a_affectChildren is set to __true__ then all children are
    updated with the new value.

    \param  a_showCollisionDetector  If __true__ then display collision detector graphically.
    \param  a_affectChildren         If __true__ then children are updated too.
*/
//==============================================================================
void cMultiMesh::setShowCollisionDetector(const bool a_showCollisionDetector, 
                                          const bool a_affectChildren)
{
    // update current object and possibly children
    cGenericObject::setShowCollisionDetector(a_showCollisionDetector, a_affectChildren);

    // updated meshes
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->setShowCollisionDetector(a_showCollisionDetector, true);
    }
}


//==============================================================================
/*!
    This method sets the rendering properties of the the graphic representation 
    of the collision detector. \n

    If argument \p a_affectChildren is set to __true__ then all children are
    updated with the new values.

    \param  a_color           Color used to render collision detector.
    \param  a_displayDepth    Indicated which depth of collision tree needs to be displayed
                              (see cGenericCollision).
    \param  a_affectChildren  If __true__ then children are updated too.
*/
//==============================================================================
void cMultiMesh::setCollisionDetectorProperties(unsigned int a_displayDepth,
                                                cColorf& a_color, 
                                                const bool a_affectChildren)
{
    // update current object and possibly children
    cGenericObject::setCollisionDetectorProperties(a_displayDepth,
                                                   a_color, 
                                                   a_affectChildren);

    // updated meshes
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->setCollisionDetectorProperties(a_displayDepth,
                                              a_color, 
                                              true);
    }
}


//==============================================================================
/*!
    This method performs a uniform scale on the object, optionally including 
    children.

    \param  a_scaleFactor     Scale factor.
    \param  a_affectChildren  If __true__ then children are updated too.
*/
//==============================================================================
void cMultiMesh::scale(const double& a_scaleFactor, 
                       const bool a_affectChildren)
{
    // update current object and possibly children
    cGenericObject::scale(a_scaleFactor, a_affectChildren);

    // updated meshes
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->m_localPos.mul(a_scaleFactor);
        (*it)->scale(a_scaleFactor, true);
    }
}


//==============================================================================
/*!
    This method scales this mesh by using different scale factors along X, Y, 
    and Z axes.

    \param  a_scaleX  Scale factor along X axis.
    \param  a_scaleY  Scale factor along Y axis.
    \param  a_scaleZ  Scale factor along Z axis.
*/
//==============================================================================
void cMultiMesh::scaleXYZ(const double a_scaleX, const double a_scaleY, const double a_scaleZ)
{
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        cMatrix3d a_R_b = (*it)->getLocalRot();
        cMatrix3d b_R_a = cTranspose(a_R_b);

        // scale vertices
        unsigned int numVertices = (*it)->m_vertices->getNumElements();
        for (unsigned int i=0; i<numVertices; i++)
        {
            cVector3d b_Vertex =  (*it)->m_vertices->getLocalPos(i);
            cVector3d a_Vertex = a_R_b *  b_Vertex;
            a_Vertex = a_R_b * b_Vertex;
            a_Vertex.mul(a_scaleX, a_scaleY, a_scaleZ);
            b_Vertex = b_R_a * a_Vertex;
            (*it)->m_vertices->setLocalPos(i, b_Vertex);
        }

        // scale position
        (*it)->m_localPos.mul(a_scaleX, a_scaleY, a_scaleZ);

        // update boundary box
        cVector3d b_BoxMin = (*it)->m_boundaryBoxMin;
        cVector3d a_BoxMin = a_R_b * b_BoxMin;
        a_BoxMin.mul(a_scaleX, a_scaleY, a_scaleZ);
        b_BoxMin = b_R_a * a_BoxMin;
        (*it)->m_boundaryBoxMin = b_BoxMin;

        cVector3d b_BoxMax = (*it)->m_boundaryBoxMax;
        cVector3d a_BoxMax = a_R_b * b_BoxMax;
        a_BoxMax.mul(a_scaleX, a_scaleY, a_scaleZ);
        b_BoxMax = b_R_a * a_BoxMax;
        (*it)->m_boundaryBoxMax = b_BoxMax;
    }

    // update boundary box
    m_boundaryBoxMax.mul(a_scaleX, a_scaleY, a_scaleZ);
    m_boundaryBoxMin.mul(a_scaleX, a_scaleY, a_scaleZ);
}


//==============================================================================
/*!
    This method creates a new mesh and adds it to the list of meshes.

    \return Pointer to new mesh object.
*/
//==============================================================================
cMesh* cMultiMesh::newMesh()
{
    // create new mesh entity
    cMesh* obj = new cMesh();

    // set parent and owner
    obj->setParent(this);
    obj->setOwner(this);

    // add mesh to list
    m_meshes->push_back(obj);

    // return result
    return (obj);
}


//==============================================================================
/*!
    This method adds an existing mesh primitives to list of meshes

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cMultiMesh::addMesh(cMesh* a_mesh)
{
    // sanity check
    if (a_mesh == NULL) { return (false); }
    if (a_mesh->getParent() != NULL) { return (false); }

    // set parent and owner
    a_mesh->setParent(this);
    a_mesh->setOwner(this);

    // add mesh to list
    m_meshes->push_back(a_mesh);

    // return success
    return (true);
}


//==============================================================================
/*!
    This method removes a mesh primitive from the list of meshes.

    \param  a_mesh  Mesh to remove.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cMultiMesh::removeMesh(cMesh* a_mesh)
{
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        if ((*it) == a_mesh)
        {
            // parent is set to NULL. Mesh becomes its own owwner.
            a_mesh->setParent(NULL);
            a_mesh->setOwner(a_mesh);

            // remove mesh from list of meshes
            m_meshes->erase(it);

            // return success
            return (true);
        }
    }
    return (false);
}


//==============================================================================
/*!
    This method removes all mesh primitives.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cMultiMesh::removeAllMesh()
{
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        // parent is set to NULL. Mesh becomes its own owner.
        (*it)->setParent(NULL);
        (*it)->setOwner(*it);
    }

    // clear list of meshes
    m_meshes->clear();

    // success
    return (true);
}


//==============================================================================
/*!
    This method deletes a mesh primitive from the list of meshes.

    \param  a_mesh  Mesh to delete.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cMultiMesh::deleteMesh(cMesh* a_mesh)
{
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        if ((*it) == a_mesh)
        {
            // remove mesh from list of meshes
            m_meshes->erase(it);

            // delete mesh
            delete (*it);

            // return success
            return (true);
        }
    }

    // operation failed, mesh was not found
    return (false);
}


//==============================================================================
/*!
    This method deletes all meshes.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cMultiMesh::deleteAllMeshes()
{
    // delete all meshes
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        cMesh* nextMesh = (*it);
        delete nextMesh;
    }

    // clear all meshes from list
    m_meshes->clear();

    // success
    return (true);
}


//==============================================================================
/*!
    This method retrieves the number of meshes that compose this multi-mesh object.

    \return Number of meshes contained in this object.
*/
//==============================================================================
int cMultiMesh::getNumMeshes()
{
    unsigned int numMeshes = (unsigned int)(m_meshes->size());
    return (numMeshes);
}


//==============================================================================
/*!
    This method returns a pointer to a mesh primitive by passing its index number.

    \param  a_index  Index number of mesh.

    \return Pointer to selected mesh.
*/
//==============================================================================
cMesh* cMultiMesh::getMesh(unsigned  int a_index)
{
    if (a_index < m_meshes->size())
    {
        return (m_meshes->at(a_index));
    }
    else
    {
        return (NULL);
    }
}


//==============================================================================
/*!
    This method returns the index number and mesh of a specific triangle that
    is part of this multi-mesh.

    \param  a_index          Index number of the requested triangle.
    \param  a_mesh           Pointer to the mesh containing the selected triangle.
    \param  a_triangleIndex  Index number of the specified triangle inside the mesh triangle array.
*/
//==============================================================================
bool cMultiMesh::getTriangle(const unsigned int a_index, cMesh*& a_mesh, unsigned int& a_triangleIndex)
{
    unsigned int index = a_index;
    unsigned int i, numMeshes;
    numMeshes = (unsigned int)(m_meshes->size());
    for (i=0; i<numMeshes; i++)
    {
        cMesh* nextMesh = m_meshes->at(i);
        if (nextMesh)
        {
             unsigned int numTriangles = nextMesh->getNumTriangles();

             if (index < numTriangles)
             {
                a_triangleIndex = index;
                a_mesh = nextMesh;
                return (true);
             }
             else 
             {
                index -= numTriangles;
             }
        }
    }

    a_mesh = NULL;
    a_triangleIndex = 0;
    return (false);
}


//==============================================================================
/*!
    This method returns the index number and mesh of a specific vertex that is 
    part of this multi-mesh.

    \param  a_index        Index number of the requested vertex.
    \param  a_mesh         Pointer to the mesh containing the selected vertex.
    \param  a_vertexIndex  Index number of the specified vertex inside the mesh vertex array.
*/
//==============================================================================
bool cMultiMesh::getVertex(const unsigned int a_index, cMesh*& a_mesh, unsigned int& a_vertexIndex)
{
    unsigned int index = a_index;
    unsigned int i, numMeshes;
    numMeshes = (unsigned int)(m_meshes->size());
    for (i=0; i<numMeshes; i++)
    {
        cMesh* nextMesh = m_meshes->at(i);
        if (nextMesh)
        {
             unsigned int numVertices = nextMesh->getNumVertices();

             if (index < numVertices)
             {
                a_vertexIndex = index;
                a_mesh = nextMesh;
                return (true);
             }
             else 
             {
                index -= numVertices;
             }
        }
    }

    a_mesh = NULL;
    a_vertexIndex = 0;
    return (false);
}


//==============================================================================
/*!
    This method returns the the number of stored triangles.

    \return Number of triangles.
*/
//==============================================================================
unsigned int cMultiMesh::getNumTriangles() const
{
    int numTriangles = 0;

    // count triangles of all meshes that compose this multi-mesh
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        numTriangles = numTriangles + (*it)->getNumTriangles();
    }

    return (numTriangles);
}


//==============================================================================
/*!
    This method returns the position data of specific vertex.

    \param  a_index  Index of the requested vertex.

    \return Local position value at vertex.
*/
//==============================================================================
cVector3d cMultiMesh::getVertexPos(unsigned int a_index)
    
{
    // sanity check
    if (a_index >= getNumVertices()) return (NULL);

    // retrieve triangle
    unsigned int i, numMeshes;
    numMeshes = (unsigned int)(m_meshes->size());
    for (i=0; i<numMeshes; i++)
    {
        cMesh* nextMesh = m_meshes->at(i);
        if (nextMesh)
        {
             unsigned int numVertices = nextMesh->getNumVertices();

             if (a_index < numVertices)
             {
                return (nextMesh->m_vertices->getLocalPos(a_index));
             }
             else 
             {
                a_index -= numVertices;
             }
        }
    }

    return (cVector3d(0,0,0));
}


//==============================================================================
/*!
    This method returns the the number of stored vertices.

    \return Number of vertices.
*/
//==============================================================================
unsigned int cMultiMesh::getNumVertices() const
{
    int numVertices = 0;

    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        numVertices = numVertices + (*it)->getNumVertices();
    }

    return (numVertices);
}


//==============================================================================
/*!
    This method clears all triangles and vertices of multi-mesh.
*/
//==============================================================================
void cMultiMesh::clear()
{
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->clear();
    }
}


//==============================================================================
/*!
    This method converts this multimesh into a single mesh object. 
    Material and texture properties are not copied.
*/
//==============================================================================
void cMultiMesh::convertToSingleMesh(cMesh* a_mesh)
{
    // clear all previous data
    a_mesh->clear();

    // store vertices for each mesh
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        cMesh* nextMesh = (*it);
        cVector3d posMesh = nextMesh->getLocalPos();
        cMatrix3d rotMesh = nextMesh->getLocalRot();

        int numVertices = nextMesh->getNumVertices();

        for (int i=0; i<numVertices; i++)
        {
            cVector3d pos = posMesh + rotMesh * nextMesh->m_vertices->getLocalPos(i);
            cVector3d normal = rotMesh * nextMesh->m_vertices->getNormal(i);
            cVector3d texCoord = nextMesh->m_vertices->getTexCoord(i);
            cColorf   color = nextMesh->m_vertices->getColor(i);

            a_mesh->newVertex(pos, normal, texCoord, color);
        }
    }

    // store triangles for each mesh
    int vertexOffset = 0;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        cMesh* nextMesh = (*it);

        int numTriangles = nextMesh->getNumTriangles();

        for (int i=0; i<numTriangles; i++)
        {
            unsigned int vertex0 = nextMesh->m_triangles->getVertexIndex0(i) + vertexOffset;
            unsigned int vertex1 = nextMesh->m_triangles->getVertexIndex1(i) + vertexOffset;
            unsigned int vertex2 = nextMesh->m_triangles->getVertexIndex2(i) + vertexOffset;

            a_mesh->newTriangle(vertex0, vertex1, vertex2);
        }

        vertexOffset = vertexOffset + nextMesh->getNumVertices();
    }
}



//==============================================================================
/*!
    This method loads a 3D mesh file. \n
    CHAI3D currently supports .obj, .3ds, and .stl files.

    \param  a_filename  Filename of 3D model.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cMultiMesh::loadFromFile(string a_filename)
{ 
    // find extension
    string extension = cGetFileExtension(a_filename);

    // we need a file extension to figure out file type
    if (extension.length() == 0)
    {
        return (false);
    }

    // convert string to lower extension
    string fileType = cStrToLower(extension);

    // result for loading file
    bool result = false;

    //--------------------------------------------------------------------
    // .3DS FORMAT
    //--------------------------------------------------------------------
    if (fileType == "3ds")
    {
        result = cLoadFile3DS(this, a_filename);
    }

    //--------------------------------------------------------------------
    // .OBJ FORMAT
    //--------------------------------------------------------------------
    else if (fileType == "obj")
    {
        result = cLoadFileOBJ(this, a_filename);
    }

    //--------------------------------------------------------------------
    // .STL FORMAT
    //--------------------------------------------------------------------
    else if (fileType == "stl")
    {
        result = cLoadFileSTL(this, a_filename);
    }

    return (result);
}


//==============================================================================
/*!
    This method saves a mesh object to file. \n
    CHAI3D currently supports .obj, .3ds, and .stl files.

    \param  a_filename  Filename of 3D model.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cMultiMesh::saveToFile(std::string a_filename)
{ 
    // find extension
    string extension = cGetFileExtension(a_filename);

    // we need a file extension to figure out file type
    if (extension.length() == 0)
    {
        return (false);
    }

    // convert string to lower extension
    string fileType = cStrToLower(extension);

    // result for loading file
    bool result = false;

    //--------------------------------------------------------------------
    // .3DS FORMAT
    //--------------------------------------------------------------------
    if (fileType == "3ds")
    {
        result = cSaveFile3DS(this, a_filename);
    }

    //--------------------------------------------------------------------
    // .OBJ FORMAT
    //--------------------------------------------------------------------
    else if (fileType == "obj")
    {
        result = cSaveFileOBJ(this, a_filename);
    }

    //--------------------------------------------------------------------
    // .STL FORMAT
    //--------------------------------------------------------------------
    else if (fileType == "stl")
    {
        result = cSaveFileSTL(this, a_filename);
    }

    return (result);
}


//==============================================================================
/*!
    This method computes all triangle normals.
*/
//==============================================================================
void cMultiMesh::computeAllNormals()
{
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->computeAllNormals();
    }
}


//==============================================================================
/*!
    This method computes the normal matrix vectors for all triangles.
*/
//==============================================================================
void cMultiMesh::computeBTN()
{
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->computeBTN();
    }
}



//==============================================================================
/*!
    This method enables or disables the rendering of tangents and bi-tangents.

    \param  a_showTangents  Display mode.
*/
//==============================================================================
void cMultiMesh::setShowTangents(const bool a_showTangents)
{
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->setShowTangents(a_showTangents);
    }
}


//==============================================================================
/*!
    This method enables or disables the rendering of triangles.

    \param  a_showTriangles  If __true__ then triangles are rendered, __false__ otherwise.
*/
//==============================================================================
void cMultiMesh::setShowTriangles(const bool a_showTriangles)
{
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->setShowTriangles(a_showTriangles);
    }
}


//==============================================================================
/*!
    This method enables or disables the rendering of edges.

    \param  a_showEdges  If __true__ then edges are rendered, __false__ otherwise.
*/
//==============================================================================
void cMultiMesh::setShowEdges(const bool a_showEdges)
{
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->setShowEdges(a_showEdges);
    }
}


//==============================================================================
/*!
    This method sets the graphic properties for edge-rendering. Options passed
    by argument include the width of the edges and their color.

    \param  a_width  Width of edge lines.
    \param  a_color  Color of edge lines.
*/
//==============================================================================
void cMultiMesh::setEdgeProperties(const double a_width, 
                                   const cColorf& a_color)
{
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->setEdgeProperties(a_width, a_color);
    }
}


//==============================================================================
/*!
    This method sets the line width of all edges.

    \param  a_width  Width of edge lines.
*/
//==============================================================================
void cMultiMesh::setEdgeLineWidth(const double a_width)
{
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->setEdgeLineWidth(a_width);
    }
}


//==============================================================================
/*!
    This method creates a list of edges by providing a threshold angle in 
    degrees. All triangles for which the angle between their respective surface 
    normals are greater than the select angle threshold are added to the list of 
    edges.

    \param  a_angleThresholdDeg  Threshold angle in degrees.
*/
//==============================================================================
void cMultiMesh::computeAllEdges(double a_angleThresholdDeg)
{
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->computeAllEdges(a_angleThresholdDeg);
    }
}


//==============================================================================
/*!
    This method clears all edges.
*/
//==============================================================================
void cMultiMesh::clearAllEdges()
{
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->clearAllEdges();
    }
}


//==============================================================================
/*!
    This method computes the global position of all vertices.

    \param  a_frameOnly  If __false__, then the global position of all vertices is computed.

*/
//==============================================================================
void cMultiMesh::updateGlobalPositions(const bool a_frameOnly)
{
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->computeGlobalPositions(a_frameOnly, 
                                      m_globalPos, 
                                      m_globalRot);
    }
}


//==============================================================================
/*!
    This method sets the color of each vertex.

    \param  a_color  New color to be applied to each vertex.
*/
//==============================================================================
void cMultiMesh::setVertexColor(const cColorf& a_color)
{
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->setVertexColor(a_color);
    }
}


//==============================================================================
/*!
    This method reverses the normal for every vertex on this model. Useful 
    for models that started with inverted faces and thus gave inward-pointing
    normals.
*/
//==============================================================================
void cMultiMesh::reverseAllNormals()
{
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->reverseAllNormals();
    }
}


//==============================================================================
/*!
    This method defines the way normals are graphically rendered.

    \param  a_length  Length of normals.
    \param  a_color   Color of normals.
*/
//==============================================================================
void cMultiMesh::setNormalsProperties(const double a_length, 
                                      const cColorf& a_color)
{
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->setNormalsProperties(a_length, a_color);
    }
}


//==============================================================================
/*!
    This method set the length of normals for display purposes.

    \param  a_length  Length of normals.
*/
//==============================================================================
void cMultiMesh::setNormalsLength(const double a_length)
{
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->setNormalsLength(a_length);
    }
}


//==============================================================================
/*!
    This method enables or disables the rendering of vertex normals.

    \param  a_showNormals  If __true__ then normal vectors are rendered graphically, __false__ otherwise.
*/
//==============================================================================
void cMultiMesh::setShowNormals(const bool& a_showNormals)
{
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->setShowNormals(a_showNormals);
    }
}


//==============================================================================
/*!
     This method builds a brute Force collision detector for this mesh.
*/
//==============================================================================
void cMultiMesh::createBruteForceCollisionDetector()
{
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->createBruteForceCollisionDetector();
    }
}


//==============================================================================
/*!
    This method builds an AABB collision detector for this mesh.

    \param  a_radius  Bounding radius.
*/
//==============================================================================
void cMultiMesh::createAABBCollisionDetector(const double a_radius)
{
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->createAABBCollisionDetector(a_radius);
    }
}


//==============================================================================
/*!
    This message renders this multi-mesh using OpenGL.

    \param  a_options  Rendering options.
*/
//==============================================================================
void cMultiMesh::render(cRenderOptions& a_options)
{
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->renderSceneGraph(a_options);
    }
}


//==============================================================================
/*!
    This method descends through child objects to compute interactions for all
    cGenericEffect classes defined for each object.

    \param  a_toolPos       Current position of tool.
    \param  a_toolVel       Current position of tool.
    \param  a_IDN           Identification number of the force algorithm.
    \param  a_interactions  List of recorded interactions.

    \return Resulting interaction force.
*/
//==============================================================================
cVector3d cMultiMesh::computeInteractions(const cVector3d& a_toolPos,
                                          const cVector3d& a_toolVel,
                                          const unsigned int a_IDN,
                                          cInteractionRecorder& a_interactions)
{
    // compute inverse rotation
    cMatrix3d localRotTrans;
    m_localRot.transr(localRotTrans);

    // compute local position of tool and velocity vector
    cVector3d toolPosLocal = cMul(localRotTrans, cSub(a_toolPos, m_localPos));

    // compute interaction between tool and current object
    cVector3d toolVelLocal = cMul(localRotTrans, a_toolVel);

    // compute forces based on the effects programmed for this object
    cVector3d localForce(0,0,0);

    // process current object if enabled
    if (m_enabled)
    {
        // check if node is a ghost. If yes, then ignore call
        if (m_ghostEnabled) { return (cVector3d(0,0,0)); }

        // compute local interaction with current object
        computeLocalInteraction(toolPosLocal,
                                toolVelLocal,
                                a_IDN);

        if(m_hapticEnabled)
        {
            // compute each force effect
            bool interactionEvent = false;
            for (unsigned int i=0; i<m_effects.size(); i++)
            {
                cGenericEffect *nextEffect = m_effects[i];

                if (nextEffect->getEnabled())
                {
                    cVector3d force(0,0,0);

                    interactionEvent = interactionEvent |
                        nextEffect->computeForce(toolPosLocal,
                                                 toolVelLocal,
                                                 a_IDN,
                                                 force);
                    localForce.add(force);
                }
            }

            // report any interaction
            if (interactionEvent)
            {
                cInteractionEvent newInteractionEvent;
                newInteractionEvent.m_object = this;
                newInteractionEvent.m_isInside = m_interactionInside;
                newInteractionEvent.m_localPos = toolPosLocal;
                newInteractionEvent.m_localSurfacePos = m_interactionPoint;
                newInteractionEvent.m_localNormal = m_interactionNormal;
                newInteractionEvent.m_localForce = localForce;
                a_interactions.m_interactions.push_back(newInteractionEvent);
            }

            // compute any other force interactions
            cVector3d force = computeOtherInteractions(toolPosLocal,
                                                       toolVelLocal,
                                                       a_IDN,
                                                       a_interactions);

            localForce.add(force);
        }
    }

    // descend through the meshes
    {
        vector<cMesh*>::iterator it;
        for (it = m_meshes->begin(); it < m_meshes->end(); it++)
        {
            cVector3d force = (*it)->computeInteractions(toolPosLocal,
                                                         toolVelLocal,
                                                         a_IDN,
                                                         a_interactions);
            localForce.add(force);
        }
    }

    // descend through the children
    {
        vector<cGenericObject*>::iterator it;
        for (it = m_children.begin(); it < m_children.end(); it++)
        {
            cVector3d force = (*it)->computeInteractions(toolPosLocal,
                                                         toolVelLocal,
                                                         a_IDN,
                                                         a_interactions);
            localForce.add(force);
        }
    }

    // convert the reaction force into my parent coordinates
    cVector3d m_globalForce = cMul(m_localRot, localForce);

    // return resulting force
    return (m_globalForce);
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
