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
    \author    Dan Morris
    \version   3.2.0 $Rev: 2158 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "world/CGenericObject.h"
#include "collisions/CGenericCollision.h"
#include "math/CMaths.h"
#include "graphics/CDraw3D.h"
#include "effects/CEffectMagnet.h"
#include "effects/CEffectStickSlip.h"
#include "effects/CEffectSurface.h"
#include "effects/CEffectVibration.h"
#include "effects/CEffectViscosity.h"
#include "shaders/CShaderProgram.h"
//------------------------------------------------------------------------------
#include <float.h>
#include <vector>
//------------------------------------------------------------------------------
using namespace std;
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
cMaterialPtr cGenericObject::s_defaultMaterial = nullptr;
cColorf cGenericObject::s_boundaryBoxColor(0.7f, 0.7f, 0.7f);
//------------------------------------------------------------------------------

//==============================================================================
/*!
    Constructor of cGenericObject.
*/
//==============================================================================
cGenericObject::cGenericObject()
{
    // object is enabled
    m_enabled = true;

    // initialize local position and orientation
    m_localPos.zero();
    m_localRot.identity();

    // initialize global position and orientation
    m_globalPos.zero();
    m_globalRot.identity();
    m_prevGlobalPos.zero();
    m_prevGlobalRot.identity();

    // initialize OpenGL matrix with position vector and orientation matrix
    m_frameGL.set(m_globalPos, m_globalRot);

    // initialize name
    m_name = "";

    // custom user information
    m_userName = "";
    m_userTag  = 0;
    m_userData = NULL;
    m_userExternalObject = NULL;

    // initialize shared default material if necessary
    if (s_defaultMaterial == nullptr) s_defaultMaterial = cMaterial::create();

    // should we use the material property?
    m_useMaterialProperty = true;

    // are vertex colors used during rendering process?
    m_useVertexColors = false;

    // should transparency be used?
    m_useTransparency = false;

    // should texture mapping be used if a texture is defined?
    m_useTextureMapping = false;

    // how are triangles displayed; FILL or LINE ?
    m_triangleMode = GL_FILL;

    // initialize texture
    m_texture = nullptr;

    // initialize normal map
    m_normalMap = nullptr;

    // turn off culling on by default
    m_cullingEnabled = false;

    // disable ghost setting
    m_ghostEnabled = false;

    // no external parent defined
    m_owner = this;

    // no parent defined
    m_parent = NULL;

    // object is not interacting with any tool
    m_interactionInside = false;
    m_interactionPoint.zero();
    m_interactionNormal.set(1,0,0);

    // empty list of haptic effects
    m_effects.clear();

    // setup default material
    m_material = s_defaultMaterial;

    // enable graphic rendering
    m_showEnabled = true;

    // enable haptic rendering
    m_hapticEnabled = true;

    // reference frame
    m_showFrame = false;
    setFrameSize(0.2);

    // boundary box
    m_showBoundaryBox = false;
    m_boundaryBoxMin.set(0.0, 0.0, 0.0); 
    m_boundaryBoxMax.set(0.0, 0.0, 0.0);
    m_boundaryBoxEmpty = true;

    // collision detector
    m_collisionDetector = NULL; 
    m_showCollisionDetector = false;

    // display list
    m_useDisplayList = false;
}


//==============================================================================
/*!
    Destructor of cGenericObject. \n

    Deletes all children starting from this point in the scene graph, so if you 
    have objects that shouldn't be deleted, be sure to remove them from the scene
    graph before deleting their parents.
*/
//==============================================================================
cGenericObject::~cGenericObject()
{
    // delete collision detector
    deleteCollisionDetector(false);

    // delete all haptics effects
    deleteAllEffects();

    // delete all children
    deleteAllChildren();
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
void cGenericObject::setEnabled(bool a_enabled, const bool a_affectChildren)
{
    m_enabled = a_enabled;

    // apply change to children
    if (a_affectChildren)
    {
        vector<cGenericObject*>::iterator it;
        for (it = m_children.begin(); it < m_children.end(); it++)
        {
            (*it)->setEnabled(a_enabled, true);
        }
    }
}


//==============================================================================
/*!
    This method translate this object by a specified offset passed as argument.

    \param  a_translation  Translation offset.
*/
//==============================================================================
void cGenericObject::translate(const cVector3d& a_translation)
{
    cVector3d localPos = cAdd(m_localPos, a_translation);
    setLocalPos(localPos);
}


//==============================================================================
/*!
    This method translates an object by a specified offset passed as argument.

    \param  a_x  Translation component X.
    \param  a_y  Translation component Y.
    \param  a_z  Translation component Z.
*/
//==============================================================================
void cGenericObject::translate(const double a_x,
    const double a_y,
    const double a_z)
{
    translate(cVector3d(a_x,a_y,a_z));
}


//==============================================================================
/*!
    This method rotates this object around a specified rotation axis and angle.
    The axis is expressed in local coordinates.

    \param  a_axis      Rotation axis. This vector must be normalized!
    \param  a_angleRad  Rotation angle in defined in __radians__.
*/
//==============================================================================
void cGenericObject::rotateAboutLocalAxisRad(const cVector3d& a_axis, const double a_angleRad)
{
    cMatrix3d rot = m_localRot;
    rot.rotateAboutLocalAxisRad(a_axis, a_angleRad);
    setLocalRot(rot);
}


//==============================================================================
/*!
    This method rotates this object around a specified rotation axis and angle.
    The axis is expressed in global coordinates.

    \param  a_axis      Rotation axis. This vector must be normalized!
    \param  a_angleRad  Rotation angle in defined in __radians__.
*/
//==============================================================================
void cGenericObject::rotateAboutGlobalAxisRad(const cVector3d& a_axis,  const double a_angleRad)
{
    cMatrix3d rot = m_localRot;
    rot.rotateAboutGlobalAxisRad(a_axis, a_angleRad);
    setLocalRot(rot);
}


//==============================================================================
/*!
    This method builds a rotation matrix from a set of Euler angles and 
    fixed axes of rotations.

    \param  a_angleRad1   Angle in radians of the first rotation in the sequence.
    \param  a_angleRad2   Angle in radians of the second rotation in the sequence.
    \param  a_angleRad3   Angle in radians of the third rotation in the sequence.
    \param  a_eulerOrder  The order of the axes about which the rotations are to be applied
*/
//==============================================================================
void cGenericObject::rotateExtrinsicEulerAnglesRad(const double& a_angleRad1,
    const double& a_angleRad2,
    const double& a_angleRad3,
    const cEulerOrder a_eulerOrder)
{
    cMatrix3d rot = m_localRot;
    rot.setExtrinsicEulerRotationRad(a_angleRad1,
                                     a_angleRad2,
                                     a_angleRad3,
                                     a_eulerOrder);
    setLocalRot(rot);
}


//==============================================================================
/*!
    This method builds a rotation matrix from a set of Euler angles and 
    co-moving axes of rotations.

    \param  a_angleRad1   Angle in radians of the first rotation in the sequence.
    \param  a_angleRad2   Angle in radians of the second rotation in the sequence.
    \param  a_angleRad3   Angle in radians of the third rotation in the sequence.
    \param  a_eulerOrder  The order of the axes about which the rotations are to be applied.
*/
//==============================================================================
void cGenericObject::rotateIntrinsicEulerAnglesRad(const double& a_angleRad1,
    const double& a_angleRad2,
    const double& a_angleRad3,
    const cEulerOrder a_eulerOrder)
{
    cMatrix3d rot = m_localRot;
    rot.setIntrinsicEulerRotationRad(a_angleRad1,
                                     a_angleRad2,
                                     a_angleRad3,
                                     a_eulerOrder);
    setLocalRot(rot);
}


//==============================================================================
/*!
    This method computes the global position and global rotation matrix given
    the local position and location rotation of this object, and the global 
    configuration of its parent. \n

    If \a a_frameOnly is set to __false__, additional global positions such as
    vertex positions are computed too (which may be time-consuming!). \n

    \param  a_frameOnly  If __true__ then only the global frame is computed
    \param  a_globalPos  Global position of parent object.
    \param  a_globalRot  Global rotation matrix of parent object.
*/
//==============================================================================
void cGenericObject::computeGlobalPositions(const bool a_frameOnly,
    const cVector3d& a_globalPos, 
    const cMatrix3d& a_globalRot)
{
    // check if node is a ghost. If yes, then ignore call
    if (m_ghostEnabled) { return; }

    // current values become previous values
    m_prevGlobalPos = m_globalPos;
    m_prevGlobalRot = m_globalRot;

    // update global position vector and global rotation matrix
    m_globalPos = cAdd(a_globalPos, cMul(a_globalRot, m_localPos));
    m_globalRot = cMul(a_globalRot, m_localRot);

    // update any positions within the current object that need to be
    // updated (e.g. vertex positions)
    updateGlobalPositions(a_frameOnly);

    // propagate this method to my children
    vector<cGenericObject*>::iterator it;
    for (it = m_children.begin(); it < m_children.end(); it++)
    {
        (*it)->computeGlobalPositions(a_frameOnly, m_globalPos, m_globalRot);
    }
}


//==============================================================================
/*!
    This method computes the global position and global rotation for this 
    object only, by recursively climbing up the scene graph tree until the root
    is reached.

    If argument \p a_frameOnly is set to __false__, additional global positions 
    such as vertex positions are computed.

    \param  a_frameOnly  If __true__ then only the global frame is computed.
*/
//==============================================================================
void cGenericObject::computeGlobalPositionsFromRoot(const bool a_frameOnly)
{
    cMatrix3d globalRot;
    cVector3d globalPos;
    globalRot.identity();
    globalPos.zero();

    // get a pointer to current object
    cGenericObject *curObject = getParent();

    if (curObject != NULL)
    {
        // walk up the scene graph until we reach the root, updating
        // my global position and rotation at each step
        do 
        {
            curObject->getLocalRot().mul(globalPos);
            globalPos.add(curObject->getLocalPos());
            cMatrix3d rot;
            curObject->getLocalRot().mulr(globalRot, rot);
            rot.copyto(globalRot);
            curObject = curObject->getParent();
        } 
        while (curObject != NULL);
    }

    // compute global positions for current object and child
    computeGlobalPositions(a_frameOnly, globalPos, globalRot);
}


//==============================================================================
/*!
    This method adds a haptic effect to this object.

    \param  a_effect  Haptic effect to be added to the list.
*/
//==============================================================================
bool cGenericObject::addEffect(cGenericEffect* a_effect)
{
    if (a_effect->m_parent != this)
    {
        return (false);
    }

    // update the effect object's parent pointer
    a_effect->m_parent = this;

    // add this child to my list of children
    m_effects.push_back(a_effect);

    // success
    return (true);
}


//==============================================================================
/*!
    This method removes a haptic effect from this object.

    \param  a_effect  Haptic effect to be removed from the list of effects.
*/
//==============================================================================
bool cGenericObject::removeEffect(cGenericEffect* a_effect)
{
    // find haptic effect and remove it from list
    vector<cGenericEffect*>::iterator it;
    for (it = m_effects.begin(); it < m_effects.end(); it++)
    {
        if ((*it) == a_effect)
        {
            (*it)->m_parent = NULL;
            m_effects.erase(it);
            return (true);
        }
    }

    // operation failed
    return (false);
}


//==============================================================================
/*!
    This method deletes all haptic effects and removes them from the list.
*/
//==============================================================================
void cGenericObject::deleteAllEffects()
{
    vector<cGenericEffect*>::iterator it;
    for (it = m_effects.begin(); it < m_effects.end(); it++)
    {
        delete (*it);
    }
    m_effects.clear();
}


//==============================================================================
/*!
    This method creates a magnetic haptic effect.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cGenericObject::createEffectMagnetic()
{
    bool flag = true;
    vector<cGenericEffect*>::iterator it;
    for (it = m_effects.begin(); it < m_effects.end(); it++)
    {
        if (dynamic_cast<cEffectMagnet*>(*it) != NULL)
        {
            flag = false;
        }
    }

    if (flag)
    {
        cGenericEffect* effect = new cEffectMagnet(this);
        addEffect(effect);
        return (true);
    }
    return (false);
}


//==============================================================================
/*!
    This method deletes the magnetic haptic effect.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cGenericObject::deleteEffectMagnetic()
{
    vector<cGenericEffect*>::iterator it;
    for (it = m_effects.begin(); it < m_effects.end(); it++)
    {
        cGenericEffect* effect = dynamic_cast<cEffectMagnet*>(*it);
        if (effect != NULL)
        {
            m_effects.erase(it);
            delete effect;
            return (true);
        }
    }
    return (false);
}


//==============================================================================
/*!
    This method creates a stick-and-slip haptic effect.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cGenericObject::createEffectStickSlip()
{
    bool flag = true;
    vector<cGenericEffect*>::iterator it;
    for (it = m_effects.begin(); it < m_effects.end(); it++)
    {
        if (dynamic_cast<cEffectStickSlip*>(*it) != NULL)
        {
            flag = false;
        }
    }

    if (flag)
    {
        cGenericEffect* effect = new cEffectStickSlip(this);
        addEffect(effect);
        return (true);
    }
    return (false);
}

//==============================================================================
/*!
    This method deletes the stick-and-slip haptic effect.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cGenericObject::deleteEffectStickSlip()
{
    vector<cGenericEffect*>::iterator it;
    for (it = m_effects.begin(); it < m_effects.end(); it++)
    {
        cGenericEffect* effect = dynamic_cast<cEffectStickSlip*>(*it);
        if (effect != NULL)
        {
            m_effects.erase(it);
            delete effect;
            return (true);
        }
    }
    return (false);
}


//==============================================================================
/*!
    This method creates a surface haptic effect.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cGenericObject::createEffectSurface()
{
    bool flag = true;
    vector<cGenericEffect*>::iterator it;
    for (it = m_effects.begin(); it < m_effects.end(); it++)
    {
        if (dynamic_cast<cEffectSurface*>(*it) != NULL)
        {
            flag = false;
        }
    }

    if (flag)
    {
        cGenericEffect* effect = new cEffectSurface(this);
        addEffect(effect);
        return (true);
    }
 
    return (false);
}


//==============================================================================
/*!
    This method deletes the surface haptic effect.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cGenericObject::deleteEffectSurface()
{
    vector<cGenericEffect*>::iterator it;
    for (it = m_effects.begin(); it < m_effects.end(); it++)
    {
        cGenericEffect* effect = dynamic_cast<cEffectSurface*>(*it);
        if (effect != NULL)
        {
            m_effects.erase(it);
            delete effect;
            return (true);
        }
    }
    return (false);
}


//==============================================================================
/*!
    This method creates a vibration haptic effect.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cGenericObject::createEffectVibration()
{
    bool flag = true;
    vector<cGenericEffect*>::iterator it;
    for (it = m_effects.begin(); it < m_effects.end(); it++)
    {
        if (dynamic_cast<cEffectVibration*>(*it) != NULL)
        {
            flag = false;
        }
    }

    if (flag)
    {
        cGenericEffect* effect = new cEffectVibration(this);
        addEffect(effect);
        return (true);
    }
    return (false);
}


//==============================================================================
/*!
    This method deletes the current vibration haptic effect.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cGenericObject::deleteEffectVibration()
{
    vector<cGenericEffect*>::iterator it;
    for (it = m_effects.begin(); it < m_effects.end(); it++)
    {
        cGenericEffect* effect = dynamic_cast<cEffectVibration*>(*it);
        if (effect != NULL)
        {
            m_effects.erase(it);
            delete effect;
            return (true);
        }
    }
    return (false);
}


//==============================================================================
/*!
    This method creates a viscous haptic effect.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cGenericObject::createEffectViscosity()
{
    bool flag = true;
    vector<cGenericEffect*>::iterator it;
    for (it = m_effects.begin(); it < m_effects.end(); it++)
    {
        if (dynamic_cast<cEffectViscosity*>(*it) != NULL)
        {
            flag = false;
        }
    }

    if (flag)
    {
        cGenericEffect* effect = new cEffectViscosity(this);
        addEffect(effect);
        return (true);
    }
    return (false);
}


//==============================================================================
/*!
    This method deletes the current viscous haptic effect.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cGenericObject::deleteEffectViscosity()
{
    vector<cGenericEffect*>::iterator it;
    for (it = m_effects.begin(); it < m_effects.end(); it++)
    {
        cGenericEffect* effect = dynamic_cast<cEffectViscosity*>(*it);
        if (effect != NULL)
        {
            m_effects.erase(it);
            delete effect;
            return (true);
        }
    }
    return (false);
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
void cGenericObject::setHapticEnabled(const bool a_hapticEnabled, 
    const bool a_affectChildren)
{
    m_hapticEnabled = a_hapticEnabled;

    // apply change to children
    if (a_affectChildren)
    {
        vector<cGenericObject*>::iterator it;
        for (it = m_children.begin(); it < m_children.end(); it++)
        {
            (*it)->setHapticEnabled(a_hapticEnabled, true);
        }
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
void cGenericObject::setStiffness(const double a_stiffness, 
    const bool a_affectChildren)
{
    m_material->setStiffness(a_stiffness);

    // apply change to children
    if (a_affectChildren)
    {
        vector<cGenericObject*>::iterator it;
        for (it = m_children.begin(); it < m_children.end(); it++)
        {
            (*it)->setStiffness(a_stiffness, true);
        }
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
void cGenericObject::setFriction(double a_staticFriction, 
    double a_dynamicFriction, 
    const bool a_affectChildren)
{
    m_material->setStaticFriction(a_staticFriction);
    m_material->setDynamicFriction(a_dynamicFriction);

    // apply change to children
    if (a_affectChildren)
    {
        vector<cGenericObject*>::iterator it;
        for (it = m_children.begin(); it < m_children.end(); it++)
        {
            (*it)->setFriction(a_staticFriction, a_dynamicFriction, true);
        }
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
void cGenericObject::setShowEnabled(const bool a_show, const bool a_affectChildren)
{
    // update current object
    m_showEnabled = a_show;

    // apply change to children
    if (a_affectChildren)
    {
        vector<cGenericObject*>::iterator it;
        for (it = m_children.begin(); it < m_children.end(); it++)
        {
            (*it)->setShowEnabled(a_show, true);
        }
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
void cGenericObject::setUseTransparency(const bool a_useTransparency, const bool a_affectChildren)
{
    // update changes to object
    m_useTransparency = a_useTransparency;

    // apply change to children
    if (a_affectChildren)
    {
        vector<cGenericObject*>::iterator it;
        for (it = m_children.begin(); it < m_children.end(); it++)
        {
            (*it)->setUseTransparency(a_useTransparency, true);
        }
    }
}


//==============================================================================
/*!
    This method sets the alpha value to all components of the object,
    optionally propagating the operation to children. \n

    Using the 'apply to textures' option causes the actual texture
    alpha values to be over-written in my texture, if it exists. \n

    \param  a_level            Level of transparency ranging from 0.0 to 1.0.
    \param  a_applyToTextures  If __true__, then apply changes to texture pixels.
    \param  a_applyToVertices  If __true__, then apply changes to vertex colors.
    \param  a_affectChildren   If __true__, then children are updated too.
*/
//==============================================================================
void cGenericObject::setTransparencyLevel(const float a_level,
    const bool a_applyToVertices,
    const bool a_applyToTextures,
    const bool a_affectChildren)
{
    // if the transparency level is equal to 1.0, then do not apply transparency
    // otherwise enable it.
    if (a_level < 1.0)
    {
        setUseTransparency(true);
    }
    else
    {
        setUseTransparency(false);
    }

    // apply new value to material
    if (m_material != nullptr)
    {
        m_material->setTransparencyLevel(a_level);
    }

    // apply new value to texture
    if (m_texture != nullptr)
    {
        if (m_texture->m_image != nullptr)
        {
            unsigned char level = (unsigned char)(255.0 * a_level);
            m_texture->m_image->setTransparency(level);
        }
    }

    // apply change to children
    if (a_affectChildren)
    {
        vector<cGenericObject*>::iterator it;
        for (it = m_children.begin(); it < m_children.end(); it++)
        {
            (*it)->setTransparencyLevel(a_level,
                                        a_applyToVertices,
                                        a_applyToTextures,
                                        true);
        }
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
void cGenericObject::setWireMode(const bool a_showWireMode, 
                                 const bool a_affectChildren)
{
    if (a_showWireMode)
    { 
        // enable line mode
        m_triangleMode = GL_LINE;
    }
    else
    {
        // enable solid mode
        m_triangleMode = GL_FILL;
    }

    // apply change to children
    if (a_affectChildren)
    {
        vector<cGenericObject*>::iterator it;
        for (it = m_children.begin(); it < m_children.end(); it++)
        {
            (*it)->setWireMode(a_showWireMode, true);
        }
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
void cGenericObject::setUseCulling(const bool a_useCulling, 
                                   const bool a_affectChildren)
{
    m_cullingEnabled = a_useCulling;

    // apply change to children
    if (a_affectChildren)
    {
        vector<cGenericObject*>::iterator it;
        for (it = m_children.begin(); it < m_children.end(); it++)
        {
            (*it)->setUseCulling(a_useCulling, true);
        }
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
void cGenericObject::setUseVertexColors(const bool a_useColors, 
                                        const bool a_affectChildren)
{
    m_useVertexColors = a_useColors;

    // apply change to children
    if (a_affectChildren)
    {
        vector<cGenericObject*>::iterator it;
        for (it = m_children.begin(); it < m_children.end(); it++)
        {
            (*it)->setUseVertexColors(a_useColors, true);
        }
    }
}


//==============================================================================
/*!
    This method creates a backup of the material color properties of this object,
    optionally recursively affecting children.

    \param  a_affectChildren  If __true__ then children are updated too.
*/
//==============================================================================
void cGenericObject::backupMaterialColors(const bool a_affectChildren)
{
    // backup colors for current material
    if ((m_material != nullptr) && (m_material != s_defaultMaterial))
    {
        m_material->backupColors();
    }

    // apply change to children
    if (a_affectChildren)
    {
        vector<cGenericObject*>::iterator it;
        for (it = m_children.begin(); it < m_children.end(); it++)
        {
            (*it)->backupMaterialColors(true);
        }
    }
}

 
//==============================================================================
/*!
    This method restores material color properties for this object, optionally 
    recursively affecting children.

    \param  a_affectChildren  If __true__ then children are updated too.
*/
//==============================================================================
void cGenericObject::restoreMaterialColors(const bool a_affectChildren)
{
    // restore original colors for current material
    if ((m_material != nullptr) && (m_material != s_defaultMaterial))
    {
        m_material->restoreColors();
    }

    // apply change to children
    if (a_affectChildren)
    {
        vector<cGenericObject*>::iterator it;
        for (it = m_children.begin(); it < m_children.end(); it++)
        {
            (*it)->restoreMaterialColors(true);
        }
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
void cGenericObject::setUseDisplayList(const bool a_useDisplayList,
                                       const bool a_affectChildren)
{
    // update changes to object
    m_useDisplayList = a_useDisplayList;

    // apply change to children
    if (a_affectChildren)
    {
        vector<cGenericObject*>::iterator it;
        for (it = m_children.begin(); it < m_children.end(); it++)
        {
            (*it)->setUseDisplayList(a_useDisplayList, true);
        }
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
void cGenericObject::markForUpdate(const bool a_affectChildren)
{
    // invalidate display list
    m_displayList.invalidate();

    // apply change to children
    if (a_affectChildren)
    {
        vector<cGenericObject*>::iterator it;
        for (it = m_children.begin(); it < m_children.end(); it++)
        {
            (*it)->markForUpdate(true);
        }
    }
}


//==============================================================================
/*!
     This method enables or disables the use of material properties.

     \param  a_useMaterial     If __true__ then material properties are used for rendering.
     \param  a_affectChildren  If __true__ then children are updated too.
*/
//==============================================================================
void cGenericObject::setUseMaterial(const bool a_useMaterial, 
                                    const bool a_affectChildren)
{
    // update changes to object
    m_useMaterialProperty = a_useMaterial;

    // apply change to children
    if (a_affectChildren)
    {
        vector<cGenericObject*>::iterator it;
        for (it = m_children.begin(); it < m_children.end(); it++)
        {
            (*it)->setUseMaterial(a_useMaterial, true);
        }
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
void cGenericObject::setMaterial(cMaterialPtr a_material,
                                 const bool a_affectChildren)
{
    // copy material properties
    m_material = a_material;

    // apply change to children
    if (a_affectChildren)
    {
        vector<cGenericObject*>::iterator it;
        for (it = m_children.begin(); it < m_children.end(); it++)
        {
            (*it)->setMaterial(a_material, true);
        }
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
void cGenericObject::setMaterial(cMaterial& a_material,
                                 const bool a_affectChildren)
{
    // copy material properties
    if ((m_material == nullptr) || (m_material == s_defaultMaterial))
    {
        m_material = cMaterial::create();
        a_material.copyTo(m_material);
    }
    else
    {
        a_material.copyTo(m_material);
    }

    // apply change to children
    if (a_affectChildren)
    {
        vector<cGenericObject*>::iterator it;
        for (it = m_children.begin(); it < m_children.end(); it++)
        {
            (*it)->setMaterial(a_material, true);
        }
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
void cGenericObject::setUseTexture(const bool a_useTexture, 
                                   const bool a_affectChildren)
{
    m_useTextureMapping = a_useTexture;

    // apply change to children
    if (a_affectChildren)
    {
        vector<cGenericObject*>::iterator it;
        for (it = m_children.begin(); it < m_children.end(); it++)
        {
            (*it)->setUseTexture(a_useTexture, true);
        }
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
void cGenericObject::setTexture(cTexture1dPtr a_texture,
                                const bool a_affectChildren)
{
    m_texture = a_texture;

    // apply change to children
    if (a_affectChildren)
    {
        vector<cGenericObject*>::iterator it;
        for (it = m_children.begin(); it < m_children.end(); it++)
        {
            (*it)->setTexture(a_texture, true);
        }
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
void cGenericObject::setShaderProgram(cShaderProgramPtr a_shaderProgram,
                                      const bool a_affectChildren)
{
    m_shaderProgram = a_shaderProgram;

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
void cGenericObject::setShowBoundaryBox(const bool a_showBoundaryBox, 
                                        const bool a_affectChildren)
{
    // update current object
    m_showBoundaryBox = a_showBoundaryBox;

    // apply change to children
    if (a_affectChildren)
    {
        vector<cGenericObject*>::iterator it;
        for (it = m_children.begin(); it < m_children.end(); it++)
        {
            (*it)->setShowBoundaryBox(a_showBoundaryBox, true);
        }
    }
}


//==============================================================================
/*!
    This method computes the boundary box of this object and all of its children.

    If argument \p a_includeChildren is set to __true__ then each object's
    bounding box covers its own volume and the volume of its children. If it is 
    set to false, then the object's bounding volume shall only cover its own
    geometrical elements.

    \param  a_includeChildren  If __true__, then children are included in the
                               boundary volume of the parent.
*/
//==============================================================================
void cGenericObject::computeBoundaryBox(const bool a_includeChildren)
{
    // check if node is a ghost. If yes, then ignore call
    if (m_ghostEnabled) { return; }

    // compute the bounding box of this object
    updateBoundaryBox();

    // if children are not included, then we simply propagate the command
    // recursively so that all children update their bounding box individually.
    if (a_includeChildren == false) 
    {
        vector<cGenericObject*>::iterator it;
        for (it = m_children.begin(); it < m_children.end(); it++)
        {
            cGenericObject* object = (*it);

            // compute boundary box of child
            object->computeBoundaryBox(false);
        }

        return;
    }

    // compute the bounding box of all my children
    cVector3d minBox, maxBox;

    if (m_boundaryBoxEmpty)
    {
        minBox.set( C_LARGE, C_LARGE, C_LARGE);
        maxBox.set(-C_LARGE,-C_LARGE,-C_LARGE);
    }
    else
    {
        minBox = m_boundaryBoxMin;
        maxBox = m_boundaryBoxMax;
    }

    bool empty = true;
    vector<cGenericObject*>::iterator it;
    for (it = m_children.begin(); it < m_children.end(); it++)
    {
        cGenericObject* object = (*it);

        // compute boundary box of child
        object->computeBoundaryBox(a_includeChildren);

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
                minBox(0) = cMin(corners[i](0),  m_boundaryBoxMin(0));
                minBox(1) = cMin(corners[i](1),  m_boundaryBoxMin(1));
                minBox(2) = cMin(corners[i](2),  m_boundaryBoxMin(2));
                maxBox(0) = cMin(corners[i](0),  m_boundaryBoxMax(0));
                maxBox(1) = cMin(corners[i](1),  m_boundaryBoxMax(1));
                maxBox(2) = cMin(corners[i](2),  m_boundaryBoxMax(2));
            }
        }
    }

    if (empty && m_boundaryBoxEmpty)
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
    This method enables of disables the display of the reference frame.
    The reference frame is a set of arrows that represent this object's position
    and orientation.

    If argument \p a_affectChildren is set to __true__ then all children are 
    updated with the new value.

    \param  a_showFrame       If __true__ then frame is displayed.
    \param  a_affectChildren  If __true__ then children are updated too.
*/
//==============================================================================
void cGenericObject::setShowFrame(const bool a_showFrame, 
                                  const bool a_affectChildren)
{
    // update current object
    m_showFrame = a_showFrame;

    // apply change to children
    if (a_affectChildren)
    {
        vector<cGenericObject*>::iterator it;
        for (it = m_children.begin(); it < m_children.end(); it++)
        {
            (*it)->setShowFrame(a_showFrame, true);
        }
    }
}


//==============================================================================
/*!
    This method sets the display size of the arrows representing my reference
    frame. The size corresponds to the length of each displayed axis (X-Y-Z). \n

    If argument \p a_affectChildren is set to __true__ then all children are 
    updated with the new value.

    \param  a_size            Length of graphical representation of frame.
    \param  a_affectChildren  If __true__ then children are updated too.
*/
//==============================================================================
void cGenericObject::setFrameSize(const double a_size, 
                                  const bool a_affectChildren)
{
    // sanity check
    m_frameSize = fabs(a_size);
    m_frameThicknessScale = 1.7 * m_frameSize;

    // apply change to children
    if (a_affectChildren)
    {
        vector<cGenericObject*>::iterator it;
        for (it = m_children.begin(); it < m_children.end(); it++)
        {
            (*it)->setFrameSize(a_size, true);
        }
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
void cGenericObject::deleteCollisionDetector(const bool a_affectChildren)
{
    // delete current collision detector
    if (m_collisionDetector)
    {
        delete m_collisionDetector;
        m_collisionDetector = NULL;
    }

    // apply change to children
    if (a_affectChildren)
    {
        vector<cGenericObject*>::iterator it;
        for (it = m_children.begin(); it < m_children.end(); it++)
        {
            (*it)->deleteCollisionDetector(true);
        }
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
bool cGenericObject::computeCollisionDetection(const cVector3d& a_segmentPointA,
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
    // CHECK COLLISIONS
    ///////////////////////////////////////////////////////////////////////////

    if ((m_enabled) &&
        ((a_settings.m_checkVisibleObjects && m_showEnabled) ||
         (a_settings.m_checkHapticObjects && m_hapticEnabled)))
    {

        ///////////////////////////////////////////////////////////////////////
        // DYNAMIC MOTION COMPENSATION
        // adjust the first segment endpoint so that it is in the same position
        // relative to the moving object as it was at the previous haptic iteration
        ///////////////////////////////////////////////////////////////////////
        cVector3d localSegmentPointAadjusted;
        if (a_settings.m_adjustObjectMotion)
        {
            adjustCollisionSegment(localSegmentPointA, localSegmentPointAadjusted);
        }
        else
        {
            localSegmentPointAadjusted = localSegmentPointA;
        }

        ///////////////////////////////////////////////////////////////////////
        // COLLISION DETECTOR
        ///////////////////////////////////////////////////////////////////////
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
        hit = hit | computeOtherCollisionDetection(localSegmentPointAadjusted,
                                                   localSegmentPointB,
                                                   a_recorder,
                                                   a_settings);
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
void cGenericObject::setShowCollisionDetector(const bool a_showCollisionDetector, 
                                              const bool a_affectChildren)
{
    m_showCollisionDetector = a_showCollisionDetector;

    // apply change to children
    if (a_affectChildren)
    {
        vector<cGenericObject*>::iterator it;
        for (it = m_children.begin(); it < m_children.end(); it++)
        {
            (*it)->setShowCollisionDetector(a_showCollisionDetector, true);
        }
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
void cGenericObject::setCollisionDetectorProperties(unsigned int a_displayDepth,
                                                    cColorf& a_color, 
                                                    const bool a_affectChildren)
{
    if (m_collisionDetector != NULL)
    {
        m_collisionDetector->m_color = a_color;
        m_collisionDetector->setDisplayDepth(a_displayDepth);
    }

    // apply change to children
    if (a_affectChildren)
    {
        vector<cGenericObject*>::iterator it;
        for (it = m_children.begin(); it < m_children.end(); it++)
        {
            (*it)->setCollisionDetectorProperties(a_displayDepth,
                                                  a_color, 
                                                  true);
        }
    }
}


//==============================================================================
/*!
    This method adds an object to the scene graph below this object. \n
    Note that an object can only be a child of \b one single object, unless 
    )ghosting_ is enabled.

    \param  a_object  Object to be added to child list.

    \return __true__ if operation succeeded.
*/
//==============================================================================
bool cGenericObject::addChild(cGenericObject* a_object)
{
    // sanity check
    if ((a_object == NULL) || (a_object == this)) { return (false); }

    // the object does not have any parent yet, so we can add it as a child
    // to current object.
    if (a_object->m_parent == NULL)
    {
        m_children.push_back(a_object);
        a_object->m_parent = this;
        return (true);
    }

    // the object already has a parent, however since ghosting is enabled we add it
    // as a child without changing its m_parent member
    else if (m_ghostEnabled)
    {
        m_children.push_back(a_object);
        return (true);
    }

    // operation fails
    return (false);
}


//==============================================================================
/*!
    This method removes an object from the list of children, without deleting the
    child object from memory. \n

    This method assigns the child object's parent point to null, so
    if you're moving an object around in your scene graph, make sure you
    call this function _before_ you add the child to another node in
    the scene graph.

    \param  a_object  Object to be removed from my list of children.

    \return __true__ if the specified object was found on my list of children,
            __false__ otherwise.
*/
//==============================================================================
bool cGenericObject::removeChild(cGenericObject* a_object)
{
    // sanity check
    if (a_object == NULL) { return (false); }

    vector<cGenericObject*>::iterator it;
    for (it = m_children.begin(); it < m_children.end(); it++)
    {
        if ((*it) == a_object)
        {
            // he doesn't have a parent any more
            a_object->m_parent = NULL;

            // remove this object from my list of children
            m_children.erase(it);

            // return success
            return (true);
        }
    }

    // operation failed
    return (false);
}


//==============================================================================
/*!
    This method removes this object from its parent's list of children.

    \return  __true__ if operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cGenericObject::removeFromGraph()
{
    if (m_parent)
    {
        return (m_parent->removeChild(this));
    }
    else 
    {
        return (false);
    }
}


//==============================================================================
/*!
    This method removes an object from its list of children, and deletes the
    child object from memory.

    \param  a_object  Object to be removed from my list of children 
                      and deleted.

    \return __true__ if the specified object was found in the child list.
*/
//==============================================================================
bool cGenericObject::deleteChild(cGenericObject* a_object)
{
    // sanity check
    if (a_object == NULL) { return (false); }

    // remove object from list
    bool result = removeChild(a_object);

    // if operation succeeds, delete the object
    if (result)
    {
        delete (a_object);
    }

    // return result
    return result;
}


//==============================================================================
/*!
    This method clears all objects from my list of children, without deleting
    them.
*/
//==============================================================================
void cGenericObject::clearAllChildren()
{
    // clear parent member for all children
    vector<cGenericObject*>::iterator it;
    for (it = m_children.begin(); it < m_children.end(); it++)
    {
        (*it)->m_parent = NULL;
    }

    // clear children list
    m_children.clear();
}


//==============================================================================
/*!
    This method deletes and clear all objects the list of children.
*/
//==============================================================================
void cGenericObject::deleteAllChildren()
{
    // delete all children
    vector<cGenericObject*>::iterator it;
    for (it = m_children.begin(); it < m_children.end(); it++)
    {
        cGenericObject* nextObject = (*it);
        delete (nextObject);
    }

    // clear my list of children
    m_children.clear();
}


//==============================================================================
/*!
    This method returns the total number of descendants, optionally including
    this object.

    \param  a_includeCurrentObject  If __true__ then this object is included in the
                                    count.

    \return Number of descendants found.
*/
//==============================================================================
unsigned int cGenericObject::getNumDescendants(bool a_includeCurrentObject)
{
    unsigned int numDescendants = a_includeCurrentObject?1:0;

    vector<cGenericObject*>::iterator it;
    for (it = m_children.begin(); it < m_children.end(); it++)
    {
        numDescendants += (*it)->getNumDescendants(true);
    }

    return (numDescendants);
}


//==============================================================================
/*!
    This method performs a uniform scale on the object, optionally including 
    children.

    \param  a_scaleFactor     Scale factor.
    \param  a_affectChildren  If __true__ then children are updated too.
*/
//==============================================================================
void cGenericObject::scale(const double& a_scaleFactor, 
                           const bool a_affectChildren)
{
    // scale object
    scaleObject(a_scaleFactor);

    // apply change to children
    if (a_affectChildren)
    {
        vector<cGenericObject*>::iterator it;
        for (it = m_children.begin(); it < m_children.end(); it++)
        {
            (*it)->m_localPos.mul(a_scaleFactor);
            (*it)->scale(a_scaleFactor, true);
        }
    }
}


//==============================================================================
/*!
    This method render this object. Subclasses will generally override this method.
    This is called from renderSceneGraph, which subclasses generally do
    not need to override. \n\n

    A word on OpenGL conventions: \n

    CHAI3D does not re-initialize the OpenGL state at every rendering
    pass.  The only OpenGL state variables that CHAI3D sets explicitly in a typical
    rendering pass are: \n\n

    * lighting is enabled (cWorld) \n
    * depth-testing is enabled (cWorld) \n
    * glColorMaterial is enabled and set to GL_AMBIENT_AND_DIFFUSE/GL_FRONT_AND_BACK (cWorld) \n
    * a perspective projection matrix is set up (cCamera) \n\n

    This adherence to the defaults is nice because it lets an application change an important
    piece of state globally and not worry about it getting changed by CHAI3D objects. \n\n

    It is expected that objects will "clean up after themselves" if they change
    any rendering state besides:\n\n

    * color (glColor) \n
    * material properties (glMaterial) \n
    * normals (glNormal) \n\n

    For example, if my object changes the rendering color, I don't need to set it back
    before returning, but if my object turns on vertex buffering, I should turn it
    off before returning.  Consequently if I care about the current color, I should
    set it up in my own render() function, because I shouldn't count on it being
    meaningful when my render() function is called. \n\n

    Necessary exceptions to these conventions include: \n\n

    * cLight will change the lighting state for his assigned GL_LIGHT \n
    * cCamera sets up relevant transformation matrices \n\n

    \param  a_options  Rendering options.
*/
//==============================================================================
void cGenericObject::render(cRenderOptions& a_options) 
{ 
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
void cGenericObject::computeLocalInteraction(const cVector3d& a_toolPos,
    const cVector3d& a_toolVel,
    const unsigned int a_IDN)
{
    m_interactionPoint.set(0,0,0);

    double length = a_toolPos.length();
    if (length == 0.0)
    {
        m_interactionNormal.set(0,0,1);
    }
    else
    {
        m_interactionNormal = -(1.0/length)*a_toolPos;
    }
    
    m_interactionInside = true;
}


//==============================================================================
/*!
    This method copies all material and texture properties from current generic
    object to another. This function is typically called by \ref copy() method 
    which duplicates an instance of a subclass of cGenericObject.

    \param  a_obj                     Object to which properties are copied to
    \param  a_duplicateMaterialData   If __true__, material (if available) is duplicated, otherwise it is shared.
    \param  a_duplicateTextureData    If __true__, texture data (if available) is duplicated, otherwise it is shared.
    \param  a_duplicateMeshData       If __true__, mesh data (if available) is duplicated, otherwise it is shared.
    \param  a_buildCollisionDetector  If __true__, collision detector (if available) is duplicated, otherwise it is shared.
*/
//==============================================================================
void cGenericObject::copyGenericObjectProperties(cGenericObject* a_obj, 
    const bool a_duplicateMaterialData,
    const bool a_duplicateTextureData, 
    const bool a_duplicateMeshData,
    const bool a_buildCollisionDetector)
{
    // sanity check
    if (a_obj == NULL) { return; }

    // copy material
    if ((a_duplicateMaterialData) && ((m_material != nullptr) || (m_material != s_defaultMaterial)))
    {
        a_obj->m_material = m_material->copy();
    }
    else
    {
        a_obj->m_material = m_material;
    }

    // copy texture
    if ((a_duplicateTextureData) && (m_texture != nullptr))
    {
        a_obj->m_texture = m_texture->copy();    
    }
    else
    {
        a_obj->m_texture = m_texture;
    }

    // copy general properties and settings
    a_obj->m_useTextureMapping    = m_useTextureMapping;
    a_obj->m_useMaterialProperty  = m_useMaterialProperty;
    a_obj->m_useVertexColors      = m_useVertexColors;
    a_obj->m_useTransparency      = m_useTransparency;
    a_obj->m_cullingEnabled       = m_cullingEnabled;
    a_obj->m_localPos             = m_localPos;
    a_obj->m_localRot             = m_localRot;
}


//==============================================================================
/*!
    This method renders the scene graph starting at this object. This method is
    called for each object and optionally render the object itself, its 
    reference frame and the collision and/or scenegraph trees. \n

    The object itself is rendered by calling render(), which should be defined
    for each subclass that has a graphical representation.  renderSceneGraph
    does not generally need to be over-ridden in subclasses. \n

    The a_options parameter is used to allow multiple rendering passes. 
    See CRenderOptionh.h for more information.

    \param  a_options  Rendering options.
*/
//==============================================================================
void cGenericObject::renderSceneGraph(cRenderOptions& a_options)
{
#ifdef C_USE_OPENGL

    /////////////////////////////////////////////////////////////////////////
    // Initialize rendering
    /////////////////////////////////////////////////////////////////////////

    // convert the position and orientation of the object into a 4x4 matrix 
    // that is supported by openGL. This task only needs to be performed during the first
    // rendering pass
    if (a_options.m_storeObjectPositions)
    {
        m_frameGL.set(m_localPos, m_localRot);
    }

    // push object position/orientation on stack
    glPushMatrix();
    glMultMatrixd( (const double *)m_frameGL.getData() );

    // render if object is enabled
    if (m_enabled)
    {
        //--------------------------------------------------------------------------
        // Request for RESET
        //-----------------------------------------------------------------------
        if(a_options.m_markForUpdate)
        {
            // invalidate display list 
            markForUpdate(false);

            // invalidate texture
            if (m_texture != nullptr)
            {
                m_texture->markForUpdate();
            }
        }

        //-----------------------------------------------------------------------
        // Init
        //-----------------------------------------------------------------------

        glEnable(GL_LIGHTING);
        glDisable(GL_BLEND);
        glDepthMask(GL_TRUE);
        glEnable(GL_DEPTH_TEST);
        glColorMaterial(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE);

        //-----------------------------------------------------------------------
        // Render bounding box, frame, collision detector. (opaque components)
        //-----------------------------------------------------------------------
        if (SECTION_RENDER_OPAQUE_PARTS_ONLY(a_options) && (!a_options.m_rendering_shadow))
        {
            // disable lighting
            glDisable(GL_LIGHTING);

            // render boundary box
            if (m_showBoundaryBox)
            {
                // set size on lines
                glLineWidth(1.0);

                // set color of boundary box
                glColor4fv(s_boundaryBoxColor.getData());

                // draw box line
                cDrawWireBox(m_boundaryBoxMin(0) , m_boundaryBoxMax(0) ,
                             m_boundaryBoxMin(1) , m_boundaryBoxMax(1) ,
                             m_boundaryBoxMin(2) , m_boundaryBoxMax(2) );
            }

            // render collision tree
            if (m_showCollisionDetector && (m_collisionDetector != NULL))
            {
                m_collisionDetector->render(a_options);
            }

            // enable lighting
            glEnable(GL_LIGHTING);
        }

        // render frame
        if (m_showFrame && (a_options.m_single_pass_only || a_options.m_render_opaque_objects_only))
        {
            glDisableClientState(GL_COLOR_ARRAY);
            glDisableClientState(GL_TEXTURE_COORD_ARRAY);
            glDisableClientState(GL_INDEX_ARRAY);
            glDisableClientState(GL_EDGE_FLAG_ARRAY);
            glDisable(GL_COLOR_MATERIAL);

            glEnable(GL_COLOR_MATERIAL);
            glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
            glColor4f(1.0,1.0,1.0,1.0);

            // set rendering properties
            glPolygonMode(GL_FRONT, GL_FILL);
                
            // draw frame
            cDrawFrame(m_frameSize, m_frameThicknessScale);
        }

        //-----------------------------------------------------------------------
        // Render graphical representation of object
        //-----------------------------------------------------------------------
        if (m_showEnabled)
        {
            // set polygon and face mode
            glPolygonMode(GL_FRONT_AND_BACK, m_triangleMode);

            // initialize line width
            glLineWidth(1.0f);

            /////////////////////////////////////////////////////////////////////
            // CREATING SHADOW DEPTH MAP
            /////////////////////////////////////////////////////////////////////
            if (a_options.m_creating_shadow_map)
            {
                glEnable(GL_CULL_FACE);
                glCullFace(GL_FRONT);

                // render object
                render(a_options);
                glDisable(GL_CULL_FACE);
            }

            /////////////////////////////////////////////////////////////////////
            // SINGLE PASS RENDERING
            /////////////////////////////////////////////////////////////////////
            else if (a_options.m_single_pass_only)
            {
                if (m_cullingEnabled)
                {
                    glEnable(GL_CULL_FACE);
                    glCullFace(GL_BACK);
                }
                else
                {
                    glDisable(GL_CULL_FACE);
                }

                if (m_useTransparency)
                {
                    glEnable(GL_BLEND);
                    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
                    glDepthMask(GL_FALSE);
                }
                else
                {
                    glDisable(GL_BLEND);
                    glDepthMask(GL_TRUE);
                }

                // render object
                render(a_options);

                // disable blending
                glDisable(GL_BLEND);
                glDepthMask(GL_TRUE);
            }


            /////////////////////////////////////////////////////////////////////
            // MULTI PASS RENDERING
            /////////////////////////////////////////////////////////////////////
            else
            {
                // opaque objects
                if (a_options.m_render_opaque_objects_only)
                {
                    if (m_cullingEnabled)
                    {
                        glEnable(GL_CULL_FACE);
                        glCullFace(GL_BACK);
                    }
                    else
                    {
                        glDisable(GL_CULL_FACE);
                    }

                    render(a_options);
                }

                // render transparent back triangles
                if (a_options.m_render_transparent_back_faces_only)
                {
                    if (m_useTransparency)
                    {
                        glEnable(GL_BLEND);
                        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
                        glDepthMask(GL_FALSE);
                    }
                    else
                    {
                        glDisable(GL_BLEND);
                        glDepthMask(GL_TRUE);
                    }

                    glEnable(GL_CULL_FACE);
                    glCullFace(GL_FRONT);
                    
                    render(a_options);

                    // disable blending
                    glDisable(GL_BLEND);
                    glDepthMask(GL_TRUE);
                }

                // render transparent front triangles
                if (a_options.m_render_transparent_front_faces_only)
                {
                    if (m_useTransparency)
                    {
                        glEnable(GL_BLEND);
                        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
                        glDepthMask(GL_FALSE);
                    }
                    else
                    {
                        glDisable(GL_BLEND);
                        glDepthMask(GL_TRUE);
                    }

                    glEnable(GL_CULL_FACE);
                    glCullFace(GL_BACK);

                    render(a_options);
    
                    // disable blending
                    glDisable(GL_BLEND);
                    glDepthMask(GL_TRUE);
                }
            }
        }
    }

    // render children
    for (unsigned int i=0; i<m_children.size(); i++)
    {
        m_children[i]->renderSceneGraph(a_options);
    }

    // pop current matrix
    glPopMatrix();

#endif
}


//==============================================================================
/*!
    This method adjusts the collision segment to take into consideration motion
    from an object. This feature is used by the finger-proxy algorithm to avoid
    "popping" through objects if their move toward the proxy while the proxy 
    remains till.

    \param  a_segmentPointA          Start point of segment.
    \param  a_segmentPointAadjusted  Adjusted start point of segment.
*/
//==============================================================================
void cGenericObject::adjustCollisionSegment(cVector3d& a_segmentPointA,
                                       cVector3d& a_segmentPointAadjusted)
{
    // convert point from local to global coordinates by using
    // the previous object position and orientation
    cVector3d point = cAdd(m_globalPos, cMul(m_globalRot, a_segmentPointA));

    // compute the new position of the point based on
    // the new object position and orientation
    a_segmentPointAadjusted = cMul( cTranspose(m_prevGlobalRot), cSub(point, m_prevGlobalPos));
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
cVector3d cGenericObject::computeInteractions(const cVector3d& a_toolPos,
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

    // check if node is a ghost. If yes, then ignore call
    if (m_ghostEnabled) { return (cVector3d(0,0,0)); }

    // process current object if enabled
    if (m_enabled)
    {
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

    // descend through the children
    vector<cGenericObject*>::iterator it;
    for (it = m_children.begin(); it < m_children.end(); it++)
    {
        cVector3d force = (*it)->computeInteractions(toolPosLocal,
                                                     toolVelLocal,
                                                     a_IDN,
                                                     a_interactions);
        localForce.add(force);
    }

    // convert the reaction force into my parent coordinates
    cVector3d m_globalForce = cMul(m_localRot, localForce);

    // return resulting force
    return (m_globalForce);
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
