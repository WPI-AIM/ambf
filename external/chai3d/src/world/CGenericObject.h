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
    \version   3.2.0 $Rev: 2014 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CGenericObjectH
#define CGenericObjectH
//------------------------------------------------------------------------------
#include "collisions/CCollisionBasics.h"
#include "effects/CGenericEffect.h"
#include "forces/CInteractionBasics.h"
#include "graphics/CDraw3D.h"
#include "graphics/CColor.h"
#include "graphics/CDisplayList.h"
#include "graphics/CRenderOptions.h"
#include "materials/CMaterial.h"
#include "materials/CNormalMap.h"
#include "math/CMaths.h"
#include "math/CTransform.h"
#include "system/CGenericType.h"
//------------------------------------------------------------------------------
#include <vector>
#include <list>
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
class cGenericCollision;
class cGenericForceAlgorithm;
class cMultiMesh;
class cShaderProgram;
class cInteractionRecorder;
//------------------------------------------------------------------------------
typedef std::shared_ptr<cShaderProgram> cShaderProgramPtr;
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CGenericObject.h

    \brief
    Implements a base class for all objects.
*/
//==============================================================================

//==============================================================================
/*!
    \class      cGenericObject
    \ingroup    world

    \brief
    This class implements a base class for all 2D or 3D objects in CHAI3D.

    \details
    This class is the root of basically every render-able object in CHAI3D.
    It defines a reference frame (position and rotation) and virtual methods 
    for rendering, which are overloaded by useful subclasses. \n

    This class also defines basic methods for maintaining a scene graph, 
    and propagating rendering passes and reference frame changes through 
    a hierarchy of cGenericObjects. \n

    The most important methods to look at here are probably the virtual 
    methods, which are listed last in CGenericObject.h. These methods 
    will be called on each cGenericObject as operations propagate through 
    the scene graph.
*/
//==============================================================================
class cGenericObject : public cGenericType
{
    friend class cMultiMesh;

    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cGenericObject.
    cGenericObject();

    //! Destructor of cGenericObject.
    virtual ~cGenericObject();


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - GENERAL
    //--------------------------------------------------------------------------

public:

    //! This method enables or disable this object. When an object is disabled, both haptic and graphic rendering no longer occur.
    virtual void setEnabled(bool a_enabled,  const bool a_affectChildren = false);

    //! This method returns __true__ if the object is enabled, __false__ otherwise.
    bool getEnabled() const { return (m_enabled); }


    //-----------------------------------------------------------------------
    // PUBLIC METHODS - COPY:
    //-----------------------------------------------------------------------

public:

    //! This method creates a copy of itself.
    virtual cGenericObject* copy(const bool a_duplicateMaterialData = false,
        const bool a_duplicateTextureData = false, 
        const bool a_duplicateMeshData = false,
        const bool a_buildCollisionDetector = true) { return (NULL); }


    //-----------------------------------------------------------------------
    // PUBLIC METHODS - TRANSLATION AND ORIENTATION:
    //-----------------------------------------------------------------------

public:

    //! This method sets the local position of this object.
    virtual void setLocalPos(const cVector3d& a_localPos)
    {
        m_localPos = a_localPos;
    }

#ifdef C_USE_EIGEN
    //! This method sets the local position of this object.
    void setLocalPos(const Eigen::Vector3d& a_localPos)
    {
        setLocalPos(cVector3d(a_localPos[0], a_localPos[1], a_localPos[2]));
    }
#endif

    //! This method sets the local position of this object.
    void setLocalPos(const double a_x = 0.0,
        const double a_y = 0.0, 
        const double a_z = 0.0)
    {
        setLocalPos(cVector3d(a_x, a_y, a_z));
    }

    //! This method returns the local position of this object.
    inline cVector3d getLocalPos() const { return (m_localPos); }

    //! This method returns the global position of this object.
    inline cVector3d getGlobalPos() const { return (m_globalPos); }

    //! This method sets the local rotation matrix for this object.
    virtual void setLocalRot(const cMatrix3d& a_localRot)
    {
        m_localRot = a_localRot;
    }

#ifdef C_USE_EIGEN
    //! This method sets the local rotation matrix for this object.
    inline void setLocalRot(const Eigen::Matrix3d a_localRot)
    {
        cMatrix3d localRot;
        localRot.copyfrom(a_localRot);
        setLocalRot(localRot);
    }
#endif

    //! This method returns the local rotation matrix of this object.
    inline cMatrix3d getLocalRot() const { return (m_localRot); }

    //! This method returns the global rotation matrix of this object.
    inline cMatrix3d getGlobalRot() const { return (m_globalRot); }

    //! This method returns the local position and rotation matrix by passing a transformation matrix.
    inline void setLocalTransform(const cTransform& a_transform) 
    {
        setLocalPos(a_transform.getLocalPos());
        setLocalRot(a_transform.getLocalRot());
    }

    //! This method returns the local position and rotation matrix in a transformation matrix.
    inline cTransform getLocalTransform() { return (cTransform(m_localPos, m_localRot)); }

    //! This method returns the global position and rotation matrix in a transformation matrix.
    inline cTransform getGlobalTransform() { return (cTransform(m_globalPos, m_globalRot)); }

    //! This method translates this object by a specified offset.
    void translate(const cVector3d& a_translation);

    //! This method translates this object by a specified offset.
    void translate(const double a_x, 
        const double a_y, 
        const double a_z = 0.0);

    //! This method rotates this object around a local axis. Angle magnitude is defined in radians.
    void rotateAboutLocalAxisRad(const cVector3d& a_axis, const double a_angleRad);

    //! This method rotates this object around a local axis. Angle magnitude is defined in degrees.
    inline void rotateAboutLocalAxisDeg(const cVector3d& a_axis,
        const double a_angleDeg) 
    { 
        rotateAboutLocalAxisRad(a_axis, cDegToRad(a_angleDeg)); 
    }

    //! This method rotates this object around a local axis. Angle magnitude is defined in radians.
    inline void rotateAboutLocalAxisRad(const double a_axisX,
        const double a_axisY, 
        const double a_axisZ, 
        const double a_angleRad) 
    { 
        rotateAboutLocalAxisRad(cVector3d(a_axisX, a_axisY, a_axisZ), a_angleRad);
    }

    //! This method rotates this object around a local axis. Angle magnitude is defined in degrees.
    inline void rotateAboutLocalAxisDeg(const double a_axisX,
        const double a_axisY, 
        const double a_axisZ, 
        const double a_angleDeg) 
    { 
        rotateAboutLocalAxisRad(cVector3d(a_axisX, a_axisY, a_axisZ), cDegToRad(a_angleDeg));
    }

    //! This method rotates this object around a global axis. Angle magnitude is defined in radians.
    void rotateAboutGlobalAxisRad(const cVector3d& a_axis, const double a_angleRad);

    //! This method rotates this object around a global axis. Angle magnitude is defined in degrees.
    inline void rotateAboutGlobalAxisDeg(const cVector3d& a_axis, const double a_angleDeg) 
    { 
        rotateAboutGlobalAxisRad(a_axis, cDegToRad(a_angleDeg)); 
    }

    //! This method rotate this object around a local axis. Angle magnitude is defined in radians.
    inline void rotateAboutGlobalAxisRad(const double a_axisX,
        const double a_axisY, 
        const double a_axisZ, 
        const double a_angleRad) 
    { 
        rotateAboutGlobalAxisRad(cVector3d(a_axisX, a_axisY, a_axisZ), a_angleRad);
    }

    //! This method rotates this object around a local axis. Angle magnitude is defined in degrees.
    inline void rotateAboutGlobalAxisDeg(const double a_axisX, 
        const double a_axisY, 
        const double a_axisZ, 
        const double a_angleDeg) 
    { 
        rotateAboutGlobalAxisRad(cVector3d(a_axisX, a_axisY, a_axisZ), cDegToRad(a_angleDeg)); 
    }

    //! This method rotates this object using fixed Euler representation. Angles are defined in radians.
    void rotateExtrinsicEulerAnglesRad(const double& a_angleRad1,
        const double& a_angleRad2,
        const double& a_angleRad3,
        const cEulerOrder a_eulerOrder);

    //! This method rotates this object using fixed Euler representation. Angles are defined in radians.
    void rotateExtrinsicEulerAnglesDeg(const double& a_angleDeg1,
        const double& a_angleDeg2,
        const double& a_angleDeg3,
        const cEulerOrder a_eulerOrder) 
    { 
        rotateExtrinsicEulerAnglesRad(cDegToRad(a_angleDeg1), cDegToRad(a_angleDeg2), cDegToRad(a_angleDeg3), a_eulerOrder); 
    }

    //! This method rotates this object using co-moving Euler representation. Angles are defined in radians.
    void rotateIntrinsicEulerAnglesRad(const double& a_angleRad1,
        const double& a_angleRad2,
        const double& a_angleRad3,
        const cEulerOrder a_eulerOrder);

    //! This method rotates this object using co-moving Euler representation. Angles are defined in radians.
    void rotateIntrinsicEulerAnglesDeg(const double& a_angleDeg1,
        const double& a_angleDeg2,
        const double& a_angleDeg3,
        const cEulerOrder a_eulerOrder) 
    { 
        rotateIntrinsicEulerAnglesRad(cDegToRad(a_angleDeg1), cDegToRad(a_angleDeg2), cDegToRad(a_angleDeg3), a_eulerOrder); 
    }


    //-----------------------------------------------------------------------
    // PUBLIC METHODS - COMPUTING GLOBAL POSITIONS:
    //-----------------------------------------------------------------------

public:

    //! This method computes the global position and rotation of this object and its children.
    virtual void computeGlobalPositions(const bool a_frameOnly = true,
        const cVector3d& a_globalPos = cVector3d(0.0, 0.0, 0.0),
        const cMatrix3d& a_globalRot = cIdentity3d());

    //! This method computes the global position and rotation of current object only.
    void computeGlobalPositionsFromRoot(const bool a_frameOnly = true);


    //-----------------------------------------------------------------------
    // PUBLIC METHODS - HAPTIC EFFECTS:
    //-----------------------------------------------------------------------

public:

    //! This method adds a haptic effect to this object.
    bool addEffect(cGenericEffect* a_effect);

    //! This method removes a haptic effect from this object.
    bool removeEffect(cGenericEffect* a_effect);
    
    //! This method removes all haptic effects.
    void deleteAllEffects();

    //! This method creates a magnetic haptic effect.
    bool createEffectMagnetic();

    //! This method deletes any current magnetic haptic effect.
    bool deleteEffectMagnetic();

    //! This method creates a stick-and-slip haptic effect.
    bool createEffectStickSlip();

    //! This method delete any current stick-and-slip haptic effect.
    bool deleteEffectStickSlip();

    //! This method creates a surface haptic effect.
    bool createEffectSurface();

    //! This method deletes any current surface haptic effect.
    bool deleteEffectSurface();

    //! This method creates a vibration haptic effect.
    bool createEffectVibration();

    //! This method deletes any current vibration haptic effect.
    bool deleteEffectVibration();

    //! This method creates a viscous haptic effect.
    bool createEffectViscosity();

    //! This method deletes any current viscous haptic effect.
    bool deleteEffectViscosity();

    
    //-----------------------------------------------------------------------
    // PUBLIC METHODS - HAPTIC PROPERTIES:
    //-----------------------------------------------------------------------

public:

    //! This method enables or disables haptic perception of this object, optionally propagating the change to children.
    virtual void setHapticEnabled(const bool a_hapticEnabled, const bool a_affectChildren = true);

    //! This method returns the haptic status of object (__true__ means it can be felt when visible).
    inline bool getHapticEnabled() const { return (m_hapticEnabled); }

    //! This method sets the haptic stiffness of the object, optionally recursively affecting children.
    virtual void setStiffness(const double a_stiffness, const bool a_affectChildren = true);

    //! This method sets the static and dynamic friction properties (polygonal models only), optionally recursively affecting children.
    virtual void setFriction(double a_staticFriction, 
        double a_dynamicFriction, 
        const bool a_affectChildren = true);


    //-----------------------------------------------------------------------
    // PUBLIC METHODS - GRAPHIC PROPERTIES:
    //-----------------------------------------------------------------------

public:

    //! This method enables or disables the graphic display of this object, optionally propagating the change to children.
    virtual void setShowEnabled(const bool a_show, const bool a_affectChildren = true);

    //! This method returns the display status of object (true means it's visible).
    inline bool getShowEnabled() const { return (m_showEnabled); }

    //! This method enables or disables wireframe rendering, optionally propagating the operation to my children.
    virtual void setWireMode(const bool a_showWireMode, const bool a_affectChildren = false);

    //! This method returns whether wireframe rendering is enabled.
    inline bool getWireMode() const { return (m_triangleMode == GL_LINE); }

    //! This method enables or disables face-culling, optionally propagating the operation to my children.
    virtual void setUseCulling(const bool a_useCulling, const bool a_affectChildren = false);

    //! This method returns __true__ if face-culling is enabled, __false__ otherwise.
    inline bool getUseCulling() const { return (m_cullingEnabled); }


    //-----------------------------------------------------------------------
    // PUBLIC METHODS - TRANSPARENCY:
    //-----------------------------------------------------------------------

    //! This method enables or disables transparency.
    virtual void setUseTransparency(const bool a_useTransparency, const bool a_affectChildren = false);

    //! This method returns __true__ if transparency is enabled, __false__ otherwise.
    inline bool getUseTransparency() const { return m_useTransparency; }

    //! This method sets the transparency level of the object.
    virtual void setTransparencyLevel(const float a_level,
        const bool a_applyToVertices = false,
        const bool a_applyToTextures = false,
        const bool a_affectChildren = false);


    //-----------------------------------------------------------------------
    // PUBLIC METHODS - DISPLAY LISTS:
    //-----------------------------------------------------------------------

public:

    //! This method enabled or disables the use of a display list for rendering, optionally propagating the operation to its children.
    virtual void setUseDisplayList(const bool a_useDisplayList, const bool a_affectChildren = false);

    //! This method returns __true__ if a display list is activated, __false__ otherwise.
    inline bool getUseDisplayList() const { return (m_useDisplayList); }

    //! This method invalidates any existing display lists, optionally propagating the operation to its children.
    virtual void markForUpdate(const bool a_affectChildren = false);


    //-----------------------------------------------------------------------
    // PUBLIC METHODS - VERTEX COLORS:
    //-----------------------------------------------------------------------

public:

    //! This method enables or disables the use of per-vertex colors, optionally propagating the operation to its children.
    virtual void setUseVertexColors(const bool a_useColors, const bool a_affectChildren = false);

    //! This method returns __true__ is per-vertex color properties are enabled, __false__ otherwise.
    inline bool getUseVertexColors() const { return (m_useVertexColors); }


    //-----------------------------------------------------------------------
    // PUBLIC METHODS - MATERIAL PROPERTIES:
    //-----------------------------------------------------------------------

public:

    //! This method enables or disables the use of material properties, optionally propagating the operation to its children.
    virtual void setUseMaterial(const bool a_useMaterial, const bool a_affectChildren = false);

    //! This method returns __true__ is material properties are enabled, __false__ otherwise.
    inline bool getUseMaterial() const { return (m_useMaterialProperty); }

    //! This method sets the material properties of this object, optionally propagating the operation to its children.
    virtual void setMaterial(cMaterialPtr a_material, const bool a_affectChildren = false);

    //! This method setd the material properties of this object, optionally propagating the operation to its children.
    virtual void setMaterial(cMaterial& a_material, const bool a_affectChildren = false);

    //! This method creates a backup of the material colors of this object, optionally propagating the operation to its children.
    virtual void backupMaterialColors(const bool a_affectChildren = false);

    //! This method restores the material color properties of this object from a previous backup, optionally propagating the operation to its children.
    virtual void restoreMaterialColors(const bool a_affectChildren = false);


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - TEXTURE PROPERTIES:
    //-----------------------------------------------------------------------

public:

    //! This method enables or disables the use of texture-mapping, optionally propagating the operation to its children.
    virtual void setUseTexture(const bool a_useTexture, const bool a_affectChildren = false);

    //! This method returns __true__ if texture-mapping is enabled, __false__ otherwise.
    inline bool getUseTexture() const { return (m_useTextureMapping); }

    //! This method sets a texture to this object, optionally propagating the operation to its children.
    virtual void setTexture(cTexture1dPtr a_texture, const bool a_affectChildren = false);


    //-----------------------------------------------------------------------
    // PUBLIC METHODS - SHADERS:
    //-----------------------------------------------------------------------

public:

    //! This method assigns a shader program to this object, optionally propagating the operation to its children..
    virtual void setShaderProgram(cShaderProgramPtr a_shaderProgram, const bool a_affectChildren = false);

    //! This method returns a pointer to the current shader program.
    virtual cShaderProgramPtr getShaderProgram() { return (m_shaderProgram); }


    //-----------------------------------------------------------------------
    // PUBLIC MEMBERS - BOUNDARY BOX:
    //-----------------------------------------------------------------------

public:

    //! Color of the boundary box.
    static cColorf s_boundaryBoxColor;


    //-----------------------------------------------------------------------
    // PUBLIC METHODS - BOUNDARY BOX:
    //-----------------------------------------------------------------------

public:

    //! This method enables or disabled the graphic display of the boundary box for this object, optionally propagating the change to its children.
    virtual void setShowBoundaryBox(const bool a_showBoundaryBox, const bool a_affectChildren = false);

    //! This method returns __true__ if the boundary box is being displayed, __false__ otherwise.
    inline bool getShowBoundaryBox() const { return (m_showBoundaryBox); }

    //! This method returns the minimum point of this object's boundary box.
    inline cVector3d getBoundaryMin() const { return (m_boundaryBoxMin); }

    //! This method returns the maximum point of this object's boundary box.
    inline cVector3d getBoundaryMax() const { return (m_boundaryBoxMax); }

    //! This method computes and returns the center of this object's boundary box.
    inline cVector3d getBoundaryCenter() const { return (m_boundaryBoxMax + m_boundaryBoxMin)/2.0; }

    //! This method returns __true__, if the boundary box is empty, otherwise __false__.
    inline bool getBoundaryBoxEmpty() { return (m_boundaryBoxEmpty); }

    //! This method computes this object's boundary box, optionally forcing it to bound child objects.
    virtual void computeBoundaryBox(const bool a_includeChildren = true);


    //-----------------------------------------------------------------------
    // PUBLIC METHODS - REFERENCE FRAME REPRESENTATION:
    //-----------------------------------------------------------------------

public:

    //! This method enables or disables the graphic display of the reference frame arrows for this object, optionally propagating the change to its children.
    virtual void setShowFrame(const bool a_showFrame, const bool a_affectChildren  = false);

    //! This method returns __true__ if the display of the reference frame is enabled, __false__ otherwise.
    inline bool getShowFrame(void) const { return (m_showFrame); }

    //! This method sets the size of the rendered reference frame, optionally propagating the change to its children.
    virtual void setFrameSize(const double a_size = 1.0, const bool a_affectChildren = false);

    //! This method returns the size of the graphical reference frame.
    inline double getFrameSize() const { return (m_frameSize); }


    //-----------------------------------------------------------------------
    // PUBLIC METHODS - COLLISION DETECTION:
    //-----------------------------------------------------------------------

public:

    //! This method sets a collision detector to this current object.
    void setCollisionDetector(cGenericCollision* a_collisionDetector) { m_collisionDetector = a_collisionDetector; }

    //! This method returns a pointer to this object's current collision detector.
    inline cGenericCollision* getCollisionDetector() const { return (m_collisionDetector); }

    //! This method deletes any existing collision detector.
    virtual void deleteCollisionDetector(const bool a_affectChildren = false);

    //! This method computes any collision between a segment and this object.
    virtual bool computeCollisionDetection(const cVector3d& a_segmentPointA,
        const cVector3d& a_segmentPointB,
        cCollisionRecorder& a_recorder,
        cCollisionSettings& a_settings);

    //! This method enables or disables the display of the collision detector, optionally propagating the change to its children.
    virtual void setShowCollisionDetector(const bool a_showCollisionDetector, const bool a_affectChildren = false);

    //! This method returns __true__ if the collision detector is being displayed graphically, __false__ otherwise.
    inline bool getShowCollisionDetector() { return (m_showCollisionDetector); }

    //! This method sets the collision detector graphic display properties.
    virtual void setCollisionDetectorProperties(unsigned int a_displayDepth, 
        cColorf& a_color, 
        const bool a_affectChildren = false);


    //-----------------------------------------------------------------------
    // PUBLIC METHODS - SCENE GRAPH:
    //-----------------------------------------------------------------------

public:

    //! This method sets the parent of this object.
    inline void setParent(cGenericObject* a_parent) { m_parent = a_parent; }

    //! This method returns the parent of this object.
    inline cGenericObject* getParent() const { return (m_parent); }

    //! This method sets a link to an object that owns this object. This could be a super parent for instance.
    inline void setOwner(cGenericObject* a_owner) { m_owner = a_owner; }

    //! This method returns the owner of this object.
    inline cGenericObject* getOwner() { return (m_owner); }

    //! This method returns a selected child from the list of children.
    inline cGenericObject* getChild(const unsigned int a_index) const { return (m_children[a_index]); }

    //! This method add an object to the list of children.
    bool addChild(cGenericObject* a_object);

    //! This method removes an object from the list of children, without deleting it.
    bool removeChild(cGenericObject* a_object);

    //! This method removes this object from its parent's list of children.
    bool removeFromGraph();

    //! This method removes an object from its list of children and deletes it.
    bool deleteChild(cGenericObject *a_object);

    //! This method clears all objects from its list of children, without deleting them.
    void clearAllChildren();

    //! This method clears and delete all objects from its list of children.
    void deleteAllChildren();

    //! This method returns the number of children from its list of children.
    inline unsigned int getNumChildren() { return ((unsigned int)m_children.size()); }

    //! This method returns the total number of descendants, optionally including this object.
    inline unsigned int getNumDescendants(bool a_includeCurrentObject = false);


    //-----------------------------------------------------------------------
    // PUBLIC METHODS - GHOSTING:
    //-----------------------------------------------------------------------

public:

    //! This method enables or disables this object to be a ghost node.
    void setGhostEnabled(bool a_ghostEnabled) { m_ghostEnabled = a_ghostEnabled; }

    //! This method returns __truee__ if this object is a ghost node.
    bool getGhostEnabled() { return (m_ghostEnabled); }


    //-----------------------------------------------------------------------
    // PUBLIC METHODS - GEOMETRY:
    //-----------------------------------------------------------------------

public:

    //! This method scales the size of this object.
    virtual void scale(const double& a_scaleFactor, const bool a_affectChildren = true);


    //-----------------------------------------------------------------------
    // PUBLIC MEMBERS - PROPERTIES:
    //-----------------------------------------------------------------------

public: 

    //! Name of current object (filename).
    std::string m_name;

    //! Material property.
    cMaterialPtr m_material;

    //! Texture property.
    cTexture1dPtr m_texture;

    //! Normal map property.
    cNormalMapPtr m_normalMap;


    //-----------------------------------------------------------------------
    // PUBLIC MEMBERS - CUSTOM USER DATA:
    //-----------------------------------------------------------------------

public:

    //! An arbitrary tag, not used by CHAI3D.
    int m_userTag;

    //! An arbitrary data pointer, not used by CHAI3D.
    void* m_userData;

    //! Name of current object, not used by CHAI3D.
    std::string m_userName;

    //! A link to an external cGenericObject object, not used by CHAI3D.
    cGenericObject* m_userExternalObject;


    //-----------------------------------------------------------------------
    // PROTECTED MEMBERS - SCENEGRAPH:
    //-----------------------------------------------------------------------

protected:

    //! Parent object.
    cGenericObject* m_parent;

    /*!
        For most objects this value is initialized to point to the object itself.
        In the case of cMultiMesh, all mesh objects contained in cMultimesh are
        owned by their parent (cMultiMesh). 
    */
    cGenericObject* m_owner;

    //! List of children.
    std::vector<cGenericObject*> m_children;


    //-----------------------------------------------------------------------
    // PROTECTED MEMBERS - POSITION & ORIENTATION:
    //-----------------------------------------------------------------------

protected:

    //! The position of this object in my parent's reference frame.
    cVector3d m_localPos;

    //! The position of this object in the world's reference frame.
    cVector3d m_globalPos;

    //! The rotation matrix that rotates my reference frame into my parent's reference frame.
    cMatrix3d m_localRot;

    //! The rotation matrix that rotates my reference frame into the world's reference frame.
    cMatrix3d m_globalRot;

    //! Previous position since last haptic computation.
    cVector3d m_prevGlobalPos;

    //! Previous rotation since last haptic computation.
    cMatrix3d m_prevGlobalRot;


    //-----------------------------------------------------------------------
    // PROTECTED MEMBERS - BOUNDARY BOX
    //-----------------------------------------------------------------------

protected:

    //! Minimum position of boundary box.
    cVector3d m_boundaryBoxMin;

    //! Maximum position of boundary box.
    cVector3d m_boundaryBoxMax;

    //! If __true__, then the boundary box does not include any object.
    bool m_boundaryBoxEmpty;


    //-----------------------------------------------------------------------
    // PROTECTED MEMBERS - FRAME REPRESENTATION [X,Y,Z]:
    //-----------------------------------------------------------------------

protected:

    //! Size of graphical representation of frame (X-Y-Z).
    double m_frameSize;

    //! Pen thickness of graphical representation of frame (X-Y-Z).
    double m_frameThicknessScale;


    //-----------------------------------------------------------------------
    // PROTECTED MEMBERS - GENERAL OPTIONS:
    //-----------------------------------------------------------------------

protected:

    //! If __true__, the object may be rendered graphically and haptically.
    bool m_enabled;

    //! If __true__, this object is rendered.
    bool m_showEnabled;

    //! If __true__, this object can be felt.
    bool m_hapticEnabled;

    //! If __true__, object is enabled as ghost. 
    bool m_ghostEnabled;

    //! If __true__, this object's reference frame is rendered as a set of arrows.
    bool m_showFrame;

    //! If __true__, this object's boundary box is displayed as a set of lines.
    bool m_showBoundaryBox;

    //! If __true__, the collision detector is displayed (if available) at this node.
    bool m_showCollisionDetector;

    //! Should texture mapping be used?
    bool m_useTextureMapping;

    //! Should material properties be used?
    bool m_useMaterialProperty;

    //! Should per-vertex colors be used?
    bool m_useVertexColors;

    //! Should we use a display list to render this mesh?
    bool m_useDisplayList;

    //! Basic display list for current object.
    cDisplayList m_displayList;

    //! The polygon rendering mode (GL_FILL or GL_LINE).
    int m_triangleMode;

    /*!
        If __true__, transparency is enabled... this turns alpha on when the mesh is
        rendered, and - if multipass transparency is enabled in the rendering camera -
        uses the camera's multiple rendering passes to approximate back-to-front
        sorting via culling.
    */
    bool m_useTransparency;

    /*!
        Should culling be used when rendering triangles? \n

        Note that this option only applies when multipass transparency is
        disabled or during the non-transparent rendering pass when multipass
        transparency is enabled... \n

        Also note that currently only back-faces are culled during non-transparent
        rendering; you can't cull front-faces.
    */
    bool m_cullingEnabled;

    //! Default material property.
    static cMaterialPtr s_defaultMaterial;

    //! Shader program.
    cShaderProgramPtr m_shaderProgram;

    //! OpenGL matrix describing my position and orientation transformation.
    cTransform m_frameGL;


    //-----------------------------------------------------------------------
    // PROTECTED MEMBERS - COLLISION DETECTION:
    //-----------------------------------------------------------------------

protected:

    //! The collision detector used to test for contact with this object.
    cGenericCollision* m_collisionDetector;


    //-----------------------------------------------------------------------
    // PROTECTED MEMBERS - HAPTIC EFFECTS AND INTERACTIONS:
    //-----------------------------------------------------------------------

protected:

    //! List of haptic effects programmed for this object.
    std::vector<cGenericEffect*> m_effects;


    //-----------------------------------------------------------------------
    // PROTECTED VIRTUAL METHODS:
    //-----------------------------------------------------------------------

protected:

    //! This method renders this object graphically using OpenGL.
    virtual void render(cRenderOptions& a_options);

    //! This method update the global position information about this object.
    virtual void updateGlobalPositions(const bool a_frameOnly) {};

    //! This method updates the boundary box of this object.
    virtual void updateBoundaryBox() {};

    //! This method scales the size of this object with given scale factor.
    virtual void scaleObject(const double& a_scaleFactor) { m_boundaryBoxMin.mul(a_scaleFactor); m_boundaryBoxMax.mul(a_scaleFactor);}

    //! This method updates the geometric relationship between the tool and the current object.
    virtual void computeLocalInteraction(const cVector3d& a_toolPos,
        const cVector3d& a_toolVel,
        const unsigned int a_IDN);

    //! This method computes any additional interactions between the object and the tools.
    virtual cVector3d computeOtherInteractions(const cVector3d& a_toolPos,
        const cVector3d& a_toolVel,
        const unsigned int a_IDN,
        cInteractionRecorder& a_interactions) { return cVector3d(0,0,0); }

    //! This method computes any additional collisions other than the ones computed by the default collision detector.
    virtual bool computeOtherCollisionDetection(cVector3d& a_segmentPointA,
        cVector3d& a_segmentPointB,
        cCollisionRecorder& a_recorder,
        cCollisionSettings& a_settings) {return(false);}


    //-----------------------------------------------------------------------
    // PROTECTED METHODS:
    //-----------------------------------------------------------------------

protected:

    //! This method copies all properties of the current generic object to another.
    void copyGenericObjectProperties(cGenericObject* a_objDest, 
        const bool a_duplicateMaterialData,
        const bool a_duplicateTextureData, 
        const bool a_duplicateMeshData,
        const bool a_buildCollisionDetector);


    //-----------------------------------------------------------------------
    // PUBLIC MEMBERS - INTERACTIONS:
    //-----------------------------------------------------------------------

public: 

    //! Projection of the most recent haptic point (tool) onto the surface of the virtual object.
    cVector3d m_interactionPoint;

    //! Surface normal at the current interaction point.
    cVector3d m_interactionNormal;

    //! Was the last tool (haptic point) located inside the object?
    bool m_interactionInside;


    //-----------------------------------------------------------------------
    // PUBLIC MEMBERS - GENERAL
    //-----------------------------------------------------------------------

public:

    //! This method renders the entire scene graph, starting from this object.
    virtual void renderSceneGraph(cRenderOptions& a_options);

    //! This method adjusts the collision segment to handle objects in motion.
    virtual void adjustCollisionSegment(cVector3d& a_segmentPointA, cVector3d& a_segmentPointAadjusted);

    //! This method computes all haptic interaction between a tool and this object using the haptic effects.
    virtual cVector3d computeInteractions(const cVector3d& a_toolPos,
        const cVector3d& a_toolVel,
        const unsigned int a_IDN,
        cInteractionRecorder& a_interactions);
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------

