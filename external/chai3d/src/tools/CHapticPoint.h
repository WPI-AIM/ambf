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
    \version   3.2.0 $Rev: 1869 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef cHapticPointH
#define cHapticPointH
//------------------------------------------------------------------------------
#include "audio/CAudioDevice.h"
#include "forces/CAlgorithmFingerProxy.h"
#include "forces/CAlgorithmPotentialField.h"
#include "world/CGenericObject.h"
#include "world/CShapeSphere.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
class cGenericTool;
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CHapticPoint.h

    \brief
    Implements a haptic point on a tool.
*/
//==============================================================================

//==============================================================================
/*!
    \class      cHapticPoint
    \ingroup    tools

    \brief
    This class implements a haptic point on a tool.

    \details
    cHapticPoint describes a haptic interaction point associated with a tool.
    A tool may combine one or more haptic points depending of its type.
    For instance a cursor type tool will typically contain a single haptic point,
    whereas a gripper type tool will contain at least two haptic points 
    (one for the finger and a second for the thumb).

    An haptic point stores a desired goal position and a current proxy position.
    The goal point represents the current position of the haptic device and 
    the proxy point follows the desired goal while respecting the constraints 
    (triangles) of the environment.

    Force rendering algorithms are assigned to each haptic point to compute
    the interaction force between the haptic point and the objects that
    compose the environment.
*/
//==============================================================================
class cHapticPoint
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cHapticPoint.
    cHapticPoint(cGenericTool* a_parentTool);

    //! Destructor of cHapticPoint.
    virtual ~cHapticPoint();


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - POSITION COMMANDS
    //--------------------------------------------------------------------------

public:

    //! This method resets the haptic point to the current position of the haptic device.
    void initialize();

    //! This method initializes the haptic point to a desired position.
    void initialize(cVector3d a_globalPos);

    //! This method returns a pointer to the parent tool.
    inline cGenericTool* getParentTool() { return (m_parentTool); }

    //! This method returns the current desired goal position of the haptic point in world coordinates.
    cVector3d getGlobalPosGoal();

    //! This method returns the the current proxy position of the haptic point in world coordinates.
    cVector3d getGlobalPosProxy();

    //! This method returns the current desired goal position of the haptic point in local tool coordinates.
    cVector3d getLocalPosGoal();

    //! This method returns the current proxy position of the haptic point in local tool coordinates.
    cVector3d getLocalPosProxy();


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - GENERAL SETTINGS
    //--------------------------------------------------------------------------

public:

    //! This method sets the radius size of the haptic point.
    void setRadius(double a_radius);

    //! This method sets the radius size of the haptic point.
    void setRadius(double a_radiusDisplay, 
                   double a_radiusContact);

    //! This method sets the radius size of the physical proxy.
    void setRadiusContact(double a_radiusContact);

    //! This method returns the radius size of the physical proxy.
    double getRadiusContact() { return (m_radiusContact); }

    //! This method sets the radius size of the sphere used to display the proxy and goal position
    void setRadiusDisplay(double a_radiusDisplay);

    //! This method returns the display radius size of the proxy and goal spheres.
    double getRadiusDisplay() { return (m_radiusDisplay); }

    //! This method sets the display options of the goal and proxy spheres
    void setShow(bool a_showProxy = true, 
                 bool a_showGoal = false,
                 cColorf a_colorLine = cColorf(0.5, 0.5, 0.5));

    //! This method Create an audio source for this haptic point.
    bool createAudioSource(cAudioDevice* a_audioDevice);


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - COLLISION EVENTS (MESH OBJECTS)
    //--------------------------------------------------------------------------

public:

    // This method returns the current number of collision events between this haptic point and the environment.
    inline int getNumCollisionEvents() { return (m_algorithmFingerProxy->getNumCollisionEvents()); }

    // This method returns the i'th collision collision event for this haptic point.
    inline cCollisionEvent* getCollisionEvent(const int a_index) { return (m_algorithmFingerProxy->m_collisionEvents[a_index]); }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - INTERACTION EVENTS (POTENTIAL FIELDS)
    //--------------------------------------------------------------------------

public:

    // This method returns the number of interaction events between this haptic point and the environment.
    inline int getNumInteractionEvents() { return (int)(m_algorithmPotentialField->m_interactionRecorder.m_interactions.size()); }

    // This method returns the i'th interaction events between this haptic point and the environment.
    inline cInteractionEvent* getInteractionEvent(const int a_index) { return (&(m_algorithmPotentialField->m_interactionRecorder.m_interactions[a_index])); }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - FORCE COMPUTATIONS
    //--------------------------------------------------------------------------

public:

    //! This method computes all interaction forces between the haptic point and the virtual environment.
    cVector3d computeInteractionForces(cVector3d& a_globalPos, 
                                       cMatrix3d& a_globalRot,
                                       cVector3d& a_globalLinVel,
                                       cVector3d& a_globalAngVel);

    //! This method returns the last computed force in global world coordinates.
    cVector3d getLastComputedForce() { return (m_lastComputedGlobalForce); }

    //! This method checks if the tool is touching a particular object.
    bool isInContact(cGenericObject* a_object);


    //--------------------------------------------------------------------------
    // PUBLIC MEMBERS - FORCE RENDERING ALGORITHMS
    //--------------------------------------------------------------------------

public:

    //! Finger-proxy algorithm used for modeling contacts with cMesh objects.
    cAlgorithmFingerProxy* m_algorithmFingerProxy;

    //! Potential field algorithm used for modeling interaction forces with objects that have haptic effects programmed.
    cAlgorithmPotentialField* m_algorithmPotentialField;


    //--------------------------------------------------------------------------
    // PUBLIC MEMBERS - GRAPHIC MODEL
    //--------------------------------------------------------------------------

public:

    //! Sphere object used for rendering the __proxy__ position. 
    cShapeSphere* m_sphereProxy;

    //! Sphere object used for rendering the __goal__ position.
    cShapeSphere* m_sphereGoal;

    //! Color of the display line that connects both spheres (__proxy__ and __goal__).
    cColorf m_colorLine;


    //--------------------------------------------------------------------------
    // PUBLIC METHODS
    //--------------------------------------------------------------------------

public:

    //! This method renders the haptic point graphically using OpenGL.
    void render(cRenderOptions& a_options);


    //--------------------------------------------------------------------------
    // PROTECTED METHODS
    //--------------------------------------------------------------------------

protected:

    //! This method updates the position of the spheres (__proxy__ and __goal__) in local tool coordinates
    void updateSpherePositions();


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS
    //--------------------------------------------------------------------------

protected:

    //! Parent tool.
    cGenericTool* m_parentTool;

    //! Last computed interaction force [N] in world global coordinates.
    cVector3d m_lastComputedGlobalForce;

    //! Radius used for rendering the proxy and goal spheres.
    double m_radiusDisplay;

    //! Radius used to model the physical radius of the proxy.
    double m_radiusContact;

    //! Pointer to mesh objects for which the proxy is in contact with.
    cGenericObject* m_meshProxyContacts[3];


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS - AUDIO
    //--------------------------------------------------------------------------

protected:

    //! Optional audio source for rendering sound impacts with environment.
    cAudioSource* m_audioSourceImpact[3];

    //! Optional audio source for rendering sound friction.
    cAudioSource* m_audioSourceFriction[3];

    //! If __true__ then audio sources are enabled.
    bool m_useAudioSources;

    //! Pointers to mesh objects for which the proxy was last in contact with.
    cGenericObject* m_audioProxyContacts[3];
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
