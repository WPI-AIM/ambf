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
    \version   3.2.0 $Rev: 2164 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CCameraH
#define CCameraH
//------------------------------------------------------------------------------
#include "audio/CAudioDevice.h"
#include "world/CGenericObject.h"
#include "math/CMaths.h"
#include "graphics/CImage.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
class cWorld;
class cDirectionalLight;
//------------------------------------------------------------------------------

enum cStereoMode
{
    C_STEREO_DISABLED,
    C_STEREO_ACTIVE,
    C_STEREO_PASSIVE_LEFT_RIGHT,
    C_STEREO_PASSIVE_TOP_BOTTOM,
    C_STEREO_PASSIVE_DUAL_DISPLAY
};

enum cEyeMode
{
    C_STEREO_LEFT_EYE,
    C_STEREO_RIGHT_EYE
};

//==============================================================================
/*!
    \file       CCamera.h

    \brief
    Implementation of a camera.
*/
//==============================================================================

//==============================================================================
/*!
    \class      cCamera
    \ingroup    display

    \brief
    This class implements a virtual camera.

    \details
    cCamera implements a virtual camera located inside the world.
    Its job is to set up the OpenGL projection matrix for the current
    OpenGL rendering context. The default camera looks down the negative
    x-axis.

    cCamera also includes front and back layers for rendering 2D widgets.
    The back layer is rendered first, followed by the main scenegraph
    (world) containing all 3d objects. Finally the front layer is rendered
    at the very end. Layers are rendered through an orthographic projection matrix,
    so the positive z axis faces the camera. Depth is currently not used.
    Lighting is disabled during rendering.
*/
//==============================================================================
class cCamera : public cGenericObject
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cCamera
    cCamera(cWorld* a_parentWorld);

    //! Destructor of cCamera
    virtual ~cCamera();


    //-----------------------------------------------------------------------
    // PUBLIC METHODS - GENERAL:
    //-----------------------------------------------------------------------

public:

    //! This method returns a pointer to the parent world.
    cWorld* getParentWorld() { return (m_parentWorld); }

    //! This method sets parent world.
    void setParentWorld(cWorld* a_world) {m_parentWorld = a_world;}


    //-----------------------------------------------------------------------
    // PUBLIC METHODS - MOUSE SELECTION:
    //-----------------------------------------------------------------------

public:

    //! This method queries whether the specified position is 'pointing at' any objects in the world.
    virtual bool selectWorld(const int a_windowPosX, const int a_windowPosY,
        const int a_windowWidth, const int a_windowHeight,
        cCollisionRecorder& a_collisionRecorder,
        cCollisionSettings& a_collisionSettings);

    //! This method queries whether the specified position is 'pointing at' any widget on the front layer.
    virtual bool selectFrontLayer(const int a_windowPosX, const int a_windowPosY,
        const int a_windowWidth, const int a_windowHeight,
        cCollisionRecorder& a_collisionRecorder,
        cCollisionSettings& a_collisionSettings);

    //! This method queries whether the specified position is 'pointing at' any widget on the back layer.
    virtual bool selectBackLayer(const int a_windowPosX, const int a_windowPosY,
        const int a_windowWidth, const int a_windowHeight,
        cCollisionRecorder& a_collisionRecorder,
        cCollisionSettings& a_collisionSettings);

    //! This method queries whether the specified position is 'pointing at' any widget on selected layers.
    virtual bool selectLayers(const int a_windowPosX, const int a_windowPosY,
        const int a_windowWidth, const int a_windowHeight,
        cCollisionRecorder& a_collisionRecorder,
        cCollisionSettings& a_collisionSettings,
        bool a_checkFrontLayer = true,
        bool a_checkBackLayer = true);


    //-----------------------------------------------------------------------
    // PUBLIC METHODS - POSITION & ORIENTATION (CARTESIAN)
    //-----------------------------------------------------------------------

public:

    //! This method set the position and orientation of the camera.
    virtual bool set(const cVector3d& a_localPosition,
                     const cVector3d& a_localLookAt,
                     const cVector3d& a_localUp);

    //! This method returns the camera "look at" position vector for this camera.
    cVector3d getLookVector()  const { return (-m_localRot.getCol0()); }

    //! This method returns the "up" vector for this camera.
    cVector3d getUpVector()    const { return (m_localRot.getCol2()); }

    //! This method returns the "right direction" vector for this camera.
    cVector3d getRightVector() const { return (m_localRot.getCol1()); }


    //-----------------------------------------------------------------------
    // PUBLIC METHODS - POSITION & ORIENTATION (SPHERICAL)
    //-----------------------------------------------------------------------

public:

    //! This method sets the position and orientation of the camera in spherical coordinates in radians.
    void setSphericalRad(const double& a_radius,
                         const double& a_polarRad,
                         const double& a_azimuthRad);

    //! This method sets the position and orientation of the camera in spherical coordinates in degrees.
    void setSphericalDeg(const double& a_radius,
                         const double& a_polarDeg,
                         const double& a_azimuthDeg);

    //! This method sets the reference vectors that define the basis of the spherical coordinate system.
    void setSphericalReferences(const cVector3d& a_originReference,
                                const cVector3d& a_zenithReference,
                                const cVector3d& a_azimuthReference);

    //! This method sets the __radius__ position of the camera (spherical coordinates).
    void setSphericalRadius(const double& a_radius);

    //! This method returns the __radius__ position of the camera (spherical coordinates).
    double getSphericalRadius() { return (m_posRadius); }

    //! This method sets the __polar__ position of the camera in __radians__ (spherical coordinates).
    void setSphericalPolarRad(const double& a_polarRad);

    //! This method returns the __polar__ position of the camera in __radians__ (spherical coordinates).
    double getSphericalPolarRad() { return (m_posPolarRad); }

    //! This method sets the __polar__ position of the camera in __degrees__ (spherical coordinates).
    void setSphericalPolarDeg(const double& a_polarDeg);

    //! This method returns the __polar__ position of the camera in __degrees__ (spherical coordinates).
    double getSphericalPolarDeg() { return (cRadToDeg(m_posPolarRad)); }

    //! This method sets the __azimuth__ position of the camera in __radians__ (spherical coordinates).
    void setSphericalAzimuthRad(const double& a_azimuthRad);

    //! This method returns the __azimuth__ position of the camera in __radians__ (spherical coordinates).
    double getSphericalAzimuthRad() { return (m_posAzimuthRad); }

    //! This method sets the __azimuth__ position of the camera in __degrees__ (spherical coordinates).
    void setSphericalAzimuthDeg(const double& a_azimuthDeg);

    //! This method returns the __azimuth__ position of the camera in __degrees__ (spherical coordinates).
    double getSphericalAzimuthDeg() { return (cRadToDeg(m_posAzimuthRad)); }

    //! This method sets the __zenith__ reference direction vector of the camera (spherical coordinates).
    void setSphericalZenithReference(const cVector3d& a_zenithReference);

    //! This method returns the __zenith__ reference direction vector of the camera (spherical coordinates).
    cVector3d getSphericalZenithReference() { return (m_zenithReference); }

    //! This method sets the __azimuth__ reference direction vector of the camera (spherical coordinates).
    void setSphericalAzimuthReference(const cVector3d& a_azimuthReference);

    //! This method returns the __azimuth__ reference direction vector of the camera (spherical coordinates).
    cVector3d getSphericalAzimuthReference() { return (m_azimuthReference); }

    //! This method sets the __origin__ target point of the camera (spherical coordinates).
    void setSphericalOriginReference(const cVector3d& a_originReference);

    //! This method returns the __origin__ target point of the camera (spherical coordinates).
    cVector3d getSphericalOriginReference() { return (m_originReference); }


    //-----------------------------------------------------------------------
    // PUBLIC MEMBER - MATRICES:
    //-----------------------------------------------------------------------

public:

    //! Projection matrix of camera.
    cTransform m_projectionMatrix;

    //! Modelview matrix of camera.
    cTransform m_modelViewMatrix;

    //! If __true__, then use projection matrix specified in m_projectionMatrix.
    bool m_useCustomProjectionMatrix;

    //! If __true__, then use projection matrix specified in m_projectionMatrix.
    bool m_useCustomModelViewMatrix;


    //-----------------------------------------------------------------------
    // PUBLIC METHODS - CLIPPING PLANES:
    //-----------------------------------------------------------------------

public:

    //! This method sets the near and far clipping plane distances.
    void setClippingPlanes(const double a_distanceNear, const double a_distanceFar);

    //! This method returns the near clipping plane distance.
    double getNearClippingPlane() { return (m_distanceNear); }

    //! This method returns the far clipping plane distance.
    double getFarClippingPlane() { return (m_distanceFar); }

    //! This method automatically adjust back and front clipping planes.
    void adjustClippingPlanes();


    //-----------------------------------------------------------------------
    // PUBLIC METHODS - FIELD OF VIEW & OPTICS:
    //-----------------------------------------------------------------------

public:

    //! This method sets the camera in orthographic mode
    void setOrthographicView(double a_viewWidth);

    //! This method gets the camera's orthographic view width
    double getOrthographicViewWidth(){ return m_orthographicWidth;}

    //! This method sets the field of view angle (in degrees).
    void setFieldViewAngleDeg(double a_fieldViewAngleDeg);

    //! This method returns the field of view angle (in degrees).
    double getFieldViewAngleDeg() { return (m_fieldViewAngleDeg); }

    //! This method set the field of view angle (in radians).
    void setFieldViewAngleRad(double a_fieldViewAngleRad) { setFieldViewAngleDeg(cRadToDeg(a_fieldViewAngleRad)); }

    //! This method returns the field of view angle (in radians).
    double getFieldViewAngleRad() { return (cDegToRad(m_fieldViewAngleDeg)); }

    //! This method returns the aspect ratio.
    double getAspectRatio();

    //! This method sets the stereo focal length.
    void setStereoFocalLength(double a_stereoFocalLength);

    //! This method returns the stereo focal length.
    double getStereoFocalLength() { return (m_stereoFocalLength); }

    //! This method sets the stereo eye separation.
    void setStereoEyeSeparation(double a_stereoEyeSeparation);

    //! This method returns the stereo eye separation.
    double getStereoEyeSeparation() { return (m_stereoEyeSeparation); }

    //! This method returns if the view mode is perspective or not for this camera
    bool isViewModePerspective() {return m_perspectiveMode;}


    //-----------------------------------------------------------------------
    // PUBLIC METHODS - RENDERING AND IMAGING:
    //-----------------------------------------------------------------------

public:

    //! This method renders the the camera view in OpenGL
    virtual void renderView(const int a_windowWidth,
        const int a_windowHeight,
        const cEyeMode a_eyeMode = C_STEREO_LEFT_EYE,
        const bool a_defaultBuffer = true);

    //! This method copies the output image data to an image structure.
    void copyImageBuffer(cImagePtr a_image);

    //! This method enables or disables additional rendering passes for transparency.
    virtual void setUseMultipassTransparency(bool a_enabled);

    //! This method returns __true__ if multipass rendering is enabled, __false__ otherwise.
    bool getUseMultipassTransparency() { return (m_useMultipassTransparency); }

    //! This method returns the width of the current window display in pixels.
    int getDisplayWidth() { return (m_lastDisplayWidth); }

    //! This method returns the height of the current window display in pixels.
    int getDisplayHeight() { return (m_lastDisplayHeight); }

    //! This method resets textures and display lists for the world associated with this camera.
    void updateGPU();


    //-----------------------------------------------------------------------
    // PUBLIC METHODS - STEREO:
    //-----------------------------------------------------------------------

public:

    //! This method sets the desired stereo mode.
    virtual void setStereoMode(cStereoMode a_stereoMode);

    //! This method returns the current stereo mode being used.
    cStereoMode getStereoMode() { return (m_stereoMode); }


    //-----------------------------------------------------------------------
    // PUBLIC METHODS - MIRRORING:
    //-----------------------------------------------------------------------

public:

    //! This method enables or disables output image mirroring horizontally.
    void setMirrorHorizontal(bool a_enabled);

    //! This method enables or disables output image mirroring vertically.
    void setMirrorVertical(bool a_enabled);

    //! This method returns __true__ then output image is mirrored horizontally, __false__ otherwise.
    bool getMirrorHorizontal() { return (m_mirrorHorizontal); }

    //! This method returns __true__ then output image is mirrored vertically, __false__ otherwise.
    bool getMirrorVertical() { return (m_mirrorVertical); }


    //-----------------------------------------------------------------------
    // PUBLIC METHODS - AUDIO DEVICE:
    //-----------------------------------------------------------------------

public:

    //! This method attaches an audio device to the camera.
    void attachAudioDevice(cAudioDevice* a_audioDevice) { m_audioDevice = a_audioDevice; }

    //! This method detaches the current audio device from the camera.
    void detachAudioDevice() { m_audioDevice = NULL; }


    //-----------------------------------------------------------------------
    // PUBLIC MEMBERS - FRONT AND BACK PLANES: (WIDGETS)
    //-----------------------------------------------------------------------

public:

    //! Front plane scene graph which can be used to attach widgets.
    cWorld* m_frontLayer;

    //! Black plane scene graph which can be used to attach widgets.
    cWorld* m_backLayer;

    //! Directional light for front plane.
    cDirectionalLight* m_lightFrontLayer;

    //! Directional light for back plane.
    cDirectionalLight* m_lightBackLayer;


    //-----------------------------------------------------------------------
    // PROTECTED MEMBERS:
    //-----------------------------------------------------------------------

protected:

    //! Parent world.
    cWorld *m_parentWorld;

    //! Distance to near clipping plane.
    double m_distanceNear;

    //! Distance to far clipping plane.
    double m_distanceFar;

    //! Field of view angle expressed in degrees.
    double m_fieldViewAngleDeg;

    //! Width of orthographic view.
    double m_orthographicWidth;

    //! If __true__, then camera operates in perspective mode. If __false__, then camera is in orthographic mode.
    bool m_perspectiveMode;

    //! If __true__, then three rendering passes are performed to approximate back-front sorting (see comment).
    bool m_useMultipassTransparency;

    //! If __true__, then shadow casting is used.
    bool m_useShadowCasting;

    //! Stereo focal length.
    double m_stereoFocalLength;

    //! Stereo eye separation.
    double m_stereoEyeSeparation;

    //! Stereo rendering mode.
    cStereoMode m_stereoMode;

    //! Last width size of the window.
    unsigned int m_lastDisplayWidth;

    //! Last height size of the window.
    unsigned int m_lastDisplayHeight;

    //! If __true__ then a display reset has been requested.
    bool m_markForUpdate;

    //! If __true__ then the output image is mirrored horizontally.
    bool m_mirrorHorizontal;

    //! If __true__ then the output image is mirrored vertically.
    bool m_mirrorVertical;

    //! If __true__ then only one of the axes is mirrored.
    bool m_mirrorStatus;

    //! Scale factor used for horizontal mirroring. (-1.0 or 1.0)
    double m_scaleH;

    //! Scale factor used for vertical mirroring. (-1.0 or 1.0)
    double m_scaleV;

    //! Camera polar position in radians (spherical coordinates).
    double m_posPolarRad;

    //! Camera azimuth position in radians (spherical coordinates).
    double m_posAzimuthRad;

    //! Camera radius position (spherical coordinates).
    double m_posRadius;

    //! Camera zenith reference vector (spherical coordinates).
    cVector3d m_zenithReference;

    //! Camera azimuth reference vector (spherical coordinates).
    cVector3d m_azimuthReference;

    //! Camera origin (spherical coordinates).
    cVector3d m_originReference;

    //! Optionally attached audio device.
    cAudioDevice* m_audioDevice;


    //-----------------------------------------------------------------------
    // PROTECTED METHODS:
    //-----------------------------------------------------------------------

protected:

    //! Renders a 2D layer within this camera's view.
    void renderLayer(cGenericObject* a_graph, int a_width, int a_height);
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
