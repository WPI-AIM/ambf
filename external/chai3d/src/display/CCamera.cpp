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
    \version   3.2.0 $Rev: 2181 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "display/CCamera.h"
using namespace std;
//------------------------------------------------------------------------------
#include "world/CWorld.h"
#include "lighting/CSpotLight.h"
#include "lighting/CDirectionalLight.h"
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
    Constructor of cCamera.

    \param  a_parentWorld  Parent world.
*/
//==============================================================================
cCamera::cCamera(cWorld* a_parentWorld)
{
    // set parent world
    m_parentWorld = a_parentWorld;
    
    // set default values for clipping planes
    setClippingPlanes(0.1, 1000.0);

    // set default field of view angle in degrees
    setFieldViewAngleDeg(45);

    // if __true__ a custom modelview matrix is used in addition to the camera
    m_useCustomModelViewMatrix = false;

    // if __true__ a custom projection matrix is used
    m_useCustomProjectionMatrix = false;

    // position and orient camera, looking down the negative x-axis
    // (the robotics convention)
    set(
          cVector3d( 0,0,0),       // Local Position of camera.
          cVector3d(-1,0,0),      // Local Look At position
          cVector3d( 0,0,1)        // Local Up Vector
        );

    // init variable related to spherical positioning
    m_zenithReference.set(0,0,1);
    m_azimuthReference.set(1,0,0);
    m_originReference.set(0,0,0);

    // by default we use a perspective camera
    m_perspectiveMode = true;

    // width of orthographic view. (not active by default)
    m_orthographicWidth = 0.0;

    // set default stereo parameters
    m_stereoMode            = C_STEREO_DISABLED;
    m_stereoFocalLength     = 2.0;
    m_stereoEyeSeparation   = 0.07;

    // disable multipass transparency rendering by default
    m_useMultipassTransparency = false;

    // reset display status
    m_markForUpdate = false;

    // create front and back layers
    m_frontLayer = new cWorld();
    m_backLayer = new cWorld();

    // create a directional light source for front layer
    m_lightFrontLayer = new cDirectionalLight(m_frontLayer);
    m_frontLayer->addChild(m_lightFrontLayer);
    m_lightFrontLayer->setDir(-1.0, -1.0, -1.0);
    m_lightFrontLayer->setEnabled(true);

    // create a directional light source for back layer
    m_lightBackLayer = new cDirectionalLight(m_backLayer);
    m_backLayer->addChild(m_lightBackLayer);
    m_lightBackLayer->setDir(-1.0, -1.0, -1.0);
    m_lightBackLayer->setEnabled(true);

    // mirroring
    m_mirrorHorizontal = false;
    m_mirrorVertical = false;
    m_mirrorStatus = false;
    m_scaleH = 1.0;
    m_scaleV = 1.0;

    // default spherical coord
    m_posRadius = 1.0;
    m_posPolarRad = 0.0;
    m_posAzimuthRad = 0.0;

    // audio device
    m_audioDevice = NULL;
}


//==============================================================================
/*!
    Destructor of cCamera. \n

    Deletes all widgets, so if you have objects that shouldn't be deleted, be
    sure to remove them from the scene graph before deleting their parents.
*/
//==============================================================================
cCamera::~cCamera()
{
    // delete front layer
    delete m_frontLayer;

    // delete back layer
    delete m_backLayer;
};


//==============================================================================
/*!
    This method sets the position and orientation of the camera. Three vectors
    are required and passed by argument: \n

    \p a_localPosition describes the position in local coordinates of the 
    camera. \n

    \p a_localLookAt describes a point at which the camera is looking at.

    \p a_localUp orients the camera around its rolling axis. This vector always
    points to the top of the camera output image. \n

    These vectors are used in the usual gluLookAt() sense.

    \param  a_localPosition  The position of the camera in local coordinates
    \param  a_localLookAt    The Point in local space at which the camera looks
    \param  a_localUp        A vector giving the rolling orientation (points toward
                             the top of the image)
*/
//==============================================================================
bool cCamera::set(const cVector3d& a_localPosition, 
                  const cVector3d& a_localLookAt,
                  const cVector3d& a_localUp)
{
    // copy new values to temp variables
    cVector3d pos = a_localPosition;
    cVector3d lookAt = a_localLookAt;
    cVector3d up = a_localUp;
    cVector3d Cy;

    // check validity of vectors
    if (pos.distancesq(lookAt) < C_SMALL) { return (false); }
    if (up.lengthsq() < C_SMALL) { return (false); }

    // compute new rotation matrix
    pos.sub(lookAt);
    pos.normalize();
    up.normalize();
    up.crossr(pos, Cy);
    if (Cy.lengthsq() < C_SMALL) { return (false); }
    Cy.normalize();
    pos.crossr(Cy,up);

    // update frame with new values
    setLocalPos(a_localPosition);
    cMatrix3d localRot;
    localRot.setCol(pos, Cy, up);
    setLocalRot(localRot);

    // return success
    return (true);
}


//==============================================================================
/*!
    This method sets the position and orientation of the camera in spherical 
    coordinates. Angles are expressed in __radians__.

    \param  a_radius      Distance from origin.
    \param  a_polarRad    Polar angle in __radians__.
    \param  a_azimuthRad  Azimuth angle in __radians__.
*/
//==============================================================================
void cCamera::setSphericalRad(const double& a_radius,
    const double& a_polarRad,
    const double& a_azimuthRad)
{
    m_posRadius = a_radius;
    m_posAzimuthRad = a_azimuthRad;
    m_posPolarRad = a_polarRad;

    cMatrix3d rot;
    rot.setCol(m_azimuthReference, cCross(m_zenithReference, m_azimuthReference), m_zenithReference);
    rot.rotateAboutLocalAxisRad(0, 0, 1, m_posAzimuthRad);
    rot.rotateAboutLocalAxisRad(0, 1, 0, m_posPolarRad - C_PI_DIV_2);

    cVector3d pos = m_originReference + rot * cVector3d(m_posRadius, 0.0, 0.0);

    setLocalPos(pos);
    setLocalRot(rot);
}


//==============================================================================
/*!
    This method sets the position and orientation of the camera in spherical 
    coordinates. Angles are expressed in __degrees__.

    \param  a_radius      Distance from origin.
    \param  a_polarDeg    Polar angle in __degrees__.
    \param  a_azimuthDeg  Azimuth angle in __degrees__.
*/
//==============================================================================
void cCamera::setSphericalDeg(const double& a_radius,
    const double& a_polarDeg,
    const double& a_azimuthDeg)
{
    setSphericalRad(a_radius, cDegToRad(a_polarDeg), cDegToRad(a_azimuthDeg));
}


//==============================================================================
/*!
    This method set the reference vectors that define the basis of the spherical
    coordinate system.

    \param  a_originReference   Origin of reference system.
    \param  a_zenithReference   Zenith direction vector.
    \param  a_azimuthReference  Azimuth direction vector.
*/
//==============================================================================
void cCamera::setSphericalReferences(const cVector3d& a_originReference, 
    const cVector3d& a_zenithReference,
    const cVector3d& a_azimuthReference)
{
    // sanity check
    if (a_zenithReference.length() < C_SMALL) { return; }
    if (a_azimuthReference.length() < C_SMALL) { return; }

    // normalize vector
    cVector3d zenithReference = cNormalize(a_zenithReference);

    // update azimuth if both vectors are not perpendicular
    cVector3d azimuthReference = cCross(cCross(zenithReference, a_azimuthReference), zenithReference);
    if (azimuthReference.length() < C_SMALL) { return; }

    m_zenithReference = zenithReference;
    m_azimuthReference = cNormalize(azimuthReference);
    m_originReference = a_originReference;

    // update camera pose
    setSphericalRad(m_posRadius, m_posPolarRad, m_posAzimuthRad);
}


//==============================================================================
/*!
    This method sets the __radius__ position of the camera
    (spherical coordinates).

    \param  a_radius  Distance from origin.
*/
//==============================================================================
void cCamera::setSphericalRadius(const double& a_radius)
{
    setSphericalRad(a_radius, m_posPolarRad, m_posAzimuthRad);
}


//==============================================================================
/*!
    This method sets the __polar__ position of the camera in __radians__ 
    (spherical coordinates).

    \param  a_polarRad  Polar angle in __radians__.
*/
//==============================================================================
void cCamera::setSphericalPolarRad(const double& a_polarRad)
{
    setSphericalRad(m_posRadius, a_polarRad, m_posAzimuthRad);
}


//==============================================================================
/*!
    This method sets the __polar__ position of the camera in __degrees__
    (spherical coordinates).

    \param  a_polarDeg  Polar angle in __degrees__.
*/
//==============================================================================
void cCamera::setSphericalPolarDeg(const double& a_polarDeg)
{
    setSphericalPolarRad(cDegToRad(a_polarDeg));
}


//==============================================================================
/*!
    This method sets the __azimuth__ position of the camera in __radians__
    (spherical coordinates).

    \param  a_azimuthRad  Azimuth angle in __radians__.
*/
//==============================================================================
void cCamera::setSphericalAzimuthRad(const double& a_azimuthRad)
{
    setSphericalRad(m_posRadius, m_posPolarRad, a_azimuthRad);
}


//==============================================================================
/*!
    This method sets the __azimuth__ position of the camera in __degrees__
    (spherical coordinates).

    \param  a_azimuthDeg  Azimuth angle in __degrees__.
*/
//==============================================================================
void cCamera::setSphericalAzimuthDeg(const double& a_azimuthDeg)
{
    setSphericalAzimuthRad(cDegToRad(a_azimuthDeg));
}


//==============================================================================
/*!
    This method sets the zenith reference direction vector of the camera
    (spherical coordinates).

    \param  a_zenithReference  Zenith reference direction.
*/
//==============================================================================
void cCamera::setSphericalZenithReference(const cVector3d& a_zenithReference)
{
    // sanity check
    if (a_zenithReference.length() < C_SMALL) { return; }

    // normalize vector
    cVector3d zenithReference = cNormalize(a_zenithReference);

    // update azimuth if both vectors are not perpendicular
    cVector3d azimuthReference = cCross(cCross(zenithReference, m_azimuthReference), zenithReference);
    if (azimuthReference.length() < C_SMALL) { return; }

    m_zenithReference = zenithReference;
    m_azimuthReference = cNormalize(azimuthReference);

    // update camera pose
    setSphericalRad(m_posRadius, m_posPolarRad, m_posAzimuthRad);
}


//==============================================================================
/*!
    This method sets the azimuth reference direction vector of the camera
    (spherical coordinates).

    \param  a_azimuthReference  Azimuth reference direction.
*/
//==============================================================================
void cCamera::setSphericalAzimuthReference(const cVector3d& a_azimuthReference)
{
    // sanity check
    if (a_azimuthReference.length() < C_SMALL) { return; }

    // normalize vector
    cVector3d azimuthReference = cNormalize(a_azimuthReference);

    // update zenith if both vectors are not perpendicular
    cVector3d zenithReference = cCross(azimuthReference, cCross(m_zenithReference, azimuthReference));
    if (zenithReference.length() < C_SMALL) { return; }

    m_zenithReference = cNormalize(zenithReference);
    m_azimuthReference = azimuthReference;

    // update camera pose
    setSphericalRad(m_posRadius, m_posPolarRad, m_posAzimuthRad);
}


//==============================================================================
/*!
    This method sets the origin target point of the camera
    (spherical coordinates).

    \param  a_originReference  Origin reference point.
*/
//==============================================================================
void cCamera::setSphericalOriginReference(const cVector3d& a_originReference)
{
    m_originReference = a_originReference;
    setSphericalRad(m_posRadius, m_posPolarRad, m_posAzimuthRad);
}


//==============================================================================
/*!
    This method sets the camera in orthographic mode.

    \param  a_viewWidth  Width of orthographic view.
*/
//==============================================================================
void cCamera::setOrthographicView(double a_viewWidth)
{
    m_orthographicWidth = cClamp0(a_viewWidth);
    m_fieldViewAngleDeg = 0.0;
    m_perspectiveMode = false;
}


//==============================================================================
/*!
    This method sets the field of view angle in __degrees__. This call sets 
    the camera in perspective mode.

    \param  a_fieldViewAngleDeg  Field of view angle in __degrees__ (0-180).
*/
//==============================================================================
void cCamera::setFieldViewAngleDeg(double a_fieldViewAngleDeg)
{
    m_fieldViewAngleDeg = cClamp(a_fieldViewAngleDeg, 0.0, 180.0);
    m_orthographicWidth = 0.0;
    m_perspectiveMode = true;
}


//==============================================================================
/*!
    This method returns the aspect ratio of output image.

    \return Aspect ratio of image. Returns 1.0 if value cannot be computed.
*/
//==============================================================================
double cCamera::getAspectRatio()
{
    double ratio = 1.0;
    if (m_lastDisplayHeight > 0)
    {
        ratio = (((double)m_lastDisplayWidth / (double)m_lastDisplayHeight));
    }
    return (ratio);
}


//==============================================================================
/*!
    This method sets the 3D stereo rendering mode. The following rendering modes are 
    supported: \n\n
      
    * C_STEREO_ACTIVE: Active stereo, requires OpenGL quad buffers. \n
    * C_STEREO_PASSIVE_LEFT_RIGHT: Passive stereo. Left and Right eye images are 
      rendered next to each other. \n
    * C_STEREO_PASSIVE_TOP_BOTTOM: Passive stereo. Left and Right eye images are 
      rendered above each other. \n
    * C_STEREO_DISABLED: Disable stereo. \n
    * C_STEREO_PASSIVE_DUAL_DISPLAY: Passive stereo. Left and Right eye images are
    rendered in different framebuffers or viewports.
      
    \param  a_stereoMode  Stereo mode.
*/
//==============================================================================
void cCamera::setStereoMode(cStereoMode a_stereoMode)
{
    if (m_perspectiveMode)
    {
        m_stereoMode = a_stereoMode;
    }
    else
    {
        m_stereoMode = C_STEREO_DISABLED;
    }
}


//==============================================================================
/*!
    This method sets the stereo focal length.

    \param  a_stereoFocalLength  Focal length.
*/
//==============================================================================
void cCamera::setStereoFocalLength(double a_stereoFocalLength)
{
    m_stereoFocalLength = a_stereoFocalLength;

    // Prevent 0 or negative focal lengths
    if (m_stereoFocalLength < C_SMALL)
    {
        m_stereoFocalLength = C_SMALL;
    }
}


//==============================================================================
/*!
    This method sets the stereo eye separation.

    \param  a_stereoEyeSeparation  Distance between the left and right eyes.
*/
//==============================================================================
void cCamera::setStereoEyeSeparation(double a_stereoEyeSeparation)
{
    m_stereoEyeSeparation = a_stereoEyeSeparation; 
}


//==============================================================================
/*!
    This method enables or disables the output image mirroring horizontally.

    \param  a_enabled  If __true_ then mirroring is enabled, __false__ otherwise.
*/
//==============================================================================
void cCamera::setMirrorHorizontal(bool a_enabled)
{
    // update state
    m_mirrorHorizontal = a_enabled;

    // update scale factor
    if (m_mirrorHorizontal)
    {
        m_scaleH = -1.0;
    }
    else
    {
        m_scaleH = 1.0;
    }

    // update mirror status
    if (((m_mirrorHorizontal) && (!m_mirrorVertical)) ||
        ((!m_mirrorHorizontal) && (m_mirrorVertical)))
    {
        m_mirrorStatus = true;
    }
    else
    {
        m_mirrorStatus = false;
    }
}


//==============================================================================
/*!
    This method enables or disables the output image mirroring vertically.

    \param  a_enabled  If __true_ then mirroring is enabled, __false__ otherwise.
*/
//==============================================================================
void cCamera::setMirrorVertical(bool a_enabled)
{
    // update state
    m_mirrorVertical = a_enabled;

    // update scale factor
    if (m_mirrorVertical)
    {
        m_scaleV = -1.0;
    }
    else
    {
        m_scaleV = 1.0;
    }

    // update mirror status
    if (((m_mirrorHorizontal) && (!m_mirrorVertical)) ||
        ((!m_mirrorHorizontal) && (m_mirrorVertical)))
    {
        m_mirrorStatus = true;
    }
    else
    {
        m_mirrorStatus = false;
    }
}



//==============================================================================
/*!
    This method sets the positions of the near and far clip planes.

    \param  a_distanceNear  Distance to near clipping plane.
    \param  a_distanceFar   Distance to far clipping plane.
*/
//==============================================================================
void cCamera::setClippingPlanes(const double a_distanceNear, const double a_distanceFar)
{
    // check values of near and far clipping planes
    if ((a_distanceNear > 0.0) &&
        (a_distanceFar > 0.0) &&
        (a_distanceFar > a_distanceNear))
    {
        m_distanceNear = a_distanceNear;
        m_distanceFar = a_distanceFar;
    }
}


//==============================================================================
/*!
    This method checks for collision detection between an x-y position 
    (typically a mouse click) and an object in the scene.

    \param  a_windowPosX         X coordinate position of mouse click.
    \param  a_windowPosY         Y coordinate position of mouse click.
    \param  a_windowWidth        Width of window display (pixels)
    \param  a_windowHeight       Height of window display (pixels)
    \param  a_collisionRecorder  Recorder used to store all collisions between mouse and objects
    \param  a_collisionSettings  Settings related to collision detection

    \return __true__ if an object has been hit, __false__ otherwise.
*/
//==============================================================================
bool cCamera::selectWorld(const int a_windowPosX,
                          const int a_windowPosY,
                          const int a_windowWidth,
                          const int a_windowHeight,
                          cCollisionRecorder& a_collisionRecorder,
                          cCollisionSettings& a_collisionSettings)
{
    // sanity check
    if ((a_windowWidth <= 0) || (a_windowHeight <= 0) || (m_parentWorld == NULL)) return (false);

    // store values
    int windowPosX = a_windowPosX;
    int windowPosY = a_windowPosY;
    int windowWidth = a_windowWidth;
    int windowHeight = a_windowHeight;
    double scaleFactorX = 1.0;
    double scaleFactorY = 1.0;

    // adjust values when passive stereo is used
    if (m_stereoMode == C_STEREO_PASSIVE_LEFT_RIGHT)
    {
        double center = 0.5 * windowWidth;
        if (windowPosX > center)
        {
            windowPosX = windowPosX - (int)center;
        }
        windowWidth = (int)center;
        scaleFactorX = 2.0;
        scaleFactorY = 1.0;
    }
    else if (m_stereoMode == C_STEREO_PASSIVE_TOP_BOTTOM)
    {
        double center = 0.5 * windowHeight;
        if (windowPosY > center)
        {
            windowPosY = windowPosY - (int)center;
        }
        windowHeight = (int)center;
        scaleFactorX = 1.0;
        scaleFactorY = 2.0;
    }

    // adjust values when image is mirrored horizontally
    if (m_mirrorHorizontal)
    {
        windowPosX = windowWidth - windowPosX;
    }

    // adjust values when image is mirrored vertically
    if (m_mirrorVertical)
    {
        windowPosY = windowHeight - windowPosY;
    }

    // clear collision recorder
    a_collisionRecorder.clear();

    // update my m_globalPos and m_globalRot variables
    m_parentWorld->computeGlobalPositions(false);

    // init variable to store result
    bool result = false;
    if (m_perspectiveMode)
    {
        // make sure we have a legitimate field of view
        if (fabs(m_fieldViewAngleDeg) < 0.001f) { return (false); }

        // compute the ray that leaves the eye point at the appropriate angle
        //
        // m_fieldViewAngleDeg / 2.0 would correspond to the _top_ of the window
        double distCam = (scaleFactorY * windowHeight / 2.0f) / cTanDeg(m_fieldViewAngleDeg / 2.0f);

        cVector3d selectRay;
        selectRay.set(-distCam,
                      scaleFactorX * (windowPosX - (windowWidth / 2.0f)),
                      scaleFactorY * (windowPosY - (windowHeight / 2.0f)));
        selectRay.normalize();

        selectRay = cMul(m_globalRot, selectRay);

        // create a point that's way out along that ray
        cVector3d selectPoint = cAdd(m_globalPos, cMul(m_distanceFar, selectRay));

        // search for intersection between the ray and objects in the world
        result = m_parentWorld->computeCollisionDetection(
                                    m_globalPos,
                                    selectPoint,
                                    a_collisionRecorder,
                                    a_collisionSettings);
    }
    else
    {
        double hw = (double)(windowWidth) * 0.5;
        double hh = (double)(windowHeight)* 0.5;
        double aspect = a_windowWidth / a_windowHeight;
        
        double offsetX = ((windowPosX - hw) / hw) * 0.5 * m_orthographicWidth;
        double offsetY =-((windowPosY - hh) / hh) * 0.5 * (m_orthographicWidth / aspect);

        cVector3d pos = cAdd(m_globalPos, 
                             cMul(offsetX, m_globalRot.getCol1()), 
                             cMul(offsetY, m_globalRot.getCol2()));

        // create a point that's way out along that ray
        cVector3d selectPoint = cAdd(pos, cMul(100000,  cNegate(m_globalRot.getCol0())));

        result = m_parentWorld->computeCollisionDetection(pos,
                                                          selectPoint,
                                                          a_collisionRecorder,
                                                          a_collisionSettings);
    }

    // return result
    return (result);
}


//==============================================================================
/*!
    This method checks for collision detection between an x-y position 
    (typically a mouse click) and a widget on the front layer. The (0,0) 
    coordinate is located at the bottom left pixel on the screen.

    \param  a_windowPosX         X coordinate position of mouse click.
    \param  a_windowPosY         Y coordinate position of mouse click.
    \param  a_windowWidth        Width of window display (pixels)
    \param  a_windowHeight       Height of window display (pixels)
    \param  a_collisionRecorder  Recorder used to store all collisions between mouse and objects
    \param  a_collisionSettings  Settings related to collision detection

    \return __true__ if an object has been hit, otherwise __false__.
*/
//==============================================================================
bool cCamera::selectFrontLayer(const int a_windowPosX, const int a_windowPosY,
    const int a_windowWidth, const int a_windowHeight,
    cCollisionRecorder& a_collisionRecorder,
    cCollisionSettings& a_collisionSettings)
{
    return (selectLayers(a_windowPosX,
        a_windowPosY,
        a_windowWidth,
        a_windowHeight,
        a_collisionRecorder,
        a_collisionSettings,
        true,
        false));
}


//==============================================================================
/*!
    This method checks for collision detection between an x-y position 
    (typically a mouse click) and a widget on the back layer. 
    The (0,0) coordinate is located at the bottom left pixel on the screen.

    \param  a_windowPosX         X coordinate position of mouse click.
    \param  a_windowPosY         Y coordinate position of mouse click.
    \param  a_windowWidth        Width of window display (pixels)
    \param  a_windowHeight       Height of window display (pixels)
    \param  a_collisionRecorder  Recorder used to store all collisions between mouse and objects
    \param  a_collisionSettings  Settings related to collision detection

    \return __true__ if an object has been hit, otherwise __false__.
*/
//==============================================================================
bool cCamera::selectBackLayer(const int a_windowPosX, const int a_windowPosY,
    const int a_windowWidth, const int a_windowHeight,
    cCollisionRecorder& a_collisionRecorder,
    cCollisionSettings& a_collisionSettings)
{
    return (selectLayers(a_windowPosX,
                 a_windowPosY,
                 a_windowWidth,
                 a_windowHeight,
                 a_collisionRecorder,
                 a_collisionSettings,
                 false,
                 true));
}


//==============================================================================
/*!
    This method checks for collision detection between an x-y position 
    (typically a mouse click) and a widget on the front layer. 
    The (0,0) coordinate is located at the bottom left pixel on the screen.

    \param  a_windowPosX         X coordinate position of mouse click.
    \param  a_windowPosY         Y coordinate position of mouse click.
    \param  a_windowWidth        Width of window display (pixels)
    \param  a_windowHeight       Height of window display (pixels)
    \param  a_collisionRecorder  Recorder used to store all collisions between mouse and objects
    \param  a_collisionSettings  Settings related to collision detection
    \param  a_checkFrontLayer    If __true__, select front layer.
    \param  a_checkBackLayer     If __true__, select back layer.

    \return __true__ if an object has been hit, otherwise __false__.
*/
//==============================================================================
bool cCamera::selectLayers(const int a_windowPosX, const int a_windowPosY,
    const int a_windowWidth, const int a_windowHeight,
    cCollisionRecorder& a_collisionRecorder,
    cCollisionSettings& a_collisionSettings,
    bool a_checkFrontLayer,
    bool a_checkBackLayer)
{
    // sanity check
    if ((a_windowWidth <= 0) || (a_windowHeight <= 0)) return (false);

    // store values
    int windowPosX = a_windowPosX;
    int windowPosY = a_windowPosY;
    int windowWidth = a_windowWidth;
    int windowHeight = a_windowHeight;

    double scaleFactorX = 1.0;
    double scaleFactorY = 1.0;

    // adjust values when passive stereo is used
    if (m_stereoMode == C_STEREO_PASSIVE_LEFT_RIGHT)
    {
        double center = 0.5 * windowWidth;
        if (windowPosX > center)
        {
            windowPosX = windowPosX - (int)center;
        }
        windowWidth = (int)center;
        scaleFactorX = 2.0;
        scaleFactorY = 1.0;
    }
    else if (m_stereoMode == C_STEREO_PASSIVE_TOP_BOTTOM)
    {
        double center = 0.5 * windowHeight;
        if (windowPosY > center)
        {
            windowPosY = windowPosY - (int)center;
        }
        windowHeight = (int)center;
        scaleFactorX = 1.0;
        scaleFactorY = 2.0;
    }

    // adjust values when image is mirrored horizontally
    if (m_mirrorHorizontal)
    {
        windowPosX = windowWidth - windowPosX;
    }

    // adjust values when image is mirrored vertically
    if (m_mirrorVertical)
    {
        windowPosY = windowHeight - windowPosY;
    }

    // clear collision recorder
    a_collisionRecorder.clear();

    // update my m_globalPos and m_globalRot variables
    m_frontLayer->computeGlobalPositions(false);
    m_backLayer->computeGlobalPositions(false);

    // init variable to store result
    bool resultFront = false;
    bool resultBack = false;

    // create ray
    double posX = scaleFactorX * windowPosX;
    double posY = scaleFactorY * windowPosY;

    cVector3d pos0(posX, posY, 10000);
    cVector3d pos1(posX, posY,-10000);

    // compute collision detection front layer
    if (a_checkFrontLayer)
    {
        resultFront = m_frontLayer->computeCollisionDetection(pos0,
            pos1,
            a_collisionRecorder,
            a_collisionSettings);
    }

    // compute collision detection back layer
    if (a_checkBackLayer)
    {
        resultBack = m_backLayer->computeCollisionDetection(pos0,
            pos1,
            a_collisionRecorder,
            a_collisionSettings);
    }

    // return result
    bool result = resultBack || resultFront;

    return (result);
}


//==============================================================================
/*!
    This method renders the scene viewed by the camera.

    \param  a_windowWidth     Width of viewport.
    \param  a_windowHeight    Height of viewport.
    \param  a_eyeMode         When using stereo mode C_STEREO_PASSIVE_DUAL_DISPLAY, 
                              specifies which eye view to render.
    \param  a_defaultBuffer   If __true__ then the scene is rendered in the default
                              OpenGL buffer. If __false_ then the scene is rendered 
                              in a framebuffer (cFrameBuffer) that will have been 
                              previously setup.
*/
//==============================================================================
void cCamera::renderView(const int a_windowWidth, 
                         const int a_windowHeight,
                         const cEyeMode a_eyeMode,
                         const bool a_defaultBuffer)
{
#ifdef C_USE_OPENGL

    //-----------------------------------------------------------------------
    // (0) INITIALIZATION
    //-----------------------------------------------------------------------

    // enable multi-sampling if available
    glEnable(GL_MULTISAMPLE);


    //-----------------------------------------------------------------------
    // (1) SHADOW CASTING
    //-----------------------------------------------------------------------

    bool useShadowCasting = false;

    // check if shadow casting is supported and if shadow maps are available
    if (m_parentWorld != NULL)
    {
        if (m_parentWorld->getUseShadowCastring() && !m_parentWorld->m_shadowMaps.empty())
        {
            useShadowCasting = true;
        }
    }

    //-----------------------------------------------------------------------
    // (2) INITIALIZE CURRENT VIEWPORT
    //-----------------------------------------------------------------------

    // check window size
    if (a_windowHeight == 0) { return; }

    // store most recent size of display
    m_lastDisplayWidth = a_windowWidth;
    m_lastDisplayHeight = a_windowHeight;

    // compute aspect ratio
    double glAspect = ((double)a_windowWidth / (double)a_windowHeight);

    // compute global pose
    computeGlobalPositionsFromRoot(true);

    // set background color
    if (m_parentWorld != NULL)
    {
        glClearColor(m_parentWorld->getBackgroundColor().getR(),
            m_parentWorld->getBackgroundColor().getG(),
            m_parentWorld->getBackgroundColor().getB(),
            1.0);
    }
    else
    {
        glClearColor(0.0, 0.0, 0.0, 1.0);
    }


    //-----------------------------------------------------------------------
    // (3) VERIFY IF STEREO DISPLAY IS ENABLED
    //-----------------------------------------------------------------------
    GLboolean stereo = false;
    unsigned int numStereoPass = 1;
    cEyeMode eyeMode = C_STEREO_LEFT_EYE;

    if (m_stereoMode != C_STEREO_DISABLED)
    {
        /////////////////////////////////////////////////////////////////////
        // ACTIVE STEREO
        /////////////////////////////////////////////////////////////////////
        if (m_stereoMode == C_STEREO_ACTIVE)
        {
            // verify if stereo is available by the graphics hardware and camera is of perspective model
            glGetBooleanv(GL_STEREO, &stereo); 

            if (stereo)
            {
                // stereo is available - we shall perform 2 rendering passes for LEFT and RIGHT eye.
                numStereoPass = 2;
            }
        }

        /////////////////////////////////////////////////////////////////////
        // PASSIVE STEREO (LEFT/RIGHT)
        /////////////////////////////////////////////////////////////////////
        else if (m_stereoMode == C_STEREO_PASSIVE_LEFT_RIGHT)
        {
            stereo = true;
            numStereoPass = 2;
        }

        /////////////////////////////////////////////////////////////////////
        // PASSIVE STEREO (TOP/BOTTOM)
        /////////////////////////////////////////////////////////////////////
        else if (m_stereoMode == C_STEREO_PASSIVE_TOP_BOTTOM)
        {
            stereo = true;
            numStereoPass = 2;
        }

        /////////////////////////////////////////////////////////////////////
        // PASSIVE STEREO (DUAL DISPLAY)
        /////////////////////////////////////////////////////////////////////
        else if (m_stereoMode == C_STEREO_PASSIVE_DUAL_DISPLAY)
        {
            stereo = true;
            numStereoPass = 1;
        }
    }


    //-----------------------------------------------------------------------
    // (4) RENDER THE ENTIRE SCENE
    //-----------------------------------------------------------------------
    for (unsigned int i=0; i<numStereoPass; i++)
    {
        //-------------------------------------------------------------------
        // (4.1) SELECTING THE DISPLAY BUFFER (MONO / STEREO)
        //-------------------------------------------------------------------
        if ((stereo) && (m_stereoMode != C_STEREO_PASSIVE_DUAL_DISPLAY))
        {
            /////////////////////////////////////////////////////////////////
            // LEFT EYE
            /////////////////////////////////////////////////////////////////
            if (i == 0)
            {
                switch (m_stereoMode)
                {
                    case C_STEREO_ACTIVE:
                        glViewport(0, 0, a_windowWidth, a_windowHeight);
                        glDrawBuffer(GL_BACK_LEFT);
                        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
                        break;

                    case C_STEREO_PASSIVE_LEFT_RIGHT:
                        glViewport(0, 0, a_windowWidth/2, a_windowHeight);
                        if (a_defaultBuffer)
                            glDrawBuffer(GL_BACK);
                        else
                            glDrawBuffer(GL_COLOR_ATTACHMENT0);
                        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
                        break;

                    case C_STEREO_PASSIVE_TOP_BOTTOM:
                        glViewport(0, a_windowHeight/2, a_windowWidth, a_windowHeight/2);
                        if (a_defaultBuffer)
                            glDrawBuffer(GL_BACK);
                        else
                            glDrawBuffer(GL_COLOR_ATTACHMENT0);
                        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
                        break;
                    
                    default: break;
                }

                // select left eye for rendering
                eyeMode = C_STEREO_LEFT_EYE;
            }

            /////////////////////////////////////////////////////////////////
            // RIGHT EYE
            /////////////////////////////////////////////////////////////////
            else
            {
                switch (m_stereoMode)
                {
                    case C_STEREO_ACTIVE:
                        glDrawBuffer(GL_BACK_RIGHT);
                        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
                        break;

                    case C_STEREO_PASSIVE_LEFT_RIGHT:
                        glViewport(a_windowWidth/2, 0, a_windowWidth/2, a_windowHeight);
                        break;

                    case C_STEREO_PASSIVE_TOP_BOTTOM:
                        glViewport(0, 0, a_windowWidth, a_windowHeight/2);
                        break;
                    
                    default: break;
                }

                // select right eye for rendering
                eyeMode = C_STEREO_RIGHT_EYE;
            }
        }
        else
        {
            glViewport(0, 0, a_windowWidth, a_windowHeight); 

            if (a_defaultBuffer)
                glDrawBuffer(GL_BACK);
            else
                glDrawBuffer(GL_COLOR_ATTACHMENT0);

            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            // select eye for rendering
            eyeMode = a_eyeMode;
        }

        //-------------------------------------------------------------------
        // (4.2) SETUP GENERAL RENDERING SETTINGS
        //-------------------------------------------------------------------

        glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
        glShadeModel(GL_SMOOTH);

        //-------------------------------------------------------------------
        // (4.3)  RENDER BACK PLANE
        //-------------------------------------------------------------------

        // render the 2D backlayer
        // it will set up its own projection matrix
        if (m_backLayer->getNumChildren())
        {
            renderLayer(m_backLayer,
                        a_windowWidth,
                        a_windowHeight);
        }

        // clear depth buffer
        glClear(GL_DEPTH_BUFFER_BIT);

        //-------------------------------------------------------------------
        // (4.4a) SETUP CAMERA  (MONO RENDERING)
        //-------------------------------------------------------------------
        if (!stereo)
        {
            // init projection matrix
            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();

            // adjust display for mirroring
            glScalef((GLfloat)m_scaleH, (GLfloat)m_scaleV, (GLfloat)1.0);
            if (m_mirrorStatus)
            {
                glFrontFace(GL_CW);
            }
            else
            {
                glFrontFace(GL_CCW);
            }

            // create projection matrix depending of camera mode
            if (m_useCustomProjectionMatrix)
            {
                glLoadMatrixd(m_projectionMatrix.getData());
            }
            else
            {
                if (m_perspectiveMode)
                {
                    // setup perspective camera
                    gluPerspective(
                            m_fieldViewAngleDeg, // Field of View angle.
                            glAspect,            // Aspect ratio of viewing volume.
                            m_distanceNear,      // Distance to Near clipping plane.
                            m_distanceFar);      // Distance to Far clipping plane.
                }
                else
                {
                    // setup orthographic camera
                    double left     = -m_orthographicWidth / 2.0;
                    double right    = -left;
                    double bottom   = left / glAspect;
                    double top      = -bottom;

                    glOrtho(left,                // Left vertical clipping plane.
                            right,               // Right vertical clipping plane.
                            bottom,              // Bottom vertical clipping plane.
                            top,                 // Top vertical clipping plane.
                            m_distanceNear,      // Distance to Near clipping plane.
                            m_distanceFar        // Distance to Far clipping plane.
                        );
                }
            }

            // setup camera position
            glMatrixMode(GL_MODELVIEW);
            if (m_useCustomModelViewMatrix)
            {
                glLoadMatrixd(m_modelViewMatrix.getData());
            }
            else
            {
                glLoadIdentity();
            }

            // compute camera location
            cVector3d lookAt = m_globalRot.getCol0();
            cVector3d lookAtPos;
            m_globalPos.subr(lookAt, lookAtPos);
            cVector3d up = m_globalRot.getCol2();

            // setup modelview matrix
            gluLookAt(m_globalPos(0), m_globalPos(1), m_globalPos(2),
                      lookAtPos(0), lookAtPos(1), lookAtPos(2),
                      up(0), up(1), up(2));

        }


        //-------------------------------------------------------------------
        // (4.4b) SETUP CAMERA  (STEREO RENDERING)
        //-------------------------------------------------------------------
        else
        {
            // init projection matrix
            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();

            // adjust display for mirroring
            glScalef((GLfloat)m_scaleH, (GLfloat)m_scaleV, (GLfloat)1.0);
            if (m_mirrorStatus)
            {
                glFrontFace(GL_CW);
            }
            else
            {
                glFrontFace(GL_CCW);
            }

            if (m_perspectiveMode)
            {
                double radians = 0.5 * cDegToRad(m_fieldViewAngleDeg);
                double wd2 = m_distanceNear * tan(radians);
                double ndfl = m_distanceNear / m_stereoFocalLength;

                // compute the look, up, and cross vectors
                cVector3d lookv = m_globalRot.getCol0();
                lookv.mul(-1.0);

                cVector3d upv = m_globalRot.getCol2();
                cVector3d offsetv = cCross(lookv,upv);

                offsetv.mul(m_stereoEyeSeparation / 2.0);

                // decide whether to offset left or right
                double stereo_multiplier;
                if (eyeMode == C_STEREO_LEFT_EYE)
                {
                    // left eye
                    stereo_multiplier = -1.0;
                    offsetv.mul(-1.0);
                }
                else
                {
                    // right eye
                    stereo_multiplier = 1.0;
                }

                double left   = -1.0 * glAspect * wd2 + stereo_multiplier * 0.5 * m_stereoEyeSeparation * ndfl;
                double right  =        glAspect * wd2 + stereo_multiplier * 0.5 * m_stereoEyeSeparation * ndfl;
                double top    =        wd2;
                double bottom = -1.0 * wd2;

                glFrustum(left, right, bottom, top, m_distanceNear, m_distanceFar);

                // initialize modelview matrix
                glMatrixMode(GL_MODELVIEW);
                glLoadIdentity();

                // compute the offset we should apply to the current camera position
                cVector3d pos = cAdd(m_globalPos, offsetv);

                // compute the shifted camera position
                cVector3d lookAtPos;
                pos.addr(lookv, lookAtPos);

                // setup modelview matrix
                gluLookAt(pos(0), pos(1), pos(2),
                          lookAtPos(0), lookAtPos(1), lookAtPos(2),
                          upv(0), upv(1), upv(2));
            }
            else
            {
                // setup orthographic camera
                double left     = -m_orthographicWidth / 2.0;
                double right    = -left;
                double bottom   = left / glAspect;
                double top      = -bottom;

                glOrtho(left,            // Left vertical clipping plane.
                    right,               // Right vertical clipping plane.
                    bottom,              // Bottom vertical clipping plane.
                    top,                 // Top vertical clipping plane.
                    m_distanceNear,      // Distance to Near clipping plane.
                    m_distanceFar        // Distance to Far clipping plane.
                    );

                // setup camera position
                glMatrixMode(GL_MODELVIEW);
                glLoadIdentity();

                // compute camera location
                cVector3d lookAt = m_globalRot.getCol0();
                cVector3d lookAtPos;
                m_globalPos.subr(lookAt, lookAtPos);
                cVector3d up = m_globalRot.getCol2();

                // setup modelview matrix
                gluLookAt(m_globalPos(0), m_globalPos(1), m_globalPos(2),
                          lookAtPos(0), lookAtPos(1), lookAtPos(2),
                          up(0), up(1), up(2));
            }
        }

        // Backup the view and projection matrix for future reference
        glGetDoublev(GL_PROJECTION_MATRIX,m_projectionMatrix.getData());
        glGetDoublev(GL_MODELVIEW_MATRIX, m_modelViewMatrix.getData());


        //-------------------------------------------------------------------
        // (4.5) RENDER THE 3D WORLD
        //-------------------------------------------------------------------

        // Set up reasonable default OpenGL state
        glEnable(GL_LIGHTING);
        glDisable(GL_BLEND);
        glDepthMask(GL_TRUE);
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LEQUAL);

        // rendering options
        cRenderOptions options;

        if (m_parentWorld != NULL)
        {
            // optionally perform multiple rendering passes for transparency
            if (m_useMultipassTransparency) 
            {
                ////////////////////////////////////////////////////////////////////
                // MULTI PASS - USING SHADOW CASTING
                ////////////////////////////////////////////////////////////////////
                if (useShadowCasting)
                {
                    //--------------------------------------------------------------
                    // OPAQUE OBJECTS
                    //--------------------------------------------------------------

                    // setup rendering options
                    options.m_camera                                = this;
                    options.m_single_pass_only                      = false;
                    options.m_render_opaque_objects_only            = true;
                    options.m_render_transparent_front_faces_only   = false;
                    options.m_render_transparent_back_faces_only    = false;
                    options.m_enable_lighting                       = true;
                    options.m_render_materials                      = true;
                    options.m_render_textures                       = true;
                    options.m_creating_shadow_map                   = false;
                    options.m_rendering_shadow                      = true;
                    options.m_shadow_light_level                    = 1.0 - m_parentWorld->getShadowIntensity();
                    options.m_storeObjectPositions                  = false;
                    options.m_markForUpdate                         = m_markForUpdate;

                    // render 1st pass (opaque objects - shadowed regions)
                    m_parentWorld->renderSceneGraph(options);

                    // setup rendering options
                    options.m_rendering_shadow                      = false;
                    options.m_shadow_light_level                    = 1.0;

                    // render 2nd pass (opaque objects - non shadowed regions)
                    list<cShadowMap*>::iterator lst;
                    for(lst = m_parentWorld->m_shadowMaps.begin(); lst !=m_parentWorld->m_shadowMaps.end(); ++lst)
                    {
                        cShadowMap *shadowMap = *lst;
                        shadowMap->render(options);

                        if (m_parentWorld != NULL)
                        {
                            m_parentWorld->renderSceneGraph(options);
                        }

                        // restore states
                        glActiveTexture(shadowMap->m_depthBuffer->getTextureUnit());
                        glDisable(GL_TEXTURE_2D);
                        glDisable(GL_TEXTURE_GEN_S);
                        glDisable(GL_TEXTURE_GEN_T);
                        glDisable(GL_TEXTURE_GEN_R);
                        glDisable(GL_TEXTURE_GEN_Q);
                        glDisable(GL_ALPHA_TEST);
                    }

                    //--------------------------------------------------------------
                    // TRANSPARENT OBJECTS
                    //--------------------------------------------------------------

                    // setup rendering options
                    options.m_render_opaque_objects_only            = false;
                    options.m_render_transparent_back_faces_only    = true;
                    options.m_render_transparent_front_faces_only   = false;
                    options.m_rendering_shadow                      = false;

                    // render 3rd pass (transparent objects - back faces only)
                    m_parentWorld->renderSceneGraph(options);

                    // modify rendering options for third pass
                    options.m_render_opaque_objects_only            = false;
                    options.m_render_transparent_back_faces_only    = false;
                    options.m_render_transparent_front_faces_only   = true;
                    options.m_rendering_shadow                      = true;
                    options.m_shadow_light_level                    = 1.0 - m_parentWorld->getShadowIntensity();

                    // render 4th pass (transparent objects - front faces only - shadowed areas)
                    m_parentWorld->renderSceneGraph(options);
                
                    for(lst = m_parentWorld->m_shadowMaps.begin(); lst != m_parentWorld->m_shadowMaps.end(); ++lst)
                    {
                        cShadowMap *shadowMap = *lst;
                        shadowMap->render(options);

                        if (m_parentWorld != NULL)
                        {
                            m_parentWorld->renderSceneGraph(options);
                        }

                        // restore states
                        glActiveTexture(shadowMap->m_depthBuffer->getTextureUnit());
                        glDisable(GL_TEXTURE_2D);
                        glDisable(GL_TEXTURE_GEN_S);
                        glDisable(GL_TEXTURE_GEN_T);
                        glDisable(GL_TEXTURE_GEN_R);
                        glDisable(GL_TEXTURE_GEN_Q);
                        glDisable(GL_ALPHA_TEST);
                    }

                    // modify rendering options for 5th pass
                    options.m_rendering_shadow                      = false;
                    options.m_shadow_light_level                    = 1.0;

                    // render 5th pass (transparent objects - front faces only - lighted regions)
                    for(lst = m_parentWorld->m_shadowMaps.begin(); lst !=m_parentWorld->m_shadowMaps.end(); ++lst)
                    {
                        cShadowMap *shadowMap = *lst;
                        shadowMap->render(options);

                        if (m_parentWorld != NULL)
                        {
                            m_parentWorld->renderSceneGraph(options);
                        }

                        // restore states
                        glActiveTexture(shadowMap->m_depthBuffer->getTextureUnit());
                        glDisable(GL_TEXTURE_2D);
                        glDisable(GL_TEXTURE_GEN_S);
                        glDisable(GL_TEXTURE_GEN_T);
                        glDisable(GL_TEXTURE_GEN_R);
                        glDisable(GL_TEXTURE_GEN_Q);
                        glDisable(GL_ALPHA_TEST);
                    }
                }


                ////////////////////////////////////////////////////////////////////
                // MULTI PASS - WITHOUT SHADOWS
                ////////////////////////////////////////////////////////////////////
                else
                {
                    // setup rendering options for first pass
                    options.m_camera                                = this;
                    options.m_single_pass_only                      = false;
                    options.m_render_opaque_objects_only            = true;
                    options.m_render_transparent_front_faces_only   = false;
                    options.m_render_transparent_back_faces_only    = false;
                    options.m_enable_lighting                       = true;
                    options.m_render_materials                      = true;
                    options.m_render_textures                       = true;
                    options.m_creating_shadow_map                   = false;
                    options.m_rendering_shadow                      = false;
                    options.m_shadow_light_level                    = 1.0;
                    options.m_storeObjectPositions                  = true;
                    options.m_markForUpdate                         = m_markForUpdate;

                    // render 1st pass (opaque objects - all faces)
                    if (m_parentWorld != NULL)
                    {
                        m_parentWorld->renderSceneGraph(options);
                    }

                    // modify rendering options
                    options.m_render_opaque_objects_only            = false;
                    options.m_render_transparent_back_faces_only    = true;
                    options.m_storeObjectPositions                  = false;

                    // render 2nd pass (transparent objects - back faces only)
                    if (m_parentWorld != NULL)
                    {
                        m_parentWorld->renderSceneGraph(options);
                    }

                    // modify rendering options
                    options.m_render_transparent_back_faces_only    = false;
                    options.m_render_transparent_front_faces_only   = true;

                    // render 3rd pass (transparent objects - front faces only)
                    if (m_parentWorld != NULL)
                    {
                        m_parentWorld->renderSceneGraph(options);
                    }
                }
            }
            else
            {
                ////////////////////////////////////////////////////////////////////
                // SINGLE PASS - USING SHADOW CASTING 
                ////////////////////////////////////////////////////////////////////
                if (useShadowCasting)
                {
                    // setup rendering options for single pass
                    options.m_camera                                = this;
                    options.m_single_pass_only                      = true;
                    options.m_render_opaque_objects_only            = true;
                    options.m_render_transparent_front_faces_only   = false;
                    options.m_render_transparent_back_faces_only    = false;
                    options.m_enable_lighting                       = true;
                    options.m_render_materials                      = true;
                    options.m_render_textures                       = true;
                    options.m_creating_shadow_map                   = false;
                    options.m_rendering_shadow                      = true;
                    options.m_shadow_light_level                    = 1.0 - m_parentWorld->getShadowIntensity();
                    options.m_storeObjectPositions                  = false;
                    options.m_markForUpdate                         = m_markForUpdate;

                    // render 1st pass (opaque objects - all faces - shadowed regions)
                    m_parentWorld->renderSceneGraph(options);

                    // setup rendering options
                    options.m_rendering_shadow                      = false;
                    options.m_shadow_light_level                    = 1.0;

                    // render 2nd pass (opaque objects - all faces - lighted regions)
                    list<cShadowMap*>::iterator lst;
                    for(lst = m_parentWorld->m_shadowMaps.begin(); lst !=m_parentWorld->m_shadowMaps.end(); ++lst)
                    {
                        cShadowMap *shadowMap = *lst;
                        shadowMap->render(options);

                        if (m_parentWorld != NULL)
                        {
                            m_parentWorld->renderSceneGraph(options);
                        }

                        // restore states
                        glActiveTexture(shadowMap->m_depthBuffer->getTextureUnit());
                        glDisable(GL_TEXTURE_2D);
                        glDisable(GL_TEXTURE_GEN_S);
                        glDisable(GL_TEXTURE_GEN_T);
                        glDisable(GL_TEXTURE_GEN_R);
                        glDisable(GL_TEXTURE_GEN_Q);
                        glDisable(GL_ALPHA_TEST);
                    }

                    // setup rendering options
                    options.m_render_opaque_objects_only            = false;
                    options.m_render_transparent_front_faces_only   = true;
                    options.m_render_transparent_back_faces_only    = true;

                    // render 3rd pass (transparent objects - all faces)
                    if (m_parentWorld != NULL)
                    {
                        m_parentWorld->renderSceneGraph(options);
                    }
                }


                ////////////////////////////////////////////////////////////////////
                // SINGLE PASS - WITHOUT SHADOWS
                ////////////////////////////////////////////////////////////////////
                else
                {
                    // setup rendering options for single pass
                    options.m_camera                                = this;
                    options.m_single_pass_only                      = true;
                    options.m_render_opaque_objects_only            = false;
                    options.m_render_transparent_front_faces_only   = false;
                    options.m_render_transparent_back_faces_only    = false;
                    options.m_enable_lighting                       = true;
                    options.m_render_materials                      = true;
                    options.m_render_textures                       = true;
                    options.m_creating_shadow_map                   = false;
                    options.m_rendering_shadow                      = false;
                    options.m_shadow_light_level                    = 1.0;
                    options.m_storeObjectPositions                  = true;
                    options.m_markForUpdate                         = m_markForUpdate;

                    // render single pass (all objects)
                    if (m_parentWorld != NULL)
                    {
                        m_parentWorld->renderSceneGraph(options); 
                    }
                }
            }
        }

        //-------------------------------------------------------------------
        // (4.6) RENDER FRONT PLANE
        //-------------------------------------------------------------------

        // clear depth buffer
        if (a_defaultBuffer){
            glClear(GL_DEPTH_BUFFER_BIT);
            // clear depth buffer
        }

        // render the 'front' 2d object layer; it will set up its own
        // projection matrix
        if (m_frontLayer->getNumChildren() > 0)
        {
            renderLayer(m_frontLayer,
                        a_windowWidth,
                        a_windowHeight);
        }

        // if requested, display reset has now been completed
        m_markForUpdate = false;
    }

    //-----------------------------------------------------------------------
    // (5) AUDIO DEVICE
    //-----------------------------------------------------------------------

    // update position of optionally attached audio device
    if (m_audioDevice != NULL)
    {
        m_audioDevice->setListenerPos(m_globalPos);
        m_audioDevice->setListenerRot(m_globalRot);
    }

#endif
}


//==============================================================================
/*!
    This method copies the OpenGL image buffer to a cImage class structure.

    \param  a_image  Destination image.
*/
//==============================================================================
void cCamera::copyImageBuffer(cImagePtr a_image)
{
#ifdef C_USE_OPENGL

    // check image structure
    if (a_image == nullptr) { return; }

    // check size
    if ((m_lastDisplayWidth  != a_image->getWidth()) ||
        (m_lastDisplayHeight != a_image->getHeight()))
    {
        a_image->allocate(m_lastDisplayWidth, m_lastDisplayHeight, GL_RGBA);
    }

    // settings
    glPixelStorei(GL_PACK_ALIGNMENT, 1);

    // copy pixel data if required
    glReadPixels(
        0,
        0,
        m_lastDisplayWidth,
        m_lastDisplayHeight,
        a_image->getFormat(),
        GL_UNSIGNED_BYTE,
        a_image->getData()
    );

#endif
}


//==============================================================================
/*!
    This method enables or disables multipass transparency. When this option is
    enabled (it's disabled by default), each time the camera is
    asked to render the scene, it will perform three rendering
    passes: a pass for non-transparent items, a pass for the back faces
    of transparent items, and a pass for the front faces of transparent
    items. \n

    Objects being rendered are told which pass is current via the 
    parameter supplied to the render() function.

    \param  a_enabled  If __true__, multipass is enabled, __false__ otherwise.
*/
//==============================================================================
void cCamera::setUseMultipassTransparency(bool a_enabled) 
{
    m_useMultipassTransparency = a_enabled;
}


//==============================================================================
/*!
    This method automatically adjusts the front and back clipping planes to
    optimize usage of the z-buffer.
*/
//==============================================================================
void cCamera::adjustClippingPlanes()
{
    // check if world is valid
    cWorld* world = getParentWorld();
    if (world == NULL) { return; }

    // compute size of the world
    world->computeBoundaryBox(true);

    // compute a distance slightly larger the world size
    cVector3d max = world->getBoundaryMax();
    cVector3d min = world->getBoundaryMin();
    double distance = 2.0 * cDistance(min, max);

    // update clipping plane
    if (distance > 0.0)
    {
        setClippingPlanes(distance / 1000.0, distance);
    }
}


//==============================================================================
/*!
    This method renders a 2D scene within the viewport.

    \param  a_graph           The root of the 2d scenegraph to be rendered.
    \param  a_width           The size of the rendering window
    \param  a_height          The size of the rendering window
*/
//==============================================================================
void cCamera::renderLayer(cGenericObject* a_graph,
                          int a_width,
                          int a_height)
{
#ifdef C_USE_OPENGL

    // set up an orthographic projection matrix
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();

    // adjust display for mirroring
    glScalef((GLfloat)m_scaleH, (GLfloat)m_scaleV, (GLfloat)1.0);
    if (m_mirrorStatus)
    {
        glFrontFace(GL_CW);
    }
    else
    {
        glFrontFace(GL_CCW);
    }

    // set orthographic rendering mode. The z-buffer front and back clipping planes
    // can be set to any desired default values.
    glOrtho(0, a_width, 0, a_height, -10000, 10000);

    // reset modeling view matrix.
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    // enable blending
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // setup default light.
    glEnable(GL_LIGHTING);

    // setup rendering options for single pass
    cRenderOptions options;
    options.m_camera                                = this;
    options.m_single_pass_only                      = true;
    options.m_render_opaque_objects_only            = false;
    options.m_render_transparent_front_faces_only   = false;
    options.m_render_transparent_back_faces_only    = false;
    options.m_enable_lighting                       = true;
    options.m_render_materials                      = true;
    options.m_render_textures                       = true;
    options.m_creating_shadow_map                   = false;
    options.m_rendering_shadow                      = false;
    options.m_shadow_light_level                    = 1.0;
    options.m_storeObjectPositions                  = true;
    options.m_markForUpdate                         = false;

    // render light source
    glColorMaterial(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE);

    // render widget scene graph
    a_graph->renderSceneGraph(options);

    // put OpenGL back into a useful state
    glEnable(GL_LIGHTING);
    glDisable(GL_BLEND);

    // restore modelview and projection matrices
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();

    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();

#endif
}


//==============================================================================
/*!
    This method updates all display lists and textures to the GPU.
*/
//==============================================================================
void cCamera::updateGPU()
{
    m_markForUpdate = true;
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
