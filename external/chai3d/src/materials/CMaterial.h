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
#ifndef CMaterialH
#define CMaterialH
//------------------------------------------------------------------------------
#include "audio/CAudioBuffer.h"
#include "graphics/CColor.h"
#include "graphics/CRenderOptions.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file    CMaterial.h
    
    \brief
    Implements material properties.
*/
//==============================================================================

//------------------------------------------------------------------------------
struct cMaterial;
typedef std::shared_ptr<cMaterial> cMaterialPtr;
//------------------------------------------------------------------------------

//==============================================================================
/*!
      \class      cMaterial
      \ingroup    materials

      \brief
      This class models material properties.

      \details
      This class provides a description for handling OpenGL graphic material 
      properties. These include: ambient color, diffuse color, specular color, 
      emissive color, and shininess. \n\n

      Haptic properties are also defined in this class. Properties include 
      stiffness, dynamic friction, and static friction, viscosity, vibration
      and magnetic effects. Force rendering algorithms will lookup the
      material properties of an object to compute the desired force rendering
      effect.\n\n

      cMaterial also allows the programmer to backup the current materials colors
      by calling method backup(). These colors can later be retrieved by calling
      the method restore(). \n\n

      This capability can be very useful when implementing a selection procedure where
      objects change color when being being selected by a computer mouse for instance.
      A different color can be applied to the selected object and then restored
      to the original color once the selection has been completed.
*/
//==============================================================================
struct cMaterial
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------
 
public:

    //! Constructor of cMaterial.
    cMaterial();

    //! Destructor of cMaterial.
    virtual ~cMaterial() {};

    //! Shared cMaterial allocator.
    static cMaterialPtr create() { return (std::make_shared<cMaterial>()); }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - COPY:
    //--------------------------------------------------------------------------

public:

    //! This method creates a copy itself.
    cMaterialPtr copy();

    //! This method set a value value to all modification flags.
    void setModificationFlags(const bool a_value);

    //! This method copies all modified members (the ones which are flagged) to another material object.
    void copyTo(cMaterialPtr a_material);


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - GRAPHIC PROPERTIES:
    //--------------------------------------------------------------------------

public:

    //! This method sets the shininess (the exponent used for specular lighting).
    void setShininess(const GLuint a_shininess);

    //! This method returns the shininess level.
    GLuint getShininess() const { return (m_shininess); }

    //! This method sets the transparency level (alpha value) to all color components.
    void setTransparencyLevel(const float a_levelTransparency);

    //! This method returns __true__ if the material includes partial or full transparency color components.
    inline bool isTransparent() const
    {
        return (m_ambient[4] < 1.0 ||
                m_diffuse[4] < 1.0 ||
                m_specular[4] < 1.0 ||
                m_emission[4]);
    }

    //! This method creates a backup of its color properties. Any previous backup is lost.
    inline void backupColors()
    {
        m_ambient.copyTo(m_ambientBackup);
        m_diffuse.copyTo(m_diffuseBackup);
        m_specular.copyTo(m_specularBackup);
        m_emission.copyTo(m_emissionBackup);
    }

    //! This method restores the color properties from values stored in the backup members.
    inline void restoreColors()
    {
        m_ambientBackup.copyTo(m_ambient);
        m_diffuseBackup.copyTo(m_diffuse);
        m_specularBackup.copyTo(m_specular);
        m_emissionBackup.copyTo(m_emission);
    }

    //! This method render the material using OpenGL.
    virtual void render(cRenderOptions& a_options);


    //--------------------------------------------------------------------------
    // PUBLIC MEMBERS - GRAPHIC PROPERTIES:
    //--------------------------------------------------------------------------

public:

    //! Ambient color.
    cColorf m_ambient;

    //! Diffuse color.
    cColorf m_diffuse;

    //! Specular color.
    cColorf m_specular;

    //! Emission color.
    cColorf m_emission;

    //! Ambient color. (Backup)
    cColorf m_ambientBackup;

    //! Diffuse color. (Backup)
    cColorf m_diffuseBackup;

    //! Specular color. (Backup)
    cColorf m_specularBackup;

    //! Emission color. (Backup)
    cColorf m_emissionBackup;


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - HAPTIC EFFECTS PROPERTIES:
    //--------------------------------------------------------------------------

public:

    ////////////////////////////////////////////////////////////////////////////
    // SURFACE STIFFNESS
    ////////////////////////////////////////////////////////////////////////////

    //! This method set the stiffness level [N/m].
    void setStiffness(const double a_stiffness);

    //! This method returns the stiffness level [N/m].
    inline double getStiffness() const { return (m_stiffness); }


    ////////////////////////////////////////////////////////////////////////////
    // VISCOSITY
    ////////////////////////////////////////////////////////////////////////////

    //! This method sets the level of viscosity.
    void setViscosity(const double a_viscosity);

    //! This method returns the level of viscosity.
    inline double getViscosity() const { return (m_viscosity); }


    ////////////////////////////////////////////////////////////////////////////
    // VIBRATION
    ////////////////////////////////////////////////////////////////////////////

    //! This method set the vibration frequency [Hz].
    void setVibrationFrequency(const double a_vibrationFrequency);

    //! This method returns the vibration frequency [Hz].
    inline double getVibrationFrequency() const {return (m_vibrationFrequency); }

    //! This method sets the vibration force amplitude [N].
    void setVibrationAmplitude(const double a_vibrationAmplitude);

    //! This method returns the vibration force amplitude [N].
    inline double getVibrationAmplitude() const { return (m_vibrationAmplitude); }


    ////////////////////////////////////////////////////////////////////////////
    // MAGNET
    ////////////////////////////////////////////////////////////////////////////

    //! This method sets the maximum force applied by the magnet effect [N]
    void setMagnetMaxForce(const double a_magnetMaxForce);

    //! This method returns the maximum force applied by the magnet effect[N]
    inline double getMagnetMaxForce() const { return (m_magnetMaxForce); }

    //! This method sets the maximum distance threshold from which magnetic forces are perceived [m].
    void setMagnetMaxDistance(const double a_magnetMaxDistance);

    //! This method returns the maximum distance threshold from which magnetic forces are perceived [m].
    inline double getMagnetMaxDistance() const { return (m_magnetMaxDistance); }


    ////////////////////////////////////////////////////////////////////////////
    // STICK AND SLIP
    ////////////////////////////////////////////////////////////////////////////

    //! This method sets the maximum force threshold of a stick and slip model [N].
    void setStickSlipForceMax(const double a_stickSlipForceMax);

    //! This method returns the maximum force threshold of a stick and slip model [N].
    inline double getStickSlipForceMax() const { return (m_stickSlipForceMax); }

    //! This method sets the stiffness of the stick and slip model [N/m]
    void setStickSlipStiffness(double const a_stickSlipStiffness);

    //! This method sets the stiffness of the stick and slip model [N/m]
    inline double getStickSlipStiffness() const { return (m_stickSlipStiffness); }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - HAPTIC PROPERTIES (MESH OBJECTS ONLY)
    //--------------------------------------------------------------------------

public:

    ////////////////////////////////////////////////////////////////////////////
    // DAMPING
    ////////////////////////////////////////////////////////////////////////////

    //! This method sets the damping coefficient.
    void setDamping(const double a_dampingCoefficient);

    //! This method returns the damping coefficient.
    inline double getDamping() const { return (m_damping); }


    ////////////////////////////////////////////////////////////////////////////
    // FRICTION
    ////////////////////////////////////////////////////////////////////////////

    //! This method enables or disables haptic friction. (Applies to polygonal objects only).
    void setUseHapticFriction(const bool a_useHapticFriction);

    //! This method returns the status of haptic friction. (Applies to polygonal objects only).
    inline bool getUseHapticFriction() const { return (m_useHapticFriction); }

    //! This method sets the static friction level [N].
    void setStaticFriction(const double a_friction);

    //! This method returns the static friction level [N].
    inline double getStaticFriction() const { return (m_staticFriction); }

    //! This method sets the dynamic friction level.
    void setDynamicFriction(const double a_friction);

    //! This method returns the dynamic friction level.
    inline double getDynamicFriction() const { return (m_dynamicFriction); }


    ////////////////////////////////////////////////////////////////////////////
    // TEXTURE
    ////////////////////////////////////////////////////////////////////////////

    //! This method enables or disables haptic texture rendering. (Applies to polygonal objects only).
    void setUseHapticTexture(const bool a_useHapticTexture);

    //! This method returns the status of haptic texture rendering. (Applies to polygonal objects only).
    inline bool getUseHapticTexture() const { return (m_useHapticTexture); }

    //! This method sets the haptic texture level.
    void setTextureLevel(const double a_textureLevel);

    //! This method returns the haptic  texture level.
    inline double getTextureLevel() const { return (m_textureLevel); }


    ////////////////////////////////////////////////////////////////////////////
    // HAPTIC MODES
    ////////////////////////////////////////////////////////////////////////////

    //! This method enables or disables haptic shading. (Applies to polygonal objects only).
    void setUseHapticShading(const bool a_useHapticShading);

    //! This method retruns the status of haptic shading. (Applies to polygonal objects only).
    inline bool getUseHapticShading() const { return (m_useHapticShading); }

    //! This method enables or disables haptic rendering on the __front__ side of mesh triangles.
    void setHapticTriangleFrontSide(const bool a_enabled);

    //! This method returns the status about __front__ side triangle haptic rendering. (Applies to polygonal objects only).
    inline bool getHapticTriangleFrontSide() const { return (m_hapticFrontSideOfTriangles); }

    //! This method enables or disables haptic rendering of __back__ side of mesh triangles.
    void setHapticTriangleBackSide(const bool a_enabled);

    //! This method returns the status about __back__ side triangle haptic rendering. (Applies to polygonal objects only).
    inline bool getHapticTriangleBackSide() const { return (m_hapticBackSideOfTriangles); }

    //! This method enables or disables haptic rendering of __front__ and __back__ sides of mesh triangles.
    void setHapticTriangleSides(const bool a_enableFrontSide, const bool a_enableBackSide);


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - AUDIO PROPERTIES
    //--------------------------------------------------------------------------

public:

    ////////////////////////////////////////////////////////////////////////////
    // AUDIO IMPACT
    ////////////////////////////////////////////////////////////////////////////

    //! This method sets an audio buffer associated to impacts between a tool and an object.
    void setAudioImpactBuffer(cAudioBuffer* a_audioImpactBuffer);

    //! This method returns a pointer to the audio buffer associated with impacts.
    inline cAudioBuffer* getAudioImpactBuffer() { return (m_audioImpactBuffer); }

    //! This method sets the audio gain for sounds associated with impacts.
    void setAudioImpactGain(const double a_audioImpactGain);

    //! This method returns the gain value for sounds associated with impacts.
    inline double getAudioImpactGain() const { return (m_audioImpactGain); }


    ////////////////////////////////////////////////////////////////////////////
    // AUDIO FRICTION
    ////////////////////////////////////////////////////////////////////////////

    //! This method sets an audio buffer associated with friction.
    void setAudioFrictionBuffer(cAudioBuffer* a_audioFrictionBuffer);

    //! This method returns a pointer to the audio buffer associated with friction.
    inline cAudioBuffer* getAudioFrictionBuffer() { return (m_audioFrictionBuffer); }

    //! This method sets the audio gain for sounds associated with friction.
    void setAudioFrictionGain(const double a_audioFrictionGain);

    //! This method returns the audio gain for sounds associated with friction.
    inline double getAudioFrictionGain() const { return (m_audioFrictionGain); }

    //! This method sets the audio pitch gain for sounds associated with friction.
    void setAudioFrictionPitchGain(const double a_audioFrictionPitchGain);

    //! This method returns the audio pitch gain for sounds associated with friction.
    inline double getAudioFrictionPitchGain() const { return (m_audioFrictionPitchGain); }

    //! This method sets the audio pitch offset for sounds associated with friction.
    void setAudioFrictionPitchOffset(const double a_audioFrictionPitchOffset);

    //! This method returns the audio pitch offset for sounds associated with friction.
    inline double getAudioFrictionPitchOffset() const { return (m_audioFrictionPitchOffset); }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - COLOR PROPERTIES:
    //--------------------------------------------------------------------------

public:

    //! This method defines a color property for this material.
    void setColor(cColorf& a_color);

    //! This method defines a color property for this material.
    void setColor(cColorb& a_color);

    //! This method defines a color property for this material.
    void setColorf(const GLfloat a_red,
        const GLfloat a_green,
        const GLfloat a_blue,
        const GLfloat a_alpha = 1.0f);


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - RED COLORS
    //--------------------------------------------------------------------------

public:

    //! This method sets the color to Red Indian.
    inline void setRedIndian()              { m_diffuse.setb(0xCD, 0x5C, 0x5C); updateColors();} 

    //! This method sets the color to Light Coral Red. 
    inline void setRedLightCoral()          { m_diffuse.setb(0xF0, 0x80, 0x80); updateColors();}

    //! This method sets the color to Red Salmon.
    inline void setRedSalmon()              { m_diffuse.setb(0xFA, 0x80, 0x72); updateColors();}

    //! This method sets the color to Dark Red Salmon.
    inline void setRedDarkSalmon()          { m_diffuse.setb(0xE9, 0x96, 0x7A); updateColors();}

    //! This method sets the color to Light Red Salmon.
    inline void setRedLightSalmon()         { m_diffuse.setb(0xFF, 0xA0, 0x7A); updateColors();}

    //! This method sets the color to Red Crimson.
    inline void setRedCrimson()             { m_diffuse.setb(0xDC, 0x14, 0x3C); updateColors();}

    //! This method sets the color to Red.
    inline void setRed()                    { m_diffuse.setb(0xFF, 0x00, 0x00); updateColors();}

    //! This method sets the color to Red Fire Brick.
    inline void setRedFireBrick()           { m_diffuse.setb(0xB2, 0x22, 0x22); updateColors();}

    //! This method sets the color to Dark Red.
    inline void setRedDark()                { m_diffuse.setb(0x8B, 0x00, 0x00); updateColors();}

    
    //--------------------------------------------------------------------------
    // PUBLIC METHODS - PINK COLORS
    //--------------------------------------------------------------------------

    //! This method sets the color to Pink.
    inline void setPink()                   { m_diffuse.setb(0xFF, 0xC0, 0xCB); updateColors();}

    //! This method sets the color to Light Pink.
    inline void setPinkLight()              { m_diffuse.setb(0xFF, 0xB6, 0xC); updateColors();}

    //! This method sets the color to Hot Pink.
    inline void setPinkHot()                { m_diffuse.setb(0xFF, 0x69, 0xB4); updateColors();}

    //! This method sets the color to Deep Pink.
    inline void setPinkDeep()               { m_diffuse.setb(0xFF, 0x14, 0x93); updateColors();}

    //! This method sets the color to Medium Violet Red.
    inline void setPinkMediumVioletRed()    { m_diffuse.setb(0xC7, 0x15, 0x85); updateColors();}

    //! This method sets the color to Pale Violet Red.
    inline void setPinkPaleVioletRed()      { m_diffuse.setb(0xDB, 0x70, 0x93); updateColors();}


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - ORANGE COLORS
    //--------------------------------------------------------------------------

    //! This method sets the color to Orange Light Salmon.
    inline void setOrangeLightSalmon()      { m_diffuse.setb(0xFF, 0xA0, 0x7A); updateColors();}

    //! This method sets the color to Orange Coral.
    inline void setOrangeCoral()            { m_diffuse.setb(0xFF, 0x7F, 0x50); updateColors();}

    //! This method sets the color to Orange Tomato.
    inline void setOrangeTomato()           { m_diffuse.setb(0xFF, 0x63, 0x47); updateColors();}

    //! This method sets the color to Orange Red.
    inline void setOrangeRed()              { m_diffuse.setb(0xFF, 0x45, 0x00); updateColors();}

    //! This method sets the color to Dark Orange.
    inline void setOrangeDark()             { m_diffuse.setb(0xFF, 0x8C, 0x00); updateColors();}

    //! This method sets the color to Orange.
    inline void setOrange()                 { m_diffuse.setb(0xFF, 0xA5, 0x00); updateColors();}


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - YELLOW COLORS
    //--------------------------------------------------------------------------

    //! This method sets the color to Gold.
    inline void setYellowGold()             { m_diffuse.setb(0xFF, 0xD7, 0x00); updateColors();}

    //! This method sets the color to Yellow.
    inline void setYellow()                 { m_diffuse.setb(0xFF, 0xFF, 0x00); updateColors();}

    //! This method sets the color to Light Yellow.
    inline void setYellowLight()            { m_diffuse.setb(0xFF, 0xFF, 0xE0); updateColors();}

    //! This method sets the color to Lemon Chiffon.
    inline void setYellowLemonChiffon()     { m_diffuse.setb(0xFF, 0xFA, 0xCD); updateColors();}

    //! This method sets the color to Light Goldenrod.
    inline void setYellowLightGoldenrod()   { m_diffuse.setb(0xFA, 0xFA, 0xD); updateColors();}

    //! This method sets the color to Papaya Whip.
    inline void setYellowPapayaWhip()       { m_diffuse.setb(0xFF, 0xEF, 0xD5); updateColors();}

    //! This method sets the color to Moccasin.
    inline void setYellowMoccasin()         { m_diffuse.setb(0xFF, 0xE4, 0xB5); updateColors();}

    //! This method sets the color to Peach Puff.
    inline void setYellowPeachPuff()        { m_diffuse.setb(0xFF, 0xDA, 0xB9); updateColors();}

    //! This method sets the color to Pale Goldenrod.
    inline void setYellowPaleGoldenrod()    { m_diffuse.setb(0xEE, 0xE8, 0xAA); updateColors();}

    //! This method sets the color to Khaki.
    inline void setYellowKhaki()            { m_diffuse.setb(0xF0, 0xE6, 0x8C); updateColors();}

    //! This method sets the color to Dark Khaki.
    inline void setYellowDarkKhaki()        { m_diffuse.setb(0xBD, 0xB7, 0x6B); updateColors();}


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - PURPLE COLORS
    //--------------------------------------------------------------------------

    //! This method sets the color to Lavendar.
    inline void setPurpleLavender()         { m_diffuse.setb(0xE6, 0xE6, 0xFA); updateColors();}

    //! This method sets the color to Thistle.
    inline void setPurpleThistle()          { m_diffuse.setb(0xD8, 0xBF, 0xD8); updateColors();}

    //! This method sets the color to Plum.
    inline void setPurplePlum()             { m_diffuse.setb(0xDD, 0xA0, 0xDD); updateColors();}

    //! This method sets the color to Violet.
    inline void setPurpleViolet()           { m_diffuse.setb(0xEE, 0x82, 0xEE); updateColors();}

    //! This method sets the color to Orchid.
    inline void setPurpleOrchid()           { m_diffuse.setb(0xDA, 0x70, 0xD6); updateColors();}

    //! This method sets the color to Fuchsia.
    inline void setPurpleFuchsia()          { m_diffuse.setb(0xFF, 0x00, 0xFF); updateColors();}

    //! This method sets the color to Magenta.
    inline void setPurpleMagenta()          { m_diffuse.setb(0xFF, 0x00, 0xFF); updateColors();}

    //! This method sets the color to Medium Orchid.
    inline void setPurpleMediumOrchid()     { m_diffuse.setb(0xBA, 0x55, 0xD3); updateColors();}

    //! This method sets the color to Medium Purple.
    inline void setPurpleMedium()           { m_diffuse.setb(0x93, 0x70, 0xDB); updateColors();}

    //! This method sets the color to Amethyst.
    inline void setPurpleAmethyst()         { m_diffuse.setb(0x99, 0x66, 0xCC); updateColors();}

    //! This method sets the color to Blue Violet.
    inline void setPurpleBlueViolet()       { m_diffuse.setb(0x8A, 0x2B, 0xE2); updateColors();}

    //! This method sets the color to Dark Violet.
    inline void setPurpleDarkViolet()       { m_diffuse.setb(0x94, 0x00, 0xD3); updateColors();}

    //! This method sets the color to Dark Orchid.
    inline void setPurpleDarkOrchid()       { m_diffuse.setb(0x99, 0x32, 0xCC); updateColors();}

    //! This method sets the color to Dark Magenta.
    inline void setPurpleDarkMagenta()      { m_diffuse.setb(0x8B, 0x00, 0x8B); updateColors();}

    //! This method sets the color to Purple.
    inline void setPurple()                 { m_diffuse.setb(0x80, 0x00, 0x80); updateColors();}

    //! This method sets the color to Indigo.
    inline void setPurpleIndigo()           { m_diffuse.setb(0x4B, 0x00, 0x82); updateColors();}

    //! This method sets the color to Slate Blue.
    inline void setPurpleSlateBlue()        { m_diffuse.setb(0x6A, 0x5A, 0xCD); updateColors();}

    //! This method sets the color to Dark Slate Blue.
    inline void setPurpleDarkSlateBlue()    { m_diffuse.setb(0x48, 0x3D, 0x8B); updateColors();}

    //! This method sets the color to Medium Slate Blue.
    inline void setPurpleMediumSlateBlue()  { m_diffuse.setb(0x7B, 0x68, 0xEE); updateColors();}


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - GREEN COLORS
    //--------------------------------------------------------------------------

    //! This method sets the color to Green Yellow.
    inline void setGreenYellow()            { m_diffuse.setb(0xAD, 0xFF, 0x2F); updateColors();}

    //! This method sets the color to Chartreuse.
    inline void setGreenChartreuse()        { m_diffuse.setb(0x7F, 0xFF, 0x00); updateColors();}

    //! This method sets the color to Lawn Green.
    inline void setGreenLawn()              { m_diffuse.setb(0x7C, 0xFC, 0x00); updateColors();}

    //! This method sets the color to Lime.
    inline void setGreenLime()              { m_diffuse.setb(0x00, 0xFF, 0x00); updateColors();}

    //! This method sets the color to Lime Green.
    inline void setGreenLimeGreen()         { m_diffuse.setb(0x32, 0xCD, 0x32); updateColors();}

    //! This method sets the color to Pale Green.
    inline void setGreenPale()              { m_diffuse.setb(0x98, 0xFB, 0x98); updateColors();}

    //! This method sets the color to Light Green.
    inline void setGreenLight()             { m_diffuse.setb(0x90, 0xEE, 0x90); updateColors();}

    //! This method sets the color to Medium Spring Green.
    inline void setGreenMediumSpring()      { m_diffuse.setb(0x00, 0xFA, 0x9A); updateColors();}

    //! This method sets the color to Spring Green.
    inline void setGreenSpring()            { m_diffuse.setb(0x00, 0xFF, 0x7F); updateColors();}

    //! This method sets the color to Medium Sea Green.
    inline void setGreenMediumSea()         { m_diffuse.setb(0x3C, 0xB3, 0x71); updateColors();}

    //! This method sets the color to Sea Green.
    inline void setGreenSea()               { m_diffuse.setb(0x2E, 0x8B, 0x57); updateColors();}

    //! This method sets the color to Forest Green.
    inline void setGreenForest()            { m_diffuse.setb(0x22, 0x8B, 0x22); updateColors();}

    //! This method sets the color to Green.
    inline void setGreen()                  { m_diffuse.setb(0x00, 0x80, 0x00); updateColors();}

    //! This method sets the color to Dark Green.
    inline void setGreenDark()              { m_diffuse.setb(0x00, 0x64, 0x00); updateColors();}

    //! This method sets the color to Yellow Green.
    inline void setGreenYellowGreen()       { m_diffuse.setb(0x9A, 0xCD, 0x32); updateColors();}

    //! This method sets the color to Olive Drab.
    inline void setGreenOliveDrab()         { m_diffuse.setb(0x6B, 0x8E, 0x23); updateColors();}

    //! This method sets the color to Olive.
    inline void setGreenOlive()             { m_diffuse.setb(0x80, 0x80, 0x00); updateColors();}

    //! This method sets the color to Dark Olive Green.
    inline void setGreenDarkOlive()         { m_diffuse.setb(0x55, 0x6B, 0x2F); updateColors();}

    //! This method sets the color to Medium Aquamarine.
    inline void setGreenMediumAquamarine()  { m_diffuse.setb(0x66, 0xCD, 0xAA); updateColors();}

    //! This method sets the color to Dark Sea Green.
    inline void setGreenDarkSea()           { m_diffuse.setb(0x8F, 0xBC, 0x8F); updateColors();}

    //! This method sets the color to Light Sea Green.
    inline void setGreenLightSea()          { m_diffuse.setb(0x20, 0xB2, 0xAA); updateColors();}

    //! This method sets the color to Dark Cyan.
    inline void setGreenDarkCyan()          { m_diffuse.setb(0x00, 0x8B, 0x8B); updateColors();}

    //! This method sets the color to Teal.
    inline void setGreenTeal()              { m_diffuse.setb(0x00, 0x80, 0x80); updateColors();}


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - BLUE COLORS
    //--------------------------------------------------------------------------

    //! This method sets the color to Aqua.
    inline void setBlueAqua()               { m_diffuse.setb(0x00, 0xFF, 0xFF); updateColors();}

    //! This method sets the color to Cyan.
    inline void setBlueCyan()               { m_diffuse.setb(0x00, 0xFF, 0xFF); updateColors();}

    //! This method sets the color to Light Cyan.
    inline void setBlueLightCyan()          { m_diffuse.setb(0xE0, 0xFF, 0xFF); updateColors();}

    //! This method sets the color to Pale Turquoise.
    inline void setBluePaleTurquoise()      { m_diffuse.setb(0xAF, 0xEE, 0xEE); updateColors();}

    //! This method sets the color to Aquamarine.
    inline void setBlueAquamarine()         { m_diffuse.setb(0x7F, 0xFF, 0xD4); updateColors();}

    //! This method sets the color to Turquoise.
    inline void setBlueTurquoise()          { m_diffuse.setb(0x40, 0xE0, 0xD0); updateColors();}

    //! This method sets the color to Medium Turquoise.
    inline void setBlueMediumTurquoise()    { m_diffuse.setb(0x48, 0xD1, 0xCC); updateColors();}

    //! This method sets the color to Dark Turquoise.
    inline void setBlueDarkTurquoise()      { m_diffuse.setb(0x00, 0xCE, 0xD1); updateColors();}

    //! This method sets the color to Cadet Blue.
    inline void setBlueCadet()              { m_diffuse.setb(0x5F, 0x9E, 0xA0); updateColors();}

    //! This method sets the color to Steel Blue.
    inline void setBlueSteel()              { m_diffuse.setb(0x46, 0x82, 0xB4); updateColors();}

    //! This method sets the color to Light Steel Blue.
    inline void setBlueLightSteel()         { m_diffuse.setb(0xB0, 0xC4, 0xDE); updateColors();}

    //! This method sets the color to Powder Blue.
    inline void setBluePowder()             { m_diffuse.setb(0xB0, 0xE0, 0xE6); updateColors();}

    //! This method sets the color to Light Blue.
    inline void setBlueLight()              { m_diffuse.setb(0xAD, 0xD8, 0xE6); updateColors();}

    //! This method sets the color to Sky Blue.
    inline void setBlueSky()                { m_diffuse.setb(0x87, 0xCE, 0xEB); updateColors();}

    //! This method sets the color to Light Sky Blue.
    inline void setBlueLightSky()           { m_diffuse.setb(0x87, 0xCE, 0xFA); updateColors();}

    //! This method sets the color to Deep Sky Blue.
    inline void setBlueDeepSky()            { m_diffuse.setb(0x00, 0xBF, 0xFF); updateColors();}

    //! This method sets the color to Doger Blue.
    inline void setBlueDodger()             { m_diffuse.setb(0x1E, 0x90, 0xFF); updateColors();}

    //! This method sets the color to Cornflower Blue.
    inline void setBlueCornflower()         { m_diffuse.setb(0x64, 0x95, 0xED); updateColors();}

    //! This method sets the color to Medium Slate Blue.
    inline void setBlueMediumSlate()        { m_diffuse.setb(0x7B, 0x68, 0xEE); updateColors();}

    //! This method sets the color to Royal Blue.
    inline void setBlueRoyal()              { m_diffuse.setb(0x41, 0x69, 0xE1); updateColors();}

    //! This method sets the color to Blue.
    inline void setBlue()                   { m_diffuse.setb(0x00, 0x00, 0xFF); updateColors();}

    //! This method sets the color to Medium Blue.
    inline void setBlueMedium()             { m_diffuse.setb(0x00, 0x00, 0xCD); updateColors();}

    //! This method sets the color to Dark Blue.
    inline void setBlueDark()               { m_diffuse.setb(0x00, 0x00, 0x8B); updateColors();}

    //! This method sets the color to Navy.
    inline void setBlueNavy()               { m_diffuse.setb(0x00, 0x00, 0x80); updateColors();}

    //! This method sets the color to Midnight Blue.
    inline void setBlueMidnight()           { m_diffuse.setb(0x19, 0x19, 0x70); updateColors();}


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - BROWN COLORS
    //--------------------------------------------------------------------------

    //! This method sets the color to Cornsilk.
    inline void setBrownCornsilk()          { m_diffuse.setb(0xFF, 0xF8, 0xDC); updateColors();}

    //! This method sets the color to Blanched Almond.
    inline void setBrownBlanchedAlmond()    { m_diffuse.setb(0xFF, 0xEB, 0xCD); updateColors();}

    //! This method sets the color to Bisque.
    inline void setBrownBisque()            { m_diffuse.setb(0xFF, 0xE4, 0xC4); updateColors();}

    //! This method sets the color to Navajo White.
    inline void setBrownNavajoWhite()       { m_diffuse.setb(0xFF, 0xDE, 0xAD); updateColors();}

    //! This method sets the color to Wheat.
    inline void setBrownWheat()             { m_diffuse.setb(0xF5, 0xDE, 0xB3); updateColors();}

    //! This method sets the color to Burly Wood.
    inline void setBrownBurlyWood()         { m_diffuse.setb(0xDE, 0xB8, 0x87); updateColors();}

    //! This method sets the color to Tan.
    inline void setBrownTan()               { m_diffuse.setb(0xD2, 0xB4, 0x8C); updateColors();}

    //! This method sets the color to Rosy Brown.
    inline void setBrownRosy()              { m_diffuse.setb(0xBC, 0x8F, 0x8F); updateColors();}

    //! This method sets the color to Sandy Brown.
    inline void setBrownSandy()             { m_diffuse.setb(0xF4, 0xA4, 0x60); updateColors();}

    //! This method sets the color to Brown Goldenrod.
    inline void setBrownGoldenrod()         { m_diffuse.setb(0xDA, 0xA5, 0x20); updateColors();}

    //! This method sets the color to Dark Brown Goldenrod.
    inline void setBrownDarkGoldenrod()     { m_diffuse.setb(0xB8, 0x86, 0x0B); updateColors();}

    //! This method sets the color to Peru.
    inline void setBrownPeru()              { m_diffuse.setb(0xCD, 0x85, 0x3F); updateColors();}

    //! This method sets the color to Chocolate.
    inline void setBrownChocolate()         { m_diffuse.setb(0xD2, 0x69, 0x1E); updateColors();}

    //! This method sets the color to Saddle Brown.
    inline void setBrownSaddle()            { m_diffuse.setb(0x8B, 0x45, 0x13); updateColors();}

    //! This method sets the color to Sienna.
    inline void setBrownSienna()            { m_diffuse.setb(0xA0, 0x52, 0x2D); updateColors();}

    //! This method sets the color to Brown.
    inline void setBrown()                  { m_diffuse.setb(0xA5, 0x2A, 0x2A); updateColors();}

    //! This method sets the color to Maroon.
    inline void setBrownMaroon()            { m_diffuse.setb(0x80, 0x00, 0x00); updateColors();}


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - WHITE COLORS
    //--------------------------------------------------------------------------

    //! This method sets the color to White.
    inline void setWhite()                  { m_diffuse.setb(0xFF, 0xFF, 0xFF); updateColors();}

    //! This method sets the color to White Snow.
    inline void setWhiteSnow()              { m_diffuse.setb(0xFF, 0xFA, 0xFA); updateColors();}

    //! This method sets the color to Honeydew.
    inline void setWhiteHoneydew()          { m_diffuse.setb(0xF0, 0xFF, 0xF0); updateColors();}

    //! This method sets the color to Mint Cream.
    inline void setWhiteMintCream()         { m_diffuse.setb(0xF5, 0xFF, 0xFA); updateColors();}

    //! This method sets the color to Azure.
    inline void setWhiteAzure()             { m_diffuse.setb(0xF0, 0xFF, 0xFF); updateColors();}

    //! This method sets the color to Alice Blue.
    inline void setWhiteAliceBlue()         { m_diffuse.setb(0xF0, 0xF8, 0xFF); updateColors();}

    //! This method sets the color to Ghost White.
    inline void setWhiteGhost()             { m_diffuse.setb(0xF8, 0xF8, 0xFF); updateColors();}

    //! This method sets the color to White Smoke.
    inline void setWhiteSmoke()             { m_diffuse.setb(0xF5, 0xF5, 0xF5); updateColors();}

    //! This method sets the color to Seashell.
    inline void setWhiteSeashell()          { m_diffuse.setb(0xFF, 0xF5, 0xEE); updateColors();}

    //! This method sets the color to Beige.
    inline void setWhiteBeige()             { m_diffuse.setb(0xF5, 0xF5, 0xDC); updateColors();}

    //! This method sets the color to Old Lace.
    inline void setWhiteOldLace()           { m_diffuse.setb(0xFD, 0xF5, 0xE6); updateColors();}

    //! This method sets the color to Floral White.
    inline void setWhiteFloral()            { m_diffuse.setb(0xFF, 0xFA, 0xF0); updateColors();}

    //! This method sets the color to Ivory.
    inline void setWhiteIvory()             { m_diffuse.setb(0xFF, 0xFF, 0xF0); updateColors();}

    //! This method sets the color to Antique White.
    inline void setWhiteAntique()           { m_diffuse.setb(0xFA, 0xEB, 0xD7); updateColors();}

    //! This method sets the color to Linen.
    inline void setWhiteLinen()             { m_diffuse.setb(0xFA, 0xF0, 0xE6); updateColors();}

    //! This method sets the color to Lavender Blush.
    inline void setWhiteLavenderBlush()     { m_diffuse.setb(0xFF, 0xF0, 0xF5); updateColors();}

    //! This method sets the color to Misty Rose.
    inline void setWhiteMistyRose()         { m_diffuse.setb(0xFF, 0xE4, 0xE1); updateColors();}


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - GRAY COLORS
    //--------------------------------------------------------------------------

    //! This method sets the color to Gainsboro.
    inline void setGrayGainsboro()          { m_diffuse.setb(0xDC, 0xDC, 0xDC); updateColors();}

    //! This method sets the color to Light Gray.
    inline void setGrayLight() 	            { m_diffuse.setb(0xD3, 0xD3, 0xD3); updateColors();}

    //! This method sets the color to Silver.
    inline void setGraySilver()             { m_diffuse.setb(0xC0, 0xC0, 0xC0); updateColors();}

    //! This method sets the color to Dark Gray.
    inline void setGrayDark()               { m_diffuse.setb(0xA9, 0xA9, 0xA9); updateColors();}

    //! This method sets the color to Gray.
    inline void setGray()                   { m_diffuse.setb(0x80, 0x80, 0x80); updateColors();}

    //! This method sets the color to Dim Gray.
    inline void setGrayDim()                { m_diffuse.setb(0x69, 0x69, 0x69); updateColors();}

    //! This method sets the color to Light Slate Gray.
    inline void setGrayLightSlate()         { m_diffuse.setb(0x77, 0x88, 0x99); updateColors();}

    //! This method sets the color to Slate Gray.
    inline void setGraySlate()              { m_diffuse.setb(0x70, 0x80, 0x90); updateColors();}

    //! This method sets the color to Dark Slate Gray.
    inline void setGrayDarkSlate()          { m_diffuse.setb(0x2F, 0x4F, 0x4F); updateColors();}

    //! This method sets the color to Black.
    inline void setBlack()                  { m_diffuse.setb(0x00, 0x00, 0x00); updateColors();}


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - CUSTOM GRAY COLOR
    //--------------------------------------------------------------------------

public:

    // This method sets a custom gray level.
    inline void setGrayLevel(const GLfloat a_level) { m_diffuse.set(a_level, a_level, a_level); updateColors(); }


    //--------------------------------------------------------------------------
    // PROTECTED METHODS - GRAPHICS:
    //--------------------------------------------------------------------------
    
protected:

    //! This method takes the current __diffuse__ color and updates __ambient__ and __specular__ components.
    void updateColors();


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS - GRAPHICS PROPERTIES:
    //--------------------------------------------------------------------------

protected:

    //! Material shininess level.
    GLuint m_shininess;

    //! Flag to track if related member has been modified.
    bool m_flag_shininess;


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS - HAPTIC PROPERTIES:
    //--------------------------------------------------------------------------

protected:

    ////////////////////////////////////////////////////////////////////////////
    // SURFACE STIFFNESS
    ////////////////////////////////////////////////////////////////////////////

    //! Stiffness [N/m].
    double m_stiffness;

    //! Flag to track if related member has been modified.
    bool m_flag_stiffness;


    ////////////////////////////////////////////////////////////////////////////
    // DAMPING
    ////////////////////////////////////////////////////////////////////////////

    // Damping level.
    double m_damping;

    //! Flag to track if related member has been modified.
    bool m_flag_damping;


    ////////////////////////////////////////////////////////////////////////////
    // VISCOSITY
    ////////////////////////////////////////////////////////////////////////////

    //! Level of viscosity.
    double m_viscosity;

    //! Flag to track if related member has been modified.
    bool m_flag_viscosity;


    ////////////////////////////////////////////////////////////////////////////
    // FRICTION
    ////////////////////////////////////////////////////////////////////////////

    //! Static friction constant [N].
    double m_staticFriction;

    //! Flag to track if related member has been modified.
    bool m_flag_staticFriction;

    //! Dynamic friction constant [N].
    double m_dynamicFriction;

    //! Flag to track if related member has been modified.
    bool m_flag_dynamicFriction;


    ////////////////////////////////////////////////////////////////////////////
    // TEXTURE
    ////////////////////////////////////////////////////////////////////////////

    //! Texture level constant.
    double m_textureLevel;

    //! Flag to track if related member has been modified.
    bool m_flag_textureLevel;


    ////////////////////////////////////////////////////////////////////////////
    // VIBRATION
    ////////////////////////////////////////////////////////////////////////////

    //! Frequency of vibrations [Hz].
    double m_vibrationFrequency;

    //! Flag to track if related member has been modified.
    bool m_flag_vibrationFrequency;

    //! Amplitude of vibrations [Hz].
    double m_vibrationAmplitude;

    //! Flag to track if related member has been modified.
    bool m_flag_vibrationAmplitude;
    

    ////////////////////////////////////////////////////////////////////////////
    // MAGNET
    ////////////////////////////////////////////////////////////////////////////

    //! Maximum force applied by magnetic effect [N].
    double m_magnetMaxForce;

    //! Flag to track if related member has been modified.
    bool m_flag_magnetMaxForce;

    //! Maximum distance from which magnetic forces can be perceived.
    double m_magnetMaxDistance;

    //! Flag to track if related member has been modified.
    bool m_flag_magnetMaxDistance;
    

    ////////////////////////////////////////////////////////////////////////////
    // STICK AND SLIP
    ////////////////////////////////////////////////////////////////////////////

    //! Force threshold for stick and slip effect [N].
    double m_stickSlipForceMax;

    //! Flag to track if related member has been modified.
    bool m_flag_stickSlipForceMax;

    //! Stiffness of stick slip model.
    double m_stickSlipStiffness;

    //! Flag to track if related member has been modified.
    bool m_flag_stickSlipStiffness;


    ////////////////////////////////////////////////////////////////////////////
    // HAPTIC MODES
    ////////////////////////////////////////////////////////////////////////////

    //! If __true__, haptic friction rendering is enabled.
    bool m_useHapticFriction;

    //! Flag to track if related member has been modified.
    bool m_flag_useHapticFriction;

    //! If __true__, haptic texture rendering is enabled.
    bool m_useHapticTexture;

    //! Flag to track if related member has been modified.
    bool m_flag_useHapticTexture;

    //! If __true__, haptic shading is enabled.
    bool m_useHapticShading;

    //! Flag to track if related member has been modified.
    bool m_flag_useHapticShading;

    //! If __true__, then front side of triangles are rendered haptically (used by the proxy algorithm).
    bool m_hapticFrontSideOfTriangles;

    //! Flag to track if related member has been modified.
    bool m_flag_hapticFrontSideOfTriangles;

    //! If __true__, then back side of triangles are rendered haptically (used by the proxy algorithm).
    bool m_hapticBackSideOfTriangles;

    //! Flag to track if related member has been modified.
    bool m_flag_hapticBackSideOfTriangles;


    ////////////////////////////////////////////////////////////////////////////
    // SOUND PROPERTIES
    ////////////////////////////////////////////////////////////////////////////

    //! Sound buffer associated with impacts.
    cAudioBuffer* m_audioImpactBuffer;

    //! Flag to track if related member has been modified.
    bool m_flag_audioImpactBuffer;

    //! Sound buffer associated with friction.
    cAudioBuffer* m_audioFrictionBuffer;

    //! Flag to track if related member has been modified.
    bool m_flag_audioFrictionBuffer;

    //! General gain for sound associated with impacts.
    double m_audioImpactGain;

    //! Flag to track if related member has been modified.
    bool m_flag_audioImpactGain;

    //! General gain for sounds associated with friction.
    double m_audioFrictionGain;

    //! Flag to track if related member has been modified.
    bool m_flag_audioFrictionGain;

    //! General pitch gain for sounds associated with friction.
    double m_audioFrictionPitchGain;

    //! Flag to track if related member has been modified.
    bool m_flag_audioFrictionPitchGain;

    //! General pitch offset for sounds associated with friction.
    double m_audioFrictionPitchOffset;

    //! Flag to track if related member has been modified.
    bool m_flag_audioFrictionPitchOffset;
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------

