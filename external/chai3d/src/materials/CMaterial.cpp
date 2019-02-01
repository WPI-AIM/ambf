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
#include "materials/CMaterial.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    Constructor of cMaterial.
*/
//==============================================================================
cMaterial::cMaterial()
{
    // default graphic settings
    m_ambient.set(0.3f, 0.3f, 0.3f, 1.0f);
    m_diffuse.set(0.7f, 0.7f, 0.7f, 1.0f);
    m_specular.set(1.0f, 1.0f, 1.0f, 1.0f);
    m_emission.set(0.0f, 0.0f, 0.0f, 1.0f);
    m_shininess = 64;

    // default haptic settings
    m_viscosity             = 0.0;
    m_stiffness             = 0.0;
    m_damping               = 0.0;
    m_staticFriction        = 0.0;
    m_dynamicFriction       = 0.0;
    m_textureLevel          = 0.0;
    m_vibrationFrequency    = 0.0;
    m_vibrationAmplitude    = 0.0;
    m_stickSlipForceMax     = 0.0;
    m_stickSlipStiffness    = 0.0;

    m_useHapticFriction             = false;
    m_useHapticTexture              = false;
    m_useHapticShading              = false;
    m_hapticFrontSideOfTriangles    = true;
    m_hapticBackSideOfTriangles     = false;

    m_audioImpactBuffer             = NULL;
    m_audioFrictionBuffer           = NULL;
    m_audioImpactGain               = 0.0;
    m_audioFrictionGain             = 0.0;
    m_audioFrictionPitchGain        = 0.0;
    m_audioFrictionPitchOffset      = 1.0;

    // set all modification flags to false
    setModificationFlags(false);
}


//==============================================================================
/*!
    This method creates a copy of itself.

    \return Pointer to new instance.
*/
//==============================================================================
cMaterialPtr cMaterial::copy()
{
    // create new instance
    cMaterialPtr obj = create();

    // copy material properties
    obj->m_ambient              = m_ambient;
    obj->m_diffuse              = m_diffuse;
    obj->m_specular             = m_specular;
    obj->m_emission             = m_emission;
    obj->m_shininess            = m_shininess;
    obj->m_viscosity            = m_viscosity;
    obj->m_stiffness            = m_stiffness;
    obj->m_damping              = m_damping;
    obj->m_staticFriction       = m_staticFriction;
    obj->m_dynamicFriction      = m_dynamicFriction;
    obj->m_textureLevel         = m_textureLevel;
    obj->m_vibrationFrequency   = m_vibrationFrequency;
    obj->m_vibrationAmplitude   = m_vibrationAmplitude;
    obj->m_magnetMaxForce       = m_magnetMaxForce;
    obj->m_magnetMaxDistance    = m_magnetMaxDistance;
    obj->m_stickSlipForceMax    = m_stickSlipForceMax;
    obj->m_stickSlipStiffness   = m_stickSlipStiffness;

    obj->m_useHapticFriction            = m_useHapticFriction;
    obj->m_useHapticTexture             = m_useHapticTexture;
    obj->m_useHapticShading             = m_useHapticShading;
    obj->m_hapticFrontSideOfTriangles   = m_hapticFrontSideOfTriangles;
    obj->m_hapticBackSideOfTriangles    = m_hapticBackSideOfTriangles;

    obj->m_audioImpactBuffer            = m_audioImpactBuffer;
    obj->m_audioFrictionBuffer          = m_audioFrictionBuffer;
    obj->m_audioImpactGain              = m_audioImpactGain;
    obj->m_audioFrictionGain            = m_audioFrictionGain;
    obj->m_audioFrictionPitchGain       = m_audioFrictionPitchGain;
    obj->m_audioFrictionPitchOffset     = m_audioFrictionPitchOffset;

    // reset all flags
    obj->setModificationFlags(false);

    // return
    return (obj);
}


//==============================================================================
/*!
    This method sets the transparency level by setting the alpha value to all
    color members including __ambient__, __diffuse__, __specular__, and 
    __emission__.

    \param  a_levelTransparency  Level of transparency.
*/
//==============================================================================
void cMaterial::setTransparencyLevel(const float a_levelTransparency)
{
    // check that value is within range [0.0 - 1.0]
    float level = cClamp(a_levelTransparency, 0.0f, 1.0f);

    // apply new value
    m_ambient.setA(level);
    m_diffuse.setA(level);
    m_specular.setA(level);
    m_emission.setA(level);
}


//==============================================================================
/*!
    This method sets the level of shininess. The value is clamped to range 
    from 0 to 128.

    \param  a_shininess  Level of shininess
*/
//==============================================================================
void cMaterial::setShininess(const GLuint a_shininess)
{
    // update value and check range
    m_shininess = cClamp(a_shininess, (GLuint)0, (GLuint)128);

    // mark variable as modified
    m_flag_shininess = true;
}


//==============================================================================
/*!
    This method defines a color property for this material.

    \param  a_red    Red component
    \param  a_green  Green component
    \param  a_blue   Blue component
    \param  a_alpha  Alpha component
*/
//==============================================================================
void cMaterial::setColorf(const GLfloat a_red, 
                          const GLfloat a_green, 
                          const GLfloat a_blue,
                          const GLfloat a_alpha)
{
    m_diffuse.set(a_red, a_green, a_blue, a_alpha);
    updateColors();
}


//==============================================================================
/*!
    This method defines a color property for this material.

    \param  a_color  Color.
*/
//==============================================================================
void cMaterial::setColor(cColorf& a_color)
{
    m_diffuse = a_color;
    updateColors();
}


//==============================================================================
/*!
    Define a color property for the material.

    \param  a_color  Color.
*/
//==============================================================================
void cMaterial::setColor(cColorb& a_color)
{
    m_diffuse = a_color.getColorf();
    updateColors();
}


//==============================================================================
/*!
    When a new color is defined by calling methods such as \ref setColorf() or 
    \ref setRed(), the selected color is first set to the diffuse component.
    This method updates the ambient component by setting it equal to 50%
    of the diffuse color. The specular color is finally set to pure white.
*/
//==============================================================================
void cMaterial::updateColors()
{
    float level = 0.5;
    m_ambient.setR(level * m_diffuse.getR());
    m_ambient.setG(level * m_diffuse.getG());
    m_ambient.setB(level * m_diffuse.getB());
    m_ambient.setA(m_diffuse.getA());
    m_specular.set(1.0, 1.0, 1.0, m_diffuse.getA());
}


//==============================================================================
/*!
    This method sets the level of stiffness for this material.
    The value is clamped to be a non-negative value.

    \param  a_stiffness  Level of stiffness.
*/
//==============================================================================
void cMaterial::setStiffness(const double a_stiffness)
{
    // update value
    m_stiffness = cClamp0(a_stiffness);

    // mark variable as modified
    m_flag_stiffness = true;
}


//==============================================================================
/*!
    This method sets the level of damping.

    \param  a_damping  Level of damping.
*/
//==============================================================================
void cMaterial::setDamping(const double a_damping)
{
    // update value
    m_damping = a_damping;

    // mark variable as modified
    m_flag_damping = true;
}


//==============================================================================
/*!
    This method sets the level of static friction.
    The value is clamped to be a non-negative value.

    \param  a_friction  Level of friction.
*/
//==============================================================================
void cMaterial::setStaticFriction(const double a_friction)
{
    // update value
    m_staticFriction = cClamp0(a_friction);

    // enable friction rendering if required
    if ((m_staticFriction > 0) || (m_dynamicFriction > 0))
    {
        setUseHapticFriction(true);
    }
    else
    {
        setUseHapticFriction(false);
    }
    
    // mark variable as modified
    m_flag_staticFriction = true;
}


//==============================================================================
/*!
    This method sets the level of dynamic friction.
    The value is clamped to be a non-negative value.

    \param  a_friction  Level of friction.
*/
//==============================================================================
void cMaterial::setDynamicFriction(const double a_friction)
{
    // update value
    m_dynamicFriction = cClamp0(a_friction);

    // enable friction rendering if required
    if ((m_staticFriction > 0) || (m_dynamicFriction > 0))
    {
        setUseHapticFriction(true);
    }
    else
    {
        setUseHapticFriction(false);
    }
    
    // mark variable as modified
    m_flag_dynamicFriction = true;
}


//==============================================================================
/*!
    This method sets the intensity at which the texture map of an object is
    perceived.

    \param  a_textureLevel  Intensity level.
*/
//==============================================================================
void cMaterial::setTextureLevel(const double a_textureLevel)
{
    // update value
    m_textureLevel = a_textureLevel;

    // enable texture rendering if required
    if (m_textureLevel > 0)
    {
        setUseHapticTexture(true);
    }
    else
    {
        setUseHapticTexture(false);
    }

    // mark variable as modified
    m_flag_textureLevel = true;
}


//==============================================================================
/*!
    This method sets the level of viscosity.
    The value is clamped to be a non-negative value.

    \param  a_viscosity  Level of viscosity.
*/
//==============================================================================
void cMaterial::setViscosity(const double a_viscosity)
{
    // update value
    m_viscosity = cClamp0(a_viscosity);

    // mark variable as modified
    m_flag_viscosity = true;
}


//==============================================================================
/*!
    This method sets the frequency of vibration.
    The value is clamped to be a non-negative value.

    \param  a_vibrationFrequency  Frequency of vibration [Hz].
*/
//==============================================================================
void cMaterial::setVibrationFrequency(const double a_vibrationFrequency)
{
    // update value
    m_vibrationFrequency = cClamp0(a_vibrationFrequency);

    // mark variable as modified
    m_flag_vibrationFrequency = true;
}


//==============================================================================
/*!
    This method set the amplitude of vibration.
    The value is clamped to be a non-negative value.

    \param  a_vibrationAmplitude  Amplitude of vibrations [N].
*/
//==============================================================================
void cMaterial::setVibrationAmplitude(const double a_vibrationAmplitude)
{
    // update value
    m_vibrationAmplitude = cClamp0(a_vibrationAmplitude);

    // mark variable as modified
    m_flag_vibrationAmplitude = true;
}


//==============================================================================
/*!
    This method sets the maximum force applied by the magnet [N].
    The value is clamped to be a non-negative value.

    \param  a_magnetMaxForce  Maximum force of magnet.
*/
//==============================================================================
void cMaterial::setMagnetMaxForce(const double a_magnetMaxForce)
{
    // update value
    m_magnetMaxForce = cClamp0(a_magnetMaxForce);

    // mark variable as modified
    m_flag_magnetMaxForce = true;
}


//==============================================================================
/*!
    This method sets the maximum distance from which the magnetic force 
    can be perceived [m]

    \param  a_magnetMaxDistance  Maximum distance from object where 
                                 magnet is active.
*/
//==============================================================================
void cMaterial::setMagnetMaxDistance(const double a_magnetMaxDistance)
{
    // update value
    m_magnetMaxDistance = cClamp0(a_magnetMaxDistance);

    // mark variable as modified
    m_flag_magnetMaxDistance = true;
}


//==============================================================================
/*!
    This method sets the maximum force threshold for the stick and slip model [N].
    The value is clamped to be a non-negative value.

    \param  a_stickSlipForceMax  Maximum force threshold.
*/
//==============================================================================
void cMaterial::setStickSlipForceMax(const double a_stickSlipForceMax)
{
    // update value
    m_stickSlipForceMax = cClamp0(a_stickSlipForceMax);

    // mark variable as modified
    m_flag_stickSlipForceMax = true;
}


//==============================================================================
/*!
    This method sets the stiffness for the stick and slip model [N/m].

    \param  a_stickSlipStiffness  Stiffness property.
*/
//==============================================================================
void cMaterial::setStickSlipStiffness(const double a_stickSlipStiffness)
{
    // update value
    m_stickSlipStiffness = cClamp0(a_stickSlipStiffness);

    // mark variable as modified
    m_flag_stickSlipStiffness = true;
}


//==============================================================================
/*!
    This method enables or disables rendering of haptic friction.

    \param  a_useHapticFriction  If __true__, haptic friction rendering in enabled.
*/
//==============================================================================
void cMaterial::setUseHapticFriction(const bool a_useHapticFriction)
{
    // update value
    m_useHapticFriction = a_useHapticFriction;

    // mark variable as modified
    m_flag_useHapticFriction = true;
}


//==============================================================================
/*!
    This method enables or disables haptic texture rendering.

    \param  a_useHapticTexture  If __true__, haptic texture rendering in enabled.
*/
//==============================================================================
void cMaterial::setUseHapticTexture(const bool a_useHapticTexture)
{
    // update value
    m_useHapticTexture = a_useHapticTexture;

    // mark variable as modified
    m_flag_useHapticTexture = true;
}


//==============================================================================
/*!
    This method enables or disables haptic shading.

    \param  a_useHapticShading  If __true__, haptic shading in enabled.
*/
//==============================================================================
void cMaterial::setUseHapticShading(const bool a_useHapticShading)
{
    // update value
    m_useHapticShading = a_useHapticShading;

    // mark variable as modified
    m_flag_useHapticShading = true;
}


//==============================================================================
/*!
    This method enables or disables __front__ side haptic rendering. \n

    If \p a_enabled is set to __true__, then haptic rendering occurs on front 
    side of triangles. This option applies to mesh objects which are rendered 
    using the proxy force algorithm.

    \param  a_enabled  If __true__, then haptic rendering is enabled.
*/
//==============================================================================
void cMaterial::setHapticTriangleFrontSide(const bool a_enabled)
{
    // update value
    m_hapticFrontSideOfTriangles = a_enabled;

    // mark variable as modified
    m_flag_hapticFrontSideOfTriangles = true;
}


//==============================================================================
/*!
    This method enables or disables __back__ side haptic rendering. \n

    If \p a_enabled is set to __true__, then haptic rendering occurs on back 
    side of triangles. This option applies to mesh objects which are rendered 
    using the proxy force algorithm.

    \param  a_enabled  If __true__, then haptic rendering is enabled.
*/
//==============================================================================
void cMaterial::setHapticTriangleBackSide(const bool a_enabled)
{
    // update value
    m_hapticBackSideOfTriangles = a_enabled;

    // mark variable as modified
    m_flag_hapticBackSideOfTriangles = true;
}


//==============================================================================
/*!
    This method defines which sides haptic rendering must occur with triangles.

    \param  a_enableFrontSide  If __true__, then haptic rendering is enabled for front sides.
    \param  a_enableBackSide   If __true__, then haptic rendering is enabled for back sides.
*/
//==============================================================================
void cMaterial::setHapticTriangleSides(const bool a_enableFrontSide, 
    const bool a_enableBackSide)
{
    // update front side
    setHapticTriangleFrontSide(a_enableFrontSide);

    // update back side
    setHapticTriangleBackSide(a_enableBackSide);
}


//==============================================================================
/*!
    This method sets an audio buffer associated with any impacts between 
    a haptic tool and an object.

    \param  a_audioImpactBuffer  Audio buffer.
*/
//==============================================================================
void cMaterial::setAudioImpactBuffer(cAudioBuffer* a_audioImpactBuffer)
{
    // update value
    m_audioImpactBuffer = a_audioImpactBuffer;

    // mark variable as modified
    m_flag_audioImpactBuffer = true;
}


//==============================================================================
/*!
    This method sets an audio buffer associated with any friction caused by a 
    haptic tool touching an object.

    \param  a_audioFrictionBuffer  Audio buffer.
*/
//==============================================================================
void cMaterial::setAudioFrictionBuffer(cAudioBuffer* a_audioFrictionBuffer)
{
    // update value
    m_audioFrictionBuffer = a_audioFrictionBuffer;

    // mark variable as modified
    m_flag_audioFrictionBuffer = true;
}


//==============================================================================
/*!
    This method sets the audio gain related to impact sounds.

    \param  a_audioImpactGain  Audio gain associated to impact.
*/
//==============================================================================
void cMaterial::setAudioImpactGain(const double a_audioImpactGain)
{
    // update value
    m_audioImpactGain = a_audioImpactGain;

    // mark variable as modified
    m_flag_audioImpactGain = true;
}


//==============================================================================
/*!
    This method sets the audio gain related to friction sounds.

    \param  a_audioFrictionGain  Audio gain associated to friction.
*/
//==============================================================================
void cMaterial::setAudioFrictionGain(const double a_audioFrictionGain)
{
    // update value
    m_audioFrictionGain = a_audioFrictionGain;

    // mark variable as modified
    m_flag_audioFrictionGain = true;
}

 
//==============================================================================
/*!
    This method sets the audio friction pitch gain.

    \param  a_audioFrictionPitchGain  Audio pitch gain associated to friction.
*/
//==============================================================================
void cMaterial::setAudioFrictionPitchGain(const double a_audioFrictionPitchGain)
{
    // update value
    m_audioFrictionPitchGain = a_audioFrictionPitchGain;

    // mark variable as modified
    m_flag_audioFrictionPitchGain = true;
}


//==============================================================================
/*!
    This method sets the audio friction pitch offset.

    \param  a_audioFrictionPitchOffset  Audio pitch offset associated to friction.
*/
//==============================================================================
void cMaterial::setAudioFrictionPitchOffset(const double a_audioFrictionPitchOffset)
{
    // update value
    m_audioFrictionPitchOffset = a_audioFrictionPitchOffset;

    // mark variable as modified
    m_flag_audioFrictionPitchOffset = true;
}


//==============================================================================
/*!
    This method renders this material using OpenGL.

    \param  a_options  Rendering options.
*/
//==============================================================================
void cMaterial::render(cRenderOptions& a_options)
{
    // check if materials should be rendered
    if (!a_options.m_render_materials) { return; }

    // render material
#ifdef C_USE_OPENGL
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, (const float *)&m_ambient);
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, (const float *)&m_diffuse);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, (const float *)&m_specular);
    glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, (const float *)&m_emission);
    glMateriali(GL_FRONT_AND_BACK, GL_SHININESS, m_shininess);
#endif
}


//==============================================================================
/*!
    This method sets all the modification flags to a desired value.

    \param  a_value  Value to be assigned to modification flags.
*/
//==============================================================================
void cMaterial::setModificationFlags(const bool a_value)
{
    m_flag_shininess                    = a_value;
    m_flag_viscosity                    = a_value;
    m_flag_stiffness                    = a_value;
    m_flag_damping                      = a_value;
    m_flag_staticFriction               = a_value;
    m_flag_dynamicFriction              = a_value;
    m_flag_textureLevel                 = a_value;
    m_flag_vibrationFrequency           = a_value;
    m_flag_vibrationAmplitude           = a_value;
    m_flag_magnetMaxForce               = a_value;
    m_flag_magnetMaxDistance            = a_value;
    m_flag_stickSlipForceMax            = a_value;
    m_flag_stickSlipStiffness           = a_value;
    m_flag_useHapticTexture             = a_value;
    m_flag_useHapticShading             = a_value;
    m_flag_hapticFrontSideOfTriangles   = a_value;
    m_flag_hapticBackSideOfTriangles    = a_value;

    m_flag_audioImpactBuffer            = a_value;
    m_flag_audioFrictionBuffer          = a_value;
    m_flag_audioImpactGain              = a_value;
    m_flag_audioFrictionGain            = a_value;
    m_flag_audioFrictionPitchGain       = a_value;
    m_flag_audioFrictionPitchOffset     = a_value;

    m_ambient.setModificationFlags(a_value);
    m_diffuse.setModificationFlags(a_value);
    m_specular.setModificationFlags(a_value);
    m_emission.setModificationFlags(a_value);
}


//==============================================================================
/*!
    This method copies all modified variables to material object passed
    as argument.

    \param  a_material  Destination material where values are copied to.
*/
//==============================================================================
void cMaterial::copyTo(cMaterialPtr a_material)
{
    if (m_flag_shininess)         
        a_material->setShininess(m_shininess);
    if (m_flag_viscosity)
        a_material->setViscosity(m_viscosity); 
    if (m_flag_stiffness)      
        a_material->setStiffness(m_stiffness);
    if (m_flag_damping)      
        a_material->setDamping(m_flag_damping);
    if (m_flag_staticFriction)  
        a_material->setStaticFriction(m_staticFriction);
    if (m_flag_dynamicFriction)    
        a_material->setDynamicFriction(m_dynamicFriction);
    if (m_flag_textureLevel)    
        a_material->setTextureLevel(m_textureLevel);
    if (m_flag_vibrationFrequency)  
        a_material->setVibrationFrequency(m_vibrationFrequency);
    if (m_flag_vibrationAmplitude)  
        a_material->setVibrationAmplitude(m_vibrationAmplitude);
    if (m_flag_magnetMaxForce)      
        a_material->setMagnetMaxForce(m_magnetMaxForce);
    if (m_flag_magnetMaxDistance)    
        a_material->setMagnetMaxDistance(m_magnetMaxDistance);
    if (m_flag_stickSlipForceMax)   
        a_material->setStickSlipForceMax(m_stickSlipForceMax);
    if (m_flag_stickSlipStiffness)  
        a_material->setStickSlipStiffness(m_stickSlipStiffness);
    if (m_flag_hapticFrontSideOfTriangles)
        a_material->setHapticTriangleFrontSide(m_hapticFrontSideOfTriangles);
    if (m_flag_hapticBackSideOfTriangles)
        a_material->setHapticTriangleBackSide(m_hapticBackSideOfTriangles);

    if (m_flag_audioImpactBuffer)
        a_material->setAudioImpactBuffer(m_audioImpactBuffer);
    if (m_flag_audioFrictionBuffer)
        a_material->setAudioFrictionBuffer(m_audioFrictionBuffer);
    if (m_flag_audioImpactGain)         
        a_material->setAudioImpactGain(m_audioImpactGain);
    if (m_flag_audioFrictionGain)         
        a_material->setAudioFrictionGain(m_audioFrictionGain);
    if (m_flag_audioFrictionPitchGain)         
        a_material->setAudioFrictionPitchGain(m_audioFrictionPitchGain);
    if (m_flag_audioFrictionPitchOffset)         
        a_material->setAudioFrictionPitchOffset(m_audioFrictionPitchOffset);

    m_ambient.copyTo(a_material->m_ambient);
    m_diffuse.copyTo(a_material->m_diffuse);
    m_specular.copyTo(a_material->m_specular);
    m_emission.copyTo(a_material->m_emission);
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
