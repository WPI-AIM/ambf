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
    \version   3.2.0 $Rev: 2159 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "world/CWorld.h"
//------------------------------------------------------------------------------
#include "lighting/CSpotLight.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==========================================================================
/*!
    Constructor of cWorld.
*/
//==============================================================================
cWorld::cWorld()
{
    // create fog
    m_fog = new cFog();

    // setup background color
    m_backgroundColor.set(0.0f, 0.0f, 0.0f, 1.0f);

    // light sources shell be rendered
    m_renderLightSources = true;  

    // set intensity of shadows
    m_shadowIntensity = 0.7;

    // use shadow maps
    m_useShadowCasting = true;

    // initialize matrix
    memset(m_worldModelView, 0, sizeof(m_worldModelView));
}


//==============================================================================
/*!
    Destructor of cWorld. \n
*/
//==============================================================================
cWorld::~cWorld()
{
    delete m_fog;
}


//==============================================================================
/*!
    This method sets the background color of the world.

    \param  a_red    __red__ component.
    \param  a_green  __green__ component.
    \param  a_blue   __blue__ component.
*/
//==============================================================================
void cWorld::setBackgroundColor(const GLfloat a_red, const GLfloat a_green,
                               const GLfloat a_blue)
{
    m_backgroundColor.set(a_red, a_green, a_blue);
}


//==============================================================================
/*!
    This method sets the background color of the world.

    \param  a_color  Background color.
*/
//==============================================================================
void cWorld::setBackgroundColor(const cColorf& a_color)
{
    m_backgroundColor = a_color;
}


//==============================================================================
/*!
    This method adds an OpenGL light source to the world. \n
    A maximum of eight light sources can be registered. For each registered light
    source, an OpenGL lightID number is defined

    \param  a_light  Light source to register.

    \return __true__ if light source was registered, otherwise __false__.
*/
//==============================================================================
bool cWorld::addLightSource(cGenericLight* a_light)
{
    // check if number of lights already equal to 8.
    if (m_lights.size() >= C_MAXIMUM_OPENGL_LIGHT_COUNT)
    {
        return (false);
    }

    // search for a free ID number
    int light_id = GL_LIGHT0;
    bool found = false;

    while (light_id < GL_LIGHT0+C_MAXIMUM_OPENGL_LIGHT_COUNT)
    {
        
        // check if ID is not already used
        unsigned int i;
        bool free = true;
        for (i=0; i<m_lights.size(); i++)
        {
            cGenericLight* nextLight = m_lights[i];

            if (nextLight->getGLLightNumber() == light_id)
            {
                free = false;
            }
        }

        // check if a free ID was found
        if (free)
        {
            a_light->setGLLightNumber(light_id);
            found = true;
            break;
        }

        light_id++;
    }

    // finalize
    if (found)
    {
        m_lights.push_back(a_light);
        return (true);
    }
    else
    {
        return (false);
    }
}


//==============================================================================
/*!
    This method removes a light source from this world.

    \param  a_light  Light source to be removed.

    \return __true__ if light source was removed, otherwise __false__.
*/
//==============================================================================
bool cWorld::removeLightSource(cGenericLight* a_light)
{
    // set iterator
    std::vector<cGenericLight*>::iterator nextLight;

    for(nextLight = m_lights.begin();
        nextLight != m_lights.end();
        nextLight++ ) 
    {
        if ((*nextLight) == a_light)
        {
            // remove object from list
            m_lights.erase(nextLight);

            // return success
            return (true);
        }
    }

    // operation failed
    return (false);
}


//==============================================================================
/*!
    This method returns a pinter to a selected light source 
    (between 0 and MAXIMUM_OPENGL_LIGHT_COUNT-1).

    \param  a_index  Light index number.

    \return Pointer to a valid light or __null__ if that light doesn't exist.
*/
//==============================================================================
cGenericLight* cWorld::getLightSource(int a_index) 
{
    // Make sure this is a valid index
    if (a_index < 0 || (unsigned int)(a_index) >= m_lights.size())
    {
        return (NULL);
    }

    // Return the light that we were supplied with by the creator of the world
    return (m_lights[a_index]);
}


//==============================================================================
/*!
    This method updates all shadow maps.

    \param  a_mirrorX         Set to __true__ if image is flip horizontally.
    \param  a_mirrorY         Set to __true__ if image is flip vertically.

    \return __true__ if any shadow maps were updated.
*/
//==============================================================================
bool cWorld::updateShadowMaps(const bool a_mirrorX,
                              const bool a_mirrorY)
{
    // clear current list of shadow maps
    m_shadowMaps.clear();

    // init variables
    bool mapsUpdated = false;
    double scaleX = 1.0;
    double scaleY = 1.0;

    if (a_mirrorX)
    {
        scaleX =-1.0;
    }

    if (a_mirrorY)
    {
        scaleY =-1.0;
    }

    // shadow casting has been requested, we will first need to verify if the hardware
    // can support it
    if (m_useShadowCasting)
    {
        // we verify if shadow casting is supported by hardware
        if (isShadowCastingSupported())
        {
            // we check every light source. if it is a spot light we verify if shadow casting is enabled
            for (unsigned int i=0; i<m_lights.size(); i++)
            {
                cSpotLight* light =  dynamic_cast<cSpotLight*>(getLightSource(i));
                if (light != NULL)
                {
                    if (light->getShadowMapEnabled())
                    {
                        // update shadow map
                        if (light->updateShadowMap(scaleX, scaleY))
                        {
                            // add shadow map to list
                            m_shadowMaps.push_back(light->m_shadowMap);

                            // shadow mapping is used by at least one light source!
                            mapsUpdated = true;
                        }
                    }
                }
            }
        }
    }

    return (mapsUpdated);
}


//==============================================================================
/*!
    This method returns __true__ if shadow casting is supported on this
    hardware, __false__ otherwise.

    \return __true__ if shadow casting is supported, otherwise __false__.
*/
//==============================================================================
bool cWorld::isShadowCastingSupported()
{
#ifdef C_USE_OPENGL
    #ifdef GLEW_VERSION
        //Check for necessary OpenGL extensions
        if(!GLEW_ARB_depth_texture || 
           !GLEW_ARB_shadow)
        {
            return (false);
        }
        else
        {
            return (true);
        }
    #else
        return (true);
    #endif
#else
    return (false);
#endif
}


//==============================================================================
/*!
    This method renders all light sources of this world.

    \param  a_options  Rendering options.
*/
//==============================================================================
void cWorld::render(cRenderOptions& a_options)
{
#ifdef C_USE_OPENGL

    // make sure that these values always remain at origin.
    // translating the world can create a bug with shadow casting!
    m_localPos.zero();
    m_globalPos.zero();
    m_localRot.identity();
    m_globalRot.identity();

    // set up the CHAI3D openGL defaults (see cGenericObject::render())
    glEnable(GL_LIGHTING);
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);

    // turn off all light sources for now
    for (int i=0; i<C_MAXIMUM_OPENGL_LIGHT_COUNT; i++)
    {
        glDisable(GL_LIGHT0+i);
    }

    // back up the "global" modelview matrix for future reference
    glGetDoublev(GL_MODELVIEW_MATRIX, m_worldModelView);

    // render light sources
    if (m_renderLightSources && a_options.m_enable_lighting) 
    {
        // enable lighting
        glEnable(GL_LIGHTING);

        // render light sources
        unsigned int i;
        for (i=0; i<m_lights.size(); i++)
        {
            m_lights[i]->renderLightSource(a_options);
        }    
    }

    // render fog
    m_fog->render(a_options);

#endif
}


//==============================================================================
/*!
    This method determines whether a given segment intersects any object in
    this world. \n
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
bool cWorld::computeCollisionDetection(const cVector3d& a_segmentPointA,
                                       const cVector3d& a_segmentPointB,
                                       cCollisionRecorder& a_recorder,
                                       cCollisionSettings& a_settings)
{
    // temp variable
    bool hit = false;

    // check for collisions with all children of this world
    unsigned int nChildren = (int)(m_children.size());
    for (unsigned int i=0; i<nChildren; i++)
    {
        hit = hit | m_children[i]->computeCollisionDetection(a_segmentPointA,
                                                       a_segmentPointB,
                                                       a_recorder,
                                                       a_settings);
    }

    // return whether there was a collision between the segment and this world
    return (hit);
}


//==============================================================================
/*!
    This method update interaction information between a tool and this world.

    \param  a_toolPos  Position of the tool.
    \param  a_toolVel  Velocity of the tool.
    \param  a_IDN      Identification number of the force algorithm.
*/
//==============================================================================
void cWorld::computeLocalInteraction(const cVector3d& a_toolPos,
                                     const cVector3d& a_toolVel,
                                     const unsigned int a_IDN)
{
    // no surface boundary defined, so we simply return the same position of the tool
    m_interactionPoint = a_toolPos;

    if (m_interactionPoint.lengthsq() > 0)
    {
        m_interactionNormal = m_interactionPoint;
        m_interactionNormal.normalize();
    }
    else
    {
        m_interactionNormal.set(0,0,1);
    }

    // no surface boundary, so we consider that we are always inside the world
    m_interactionInside = true;
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
