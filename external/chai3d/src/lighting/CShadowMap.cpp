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
#include "lighting/CShadowMap.h"
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
    Constructor of cShadowMap.
*/
//==============================================================================
cShadowMap::cShadowMap()
{
    // setup default values of shadowmap framebuffer
    setup(NULL,    // no camera attached
          256,     // resolution in pixels (width)
          256,     // resolution in pixels (height)
          false,   // disable color buffer
          true);   // enable depth buffer.

    // assign texture unit for shadows.
    m_depthBuffer->setTextureUnit(GL_TEXTURE1);
}


//==============================================================================
/*!
    This method contains the OpenGL rendering code for the shadowmap.
    This code is called internally by CHAI3D. Do not call this method
    directly.

    \param  a_options  Rendering options.
*/
//==============================================================================
void cShadowMap::render(cRenderOptions& a_options)
{
#ifdef C_USE_OPENGL

    // check if shadow map is enabled
    if (!m_enabled) { return; }

    // check if shadow texture map has been initialized 
    if (m_depthBuffer->m_image->isInitialized() == 0) return;

    // bind the texture object used to store the depthmap
    if (m_depthBuffer->getTextureId() == 0)
    {
        return;
    }
    else
    {
        glBindTexture(GL_TEXTURE_2D, m_depthBuffer->getTextureId());
    }

    // calculate texture matrix for projection;
    // this matrix takes us from eye space to the light's clip space;
    // it is postmultiplied by the inverse of the current view matrix when specifying texgen
    cTransform biasMatrix;
    biasMatrix.set(0.5f, 0.0f, 0.0f, 0.0f,
                   0.0f, 0.5f, 0.0f, 0.0f,
                   0.0f, 0.0f, 0.5f, 0.0f,
                   0.5f, 0.5f, 0.5f, 1.0f);  //bias from [-1, 1] to [0, 1]
    
    // textureMatrix = biasMatrix * lightProjectionMatrix * lightViewMatrix
    cTransform textureMatrix = biasMatrix * m_lightProjectionMatrix * m_lightViewMatrix;

    // set up texture coordinate generation
    double matRow0[4];
    matRow0[0] = textureMatrix.m[0][0];
    matRow0[1] = textureMatrix.m[1][0];
    matRow0[2] = textureMatrix.m[2][0];
    matRow0[3] = textureMatrix.m[3][0];

    double matRow1[4];
    matRow1[0] = textureMatrix.m[0][1];
    matRow1[1] = textureMatrix.m[1][1];
    matRow1[2] = textureMatrix.m[2][1];
    matRow1[3] = textureMatrix.m[3][1];
    
    double matRow2[4];
    matRow2[0] = textureMatrix.m[0][2];
    matRow2[1] = textureMatrix.m[1][2];
    matRow2[2] = textureMatrix.m[2][2];
    matRow2[3] = textureMatrix.m[3][2];
    
    double matRow3[4];
    matRow3[0] = textureMatrix.m[0][3];
    matRow3[1] = textureMatrix.m[1][3];
    matRow3[2] = textureMatrix.m[2][3];
    matRow3[3] = textureMatrix.m[3][3];

    // enable texture
    glActiveTexture(m_depthBuffer->getTextureUnit());
   
    // setup parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, m_depthBuffer->getMinFunction());       
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, m_depthBuffer->getMagFunction());    
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, m_depthBuffer->getWrapModeS());  
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, m_depthBuffer->getWrapModeT()); 

    glTexGeni(GL_S, GL_TEXTURE_GEN_MODE, GL_EYE_LINEAR);
    glTexGendv(GL_S, GL_EYE_PLANE, &matRow0[0]);
    glEnable(GL_TEXTURE_GEN_S);

    glTexGeni(GL_T, GL_TEXTURE_GEN_MODE, GL_EYE_LINEAR);
    glTexGendv(GL_T, GL_EYE_PLANE, &matRow1[0]);
    glEnable(GL_TEXTURE_GEN_T);

    glTexGeni(GL_R, GL_TEXTURE_GEN_MODE, GL_EYE_LINEAR);
    glTexGendv(GL_R, GL_EYE_PLANE, &matRow2[0]);
    glEnable(GL_TEXTURE_GEN_R);

    glTexGeni(GL_Q, GL_TEXTURE_GEN_MODE, GL_EYE_LINEAR);
    glTexGendv(GL_Q, GL_EYE_PLANE, &matRow3[0]);
    glEnable(GL_TEXTURE_GEN_Q);

    // bind & enable shadow map texture
    glBindTexture(GL_TEXTURE_2D, m_depthBuffer->getTextureId());
    glEnable(GL_TEXTURE_2D);

    if(a_options.m_render_transparent_front_faces_only && a_options.m_rendering_shadow)
    {
        // enable shadow comparison
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_R_TO_TEXTURE);

        // shadow comparison should be true (i.e. not in shadow) if r<=texture
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_GEQUAL);  // GL_LESS

        // shadow comparison should generate an INTENSITY result
        glTexParameteri(GL_TEXTURE_2D, GL_DEPTH_TEXTURE_MODE, GL_INTENSITY);

        // set alpha test to discard false comparisons
        glAlphaFunc(GL_LEQUAL, 0.99f);
        glEnable(GL_ALPHA_TEST);
    }
    else
    {
        // enable shadow comparison
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_R_TO_TEXTURE);

        // shadow comparison should be true (i.e. not in shadow) if r<=texture
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);  // GL_LESS

        // shadow comparison should generate an INTENSITY result
        glTexParameteri(GL_TEXTURE_2D, GL_DEPTH_TEXTURE_MODE, GL_INTENSITY);

        // set alpha test to discard false comparisons
        glAlphaFunc(GL_GEQUAL, 0.99f);
        glEnable(GL_ALPHA_TEST);
    }

#endif
}


//==============================================================================
/*!
    This method creates a shadow map by providing a world and view point information.

    \param  a_world                World in which to create shadow
    \param  a_lightPos             Position of eye.
    \param  a_lightLookat          Eye lookat position
    \param  a_lightUp              Eye orientation (up vector).
    \param  a_lightFieldViewAngle  Field of view of light source
    \param  a_distanceNear         Distance to near clipping plane.
    \param  a_distanceFar          Distance to far clipping plane.
    \param  a_mirrorH              Scale factor for horizontal mirroring (-1.0 or 1.0)
    \param  a_mirrorV              Scale factor for vertical mirroring (-1.0 or 1.0)

    \return __true__ if shadow map was created successfully. __false__ otherwise.
*/
//==============================================================================
bool cShadowMap::updateMap(cWorld* a_world,
    const cVector3d& a_lightPos,
    const cVector3d& a_lightLookat,
    const cVector3d& a_lightUp,
    const double a_lightFieldViewAngle,
    const double a_distanceNear,
    const double a_distanceFar,
    const double a_mirrorH,
    const double a_mirrorV)
{
#ifdef C_USE_OPENGL

    // check if shadow is enabled
    if (!m_enabled) { return (false); }

    // check for necessary OpenGL extensions
#ifdef GLEW_VERSION
    if(!(GLEW_ARB_depth_texture && GLEW_ARB_shadow && GLEW_ARB_framebuffer_object))
    {
        return (false);
    }
#endif

    // verify size of image
    int width  = m_depthBuffer->m_image->getWidth();
    int height = m_depthBuffer->m_image->getHeight();
    if (!((width > 0) && (height > 0))) { return (false); }

    if (renderInitialize())
    {
        //---------------------------------------------------------------------------
        // Render the world from the point of view of the camera.
        // Here we are only interested in rendering the depth buffer.
        //---------------------------------------------------------------------------

        // we shall not render any colors, only depth information
        glDrawBuffer(GL_NONE);
        glReadBuffer(GL_NONE);

        // setup our viewport
        glViewport(0, 0, width, height);

        // compute aspect ratio
        double glAspect = ((double)width / (double)height);
    
        // setup projection matrix
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();

        // adjust display for mirroring
        glScalef((GLfloat)a_mirrorH, (GLfloat)a_mirrorV, (GLfloat)1.0);

        gluPerspective(
                2.0 * a_lightFieldViewAngle,   // Field of View Angle.
                glAspect,           // Aspect ratio of viewing volume.
                a_distanceNear,     // Distance to Near clipping plane.
                a_distanceFar);     // Distance to Far clipping plane.


        // setup modelview matrix
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        gluLookAt( a_lightPos(0) ,    a_lightPos(1) ,    a_lightPos(2) ,
                   a_lightLookat(0) , a_lightLookat(1) , a_lightLookat(2) ,
                   a_lightUp(0) ,     a_lightUp(1) ,     a_lightUp(2)  );

        // Back up the view and projection matrix for future reference
        cTransform projectionMatrix;
        cTransform viewMatrix;
        glGetDoublev(GL_PROJECTION_MATRIX, projectionMatrix.getData());
        glGetDoublev(GL_MODELVIEW_MATRIX, viewMatrix.getData());
        m_lightProjectionMatrix = projectionMatrix;
        m_lightViewMatrix = viewMatrix;

        // draw back faces only into the shadow map
        glCullFace(GL_FRONT);
    
        // disable color writes, and use flat shading for speed
        glShadeModel(GL_FLAT);
        glColorMask(0, 0, 0, 0);

        // set up OpenGL state
        glEnable(GL_LIGHTING);
        glDisable(GL_BLEND);
        glDepthMask(GL_TRUE);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_POLYGON_OFFSET_FILL);
        glPolygonOffset(1.0, 1.0);
        glDisable(GL_POLYGON_OFFSET_FILL);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

        // clear depth buffer
        glClear(GL_DEPTH_BUFFER_BIT);

        // draw the scene
        cRenderOptions options;
        options.m_camera                                = NULL;
        options.m_single_pass_only                      = true;
        options.m_render_opaque_objects_only            = false;
        options.m_render_transparent_front_faces_only   = false;
        options.m_render_transparent_back_faces_only    = false;
        options.m_enable_lighting                       = true;
        options.m_render_materials                      = true;
        options.m_render_textures                       = true;
        options.m_creating_shadow_map                   = true;
        options.m_rendering_shadow                      = true;
        options.m_shadow_light_level                    = 1.0;
        options.m_storeObjectPositions                  = true;
        options.m_markForUpdate                         = false;

        // render single pass (all objects)
        a_world->renderSceneGraph(options);

        // finalize
        renderFinalize();

        // return success
        return (true);
    }
    else
    {
        return (false);
    }

#else // C_USE_OPENGL
    return (false);
#endif
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
