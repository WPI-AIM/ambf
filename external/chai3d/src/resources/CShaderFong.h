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
    \author    Sonny Chan
    \version   3.2.0 $Rev: 2015 $
 */
//==============================================================================

//---------------------------------------------------------------------------
#ifndef CShaderFong
#define CShaderFong
//---------------------------------------------------------------------------
#include "system/CGlobals.h"
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
namespace chai3d {
//---------------------------------------------------------------------------

const std::string C_SHADER_FONG_VERT =
"                                                                                                                       \n"
"   attribute vec3 aPosition;                                                                                           \n"
"   attribute vec3 aNormal;                                                                                             \n"
"   attribute vec3 aTexCoord;                                                                                           \n"
"   attribute vec4 aColor;                                                                                              \n"
"   attribute vec3 aTangent;                                                                                            \n"
"   attribute vec3 aBitangent;                                                                                          \n"
"                                                                                                                       \n"
"   // vertex position and normal in eye coordinate space                                                               \n"
"   varying vec4 vPosition;                                                                                             \n"
"   varying vec3 vNormal;                                                                                               \n"
"                                                                                                                       \n"
"   //----------------------------------------------------------------------                                            \n"
"   // Main vertex shader code.                                                                                         \n"
"   //----------------------------------------------------------------------                                            \n"
"                                                                                                                       \n"
"   void main(void)                                                                                                     \n"
"   {                                                                                                                   \n"
"       // pass along a transformed vertex position, normal, and texture                                                \n"
"       vPosition = gl_ModelViewMatrix * gl_Vertex;                                                                     \n"
"       vNormal = gl_NormalMatrix * aNormal;                                                                            \n"
"       gl_TexCoord[0] = gl_TextureMatrix[0] * vec4(aTexCoord, 1.0);                                                    \n"
"                                                                                                                       \n"
"       // transform the vertex to get shadow map texture coordinates                                                   \n"
"       float s = dot(gl_EyePlaneS[1], vPosition);                                                                      \n"
"       float t = dot(gl_EyePlaneT[1], vPosition);                                                                      \n"
"       float r = dot(gl_EyePlaneR[1], vPosition);                                                                      \n"
"       float q = dot(gl_EyePlaneQ[1], vPosition);                                                                      \n"
"       gl_TexCoord[1] = vec4(s, t, r, q);                                                                              \n"
"                                                                                                                       \n"
"       // fixed function vertex transform                                                                              \n"
"       gl_Position = ftransform();                                                                                     \n"
"   }                                                                                                                   \n"
"                                                                                                                       \n";

const std::string C_SHADER_FONG_FRAG =
"                                                                                                                       \n"
"// interpolated vertex position in eye coordinate space (from vertex shader)                                           \n"
"   varying vec4 vPosition;                                                                                             \n"
"   varying vec3 vNormal;                                                                                               \n"
"                                                                                                                       \n"
"   // shadow (depth) map                                                                                               \n"
"   uniform sampler2DShadow uShadowMap;                                                                                 \n"
"                                                                                                                       \n"
"   //----------------------------------------------------------------------                                            \n"
"   // Computes lighting power and attenuation (Section 2.14.1 of specification)                                        \n"
"   //----------------------------------------------------------------------                                            \n"
"                                                                                                                       \n"
"   float attenuation(vec3 p, int i)                                                                                    \n"
"   {                                                                                                                   \n"
"       vec4 p_l = gl_LightSource[i].position;                                                                          \n"
"       if (p_l.w == 0.0) return 1.0;                                                                                   \n"
"                                                                                                                       \n"
"       float d = distance(p, p_l.xyz);                                                                                 \n"
"       float k0 = gl_LightSource[i].constantAttenuation;                                                               \n"
"       float k1 = gl_LightSource[i].linearAttenuation;                                                                 \n"
"       float k2 = gl_LightSource[i].quadraticAttenuation;                                                              \n"
"                                                                                                                       \n"
"       return 1.0 / (k0 + k1*d + k2*d*d);                                                                              \n"
"   }                                                                                                                   \n"
"                                                                                                                       \n"
"   float spotlight(vec3 p, int i)                                                                                      \n"
"   {                                                                                                                   \n"
"       if (gl_LightSource[i].spotCosCutoff < 0.0) return 1.0;                                                          \n"
"                                                                                                                       \n"
"       vec4 p_l = gl_LightSource[i].position;                                                                          \n"
"       if (p_l.w == 0.0) return 1.0;                                                                                   \n"
"                                                                                                                       \n"
"       vec3 v = normalize(p - p_l.xyz);                                                                                \n"
"       vec3 s = normalize(gl_LightSource[i].spotDirection);                                                            \n"
"                                                                                                                       \n"
"       float cosine = max(dot(v, s), 0.0);                                                                             \n"
"       if (cosine >= gl_LightSource[i].spotCosCutoff)                                                                  \n"
"           return pow(cosine, gl_LightSource[i].spotExponent);                                                         \n"
"       else return 0.0;                                                                                                \n"
"   }                                                                                                                   \n"
"                                                                                                                       \n"
"                                                                                                                       \n"
"   //----------------------------------------------------------------------                                            \n"
"   // Computes phong shading based on current light and material properties.                                           \n"
"   //----------------------------------------------------------------------                                            \n"
"                                                                                                                       \n"
"   vec4 shade(vec3 p, vec3 v, vec3 n)                                                                                  \n"
"   {                                                                                                                   \n"
"       vec3 Ie = gl_FrontMaterial.emission.rgb;                                                                        \n"
"       vec3 Ia = gl_FrontLightModelProduct.sceneColor.rgb;                                                             \n"
"       vec3 Il = vec3(0.0);                                                                                            \n"
"                                                                                                                       \n"
"       for (int i = 0; i < gl_MaxLights; ++i)                                                                          \n"
"       {                                                                                                               \n"
"           vec4 p_l = gl_LightSource[i].position;                                                                      \n"
"           vec3 l = normalize(p_l.xyz - p * p_l.w);                                                                    \n"
"           vec3 h = normalize(l + v);                                                                                  \n"
"           float s_m = gl_FrontMaterial.shininess;                                                                     \n"
"                                                                                                                       \n"
"           float cosNL = max(dot(n, l), 0.0);                                                                          \n"
"           float cosNH = max(dot(n, h), 0.0);                                                                          \n"
"                                                                                                                       \n"
"           vec3 phong = gl_FrontLightProduct[i].ambient.rgb                                                            \n"
"               + cosNL * gl_FrontLightProduct[i].diffuse.rgb                                                           \n"
"               + pow(cosNH, s_m) * gl_FrontLightProduct[i].specular.rgb;                                               \n"
"           Il += attenuation(p, i) * spotlight(p, i) * phong;                                                          \n"
"       }                                                                                                               \n"
"                                                                                                                       \n"
"       float alpha = gl_FrontMaterial.diffuse.a;                                                                       \n"
"       return vec4(Ie + Ia + Il, alpha);                                                                               \n"
"   }                                                                                                                   \n"
"                                                                                                                       \n"
"                                                                                                                       \n"
"   //----------------------------------------------------------------------                                            \n"
"   // Main fragment shader code.                                                                                       \n"
"   //----------------------------------------------------------------------                                            \n"
"                                                                                                                       \n"
"   void main(void)                                                                                                     \n"
"   {                                                                                                                   \n"
"       vec3 view = normalize(-vPosition.xyz);                                                                          \n"
"       vec3 normal = normalize(vNormal);                                                                               \n"
"       vec4 shadow = shadow2DProj(uShadowMap, gl_TexCoord[1]);                                                         \n"
"       gl_FragColor = vec4(shade(vPosition.xyz, view, normal).rgb, shadow.a);                                          \n"
"   }                                                                                                                   \n";


//---------------------------------------------------------------------------
} // namespace chai3d
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
