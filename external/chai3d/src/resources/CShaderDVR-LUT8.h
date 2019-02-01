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
#ifndef CShaderDVRLUT8
#define CShaderDVRLUT8
//---------------------------------------------------------------------------
#include "system/CGlobals.h"
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
namespace chai3d {
//---------------------------------------------------------------------------

const std::string C_SHADER_DVR_LUT8_VERT = 
"                                                                                                                       \n"
"   attribute vec3 aPosition;                                                                                           \n"
"   attribute vec3 aNormal;                                                                                             \n"
"   attribute vec3 aTexCoord;                                                                                           \n"
"   attribute vec4 aColor;                                                                                              \n"
"   attribute vec3 aTangent;                                                                                            \n"
"   attribute vec3 aBitangent;                                                                                          \n"
"                                                                                                                       \n"
"   varying vec4 vPosition;                                                                                             \n"
"                                                                                                                       \n"
"   //----------------------------------------------------------------------                                            \n"
"   //  Main vertex shader code.                                                                                        \n"
"   //----------------------------------------------------------------------                                            \n"
"                                                                                                                       \n"
"   void main(void)                                                                                                     \n"
"   {                                                                                                                   \n"
"       gl_TexCoord[0] = gl_TextureMatrix[0] * vec4(aTexCoord, 1.0);                                                    \n"
"       vPosition = gl_Vertex;                                                                                          \n"
"       gl_Position = ftransform();                                                                                     \n"
"   }                                                                                                                   \n"
"                                                                                                                       \n";

const std::string C_SHADER_DVR_LUT8_FRAG = 
"                                                                                                                       \n"
"   uniform vec3 uMinCorner;              // lower bounding corner, model space                                         \n"
"   uniform vec3 uMaxCorner;              // upper bounding corner, model space                                         \n"
"   uniform vec3 uTextureScale;           // convert model to texture coordinates                                       \n"
"   uniform vec3 uGradientDelta;          // one voxel step in texture space                                            \n"
"   uniform sampler3D uVolume;            // volume texture                                                             \n"
"   uniform sampler1D uColorLUT;          // 1D transfer function texture                                               \n"
"   uniform float uIsosurface;            // isosurface level                                                           \n"
"   uniform float uOpacityThreshold;      // opacity threshold                                                          \n"
"   uniform float uOpticalDensityFactor;  // Optical density factor                                                     \n"
"   uniform float uResolution;            // max samples to take on ray traversal                                       \n"
"                                                                                                                       \n"
"   varying vec4 vPosition;                                                                                             \n"
"                                                                                                                       \n"
"   // deltas for gradient estimation along each axis in texture space                                                  \n"
"   vec3 dx = vec3(uGradientDelta.x, 0.0, 0.0);                                                                         \n"
"   vec3 dy = vec3(0.0, uGradientDelta.y, 0.0);                                                                         \n"
"   vec3 dz = vec3(0.0, 0.0, uGradientDelta.z);                                                                         \n"
"                                                                                                                       \n"
"                                                                                                                       \n"
"   //----------------------------------------------------------------------                                            \n"
"   // Finds the entering intersection between a ray e1+d and the volume's                                              \n"
"   // bounding box.                                                                                                    \n"
"   //----------------------------------------------------------------------                                            \n"
"                                                                                                                       \n"
"   float entry(vec3 e1, vec3 d)                                                                                        \n"
"   {                                                                                                                   \n"
"       float t = distance(uMinCorner, uMaxCorner);                                                                     \n"
"                                                                                                                       \n"
"       vec3 a = (uMinCorner - e1) / d;                                                                                 \n"
"       vec3 b = (uMaxCorner - e1) / d;                                                                                 \n"
"       vec3 u = min(a, b);                                                                                             \n"
"                                                                                                                       \n"
"       return max( max(-t, u.x), max(u.y, u.z) );                                                                      \n"
"   }                                                                                                                   \n"
"                                                                                                                       \n"
"                                                                                                                       \n"
"   //----------------------------------------------------------------------                                            \n"
"   // Estimates the intensity gradient of the volume in model space                                                    \n"
"   //----------------------------------------------------------------------                                            \n"
"                                                                                                                       \n"
"   vec3 gradient(vec3 tc)                                                                                              \n"
"   {                                                                                                                   \n"
"       vec3 nabla = vec3(                                                                                              \n"
"           texture3D(uVolume, tc + dx).r - texture3D(uVolume, tc - dx).r,                                              \n"
"           texture3D(uVolume, tc + dy).r - texture3D(uVolume, tc - dy).r,                                              \n"
"           texture3D(uVolume, tc + dz).r - texture3D(uVolume, tc - dz).r                                               \n"
"       );                                                                                                              \n"
"                                                                                                                       \n"
"       return (nabla / uGradientDelta) * uTextureScale;                                                                \n"
"   }                                                                                                                   \n"
"                                                                                                                       \n"
"                                                                                                                       \n"
"   //----------------------------------------------------------------------                                            \n"
"   //  Computes phong shading based on current light and material                                                      \n"
"   //  properties.                                                                                                     \n"
"   //----------------------------------------------------------------------                                            \n"
"                                                                                                                       \n"
"   vec3 shade(vec3 p, vec3 v, vec3 n)                                                                                  \n"
"   {                                                                                                                   \n"
"       vec4 lp = gl_ModelViewMatrixInverse * gl_LightSource[0].position;                                               \n"
"       vec3 l = normalize(lp.xyz - p * lp.w);                                                                          \n"
"       vec3 h = normalize(l+v);                                                                                        \n"
"       float cos_i = max(dot(n, l), 0.0);                                                                              \n"
"       float cos_h = max(dot(n, h), 0.0);                                                                              \n"
"                                                                                                                       \n"
"       vec3 Ia = gl_FrontLightProduct[0].ambient.rgb;                                                                  \n"
"       vec3 Id = gl_FrontLightProduct[0].diffuse.rgb * cos_i;                                                          \n"
"       vec3 Is = gl_FrontLightProduct[0].specular.rgb * pow(cos_h, gl_FrontMaterial.shininess);                        \n"
"       return (Ia + Id + Is);                                                                                          \n"
"   }                                                                                                                   \n"
"                                                                                                                       \n"
"                                                                                                                       \n"
"   //----------------------------------------------------------------------                                            \n"
"   //  Main fragment shader code.                                                                                      \n"
"   //----------------------------------------------------------------------                                            \n"
"                                                                                                                       \n"
"   void main(void)                                                                                                     \n"
"   {                                                                                                                   \n"
"       vec4 camera = gl_ModelViewMatrixInverse * vec4(0.0, 0.0, 0.0, 1.0);                                             \n"
"       vec3 raydir = normalize(vPosition.xyz - camera.xyz);                                                            \n"
"                                                                                                                       \n"
"       float t_entry = entry(vPosition.xyz, raydir);                                                                   \n"
"       t_entry = max(t_entry, -distance(camera.xyz, vPosition.xyz));                                                   \n"
"                                                                                                                       \n"
"       // estimate a reasonable step size                                                                              \n"
"       float t_step = distance(uMinCorner, uMaxCorner) / uResolution;                                                  \n"
"       vec3 tc_step = uTextureScale * (t_step * raydir);                                                               \n"
"                                                                                                                       \n"
"       // cast the ray (in model space)                                                                                \n"
"       vec4 sum = vec4(0.0);                                                                                           \n"
"       vec3 tc = gl_TexCoord[0].stp + t_entry * tc_step / t_step;                                                      \n"
"                                                                                                                       \n"
"       for (float t = t_entry; t < 0.0; t += t_step, tc += tc_step)                                                    \n"
"       {                                                                                                               \n"
"           // sample the volume for intensity (red channel)                                                            \n"
"           float intensity = texture3D(uVolume, tc).r;                                                                 \n"
"                                                                                                                       \n"
"           // look up intensity in the colour LUT                                                                      \n"
"           vec4 colour = texture1D(uColorLUT, intensity);                                                              \n"
"                                                                                                                       \n"
"           // skip empty space                                                                                         \n"
"           if (colour.a < 0.001) continue;                                                                             \n"
"                                                                                                                       \n"
"           // estimate gradient                                                                                        \n"
"           vec3 nabla = gradient(tc);                                                                                  \n"
"                                                                                                                       \n"
"           // compute shading                                                                                          \n"
"           vec3 position = vPosition.xyz + t * raydir;                                                                 \n"
"           vec3 normal = -normalize(nabla);                                                                            \n"
"           vec3 view = -raydir;                                                                                        \n"
"           vec3 shaded = shade(position, view, normal);                                                                \n"
"           colour.rgb *= shaded;                                                                                       \n"
"                                                                                                                       \n"
"           // compute transmission for this segment                                                                    \n"
"           float Tr = exp(-colour.a * uOpticalDensityFactor);                                                          \n"
"           colour.rgb *= 1.0 - Tr;                                                                                     \n"
"           colour.a = 1.0 - Tr;                                                                                        \n"
"                                                                                                                       \n"
"           // accumulate colour and opacity                                                                            \n"
"           sum += (1.0 - sum.a) * colour;                                                                              \n"
"                                                                                                                       \n"
"           // early ray termination test                                                                               \n"
"           if (sum.a > uOpacityThreshold)                                                                              \n"
"           {                                                                                                           \n"
"               // calculate fragment depth                                                                             \n"
"               vec4 clip = gl_ModelViewProjectionMatrix * vec4(position, 1.0);                                         \n"
"               gl_FragDepth = (gl_DepthRange.diff * clip.z / clip.w +                                                  \n"
"                            gl_DepthRange.near + gl_DepthRange.far) * 0.5;                                             \n"
"                                                                                                                       \n"
"               break;                                                                                                  \n"
"           }                                                                                                           \n"
"       }                                                                                                               \n"
"                                                                                                                       \n"
"       gl_FragColor = sum;                                                                                             \n"
"   }                                                                                                                   \n"
"                                                                                                                       \n";

//---------------------------------------------------------------------------
} // namespace chai3d
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
