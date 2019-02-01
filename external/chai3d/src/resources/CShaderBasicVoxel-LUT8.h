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
#ifndef CShaderBasicVoxelLUT8
#define CShaderBasicVoxelLUT8
//---------------------------------------------------------------------------
#include "system/CGlobals.h"
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
namespace chai3d {
//---------------------------------------------------------------------------

const std::string C_SHADER_BASIC_VOXEL_LUT8_VERT = 
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

const std::string C_SHADER_BASIC_VOXEL_LUT8_FRAG = 
"                                                                                                                       \n"
"   uniform vec3 uMinCorner;                                                                                            \n"
"   uniform vec3 uMaxCorner;                                                                                            \n"
"   uniform vec3 uTextureScale;                                                                                         \n"
"   uniform sampler3D uVolume;                                                                                          \n"
"   uniform sampler1D uColorLUT;                                                                                        \n"
"   uniform float uResolution;                                                                                          \n"
"   uniform float uOpacity;                                                                                             \n"
"                                                                                                                       \n"
"   varying vec4 vPosition;                                                                                             \n"
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
"   //  Performs interval bisection and returns the value between a and b                                               \n"
"   //  closest to isosurface. When s(b) > s(a), direction should be +1.0,                                              \n"
"   //  and -1.0 otherwise.                                                                                             \n"
"   //----------------------------------------------------------------------                                            \n"
"                                                                                                                       \n"
"   vec3 refine(vec3 a, vec3 b, float direction)                                                                        \n"
"   {                                                                                                                   \n"
"       for (int i = 0; i < 6; ++i)                                                                                     \n"
"       {                                                                                                               \n"
"           vec3 m = 0.5 * (a + b);                                                                                     \n"
"           float v = texture3D(uVolume, m).a * direction;                                                              \n"
"           if (v >= 0.0)   b = m;                                                                                      \n"
"           else            a = m;                                                                                      \n"
"       }                                                                                                               \n"
"       return b;                                                                                                       \n"
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
"       float sigma = 100.0 * uOpacity;                                                                                 \n"
"       if (uOpacity == 1.0)                                                                                            \n"
"       {                                                                                                               \n"
"           sigma = 1000.0;                                                                                             \n"
"       }                                                                                                               \n"
"                                                                                                                       \n"
"       // cast the ray (in model space)                                                                                \n"
"       vec4 sum = vec4(0.0);                                                                                           \n"
"       vec3 tc = gl_TexCoord[0].stp + t_entry * tc_step / t_step;                                                      \n"
"       bool flag = true;                                                                                               \n"
"       for (float t = t_entry; t < 0.0; t += t_step, tc += tc_step)                                                    \n"
"       {                                                                                                               \n"
"           vec4 voxel = texture3D(uVolume, tc);                                                                        \n"
"           vec4 colour = texture1D(uColorLUT, voxel.r);                                                                \n"
"                                                                                                                       \n"
"           float Tr = exp(-sigma * t_step);                                                                            \n"
"           colour *= 1.0 - Tr;                                                                                         \n"
"                                                                                                                       \n"
"           if (flag)                                                                                                   \n"
"           {                                                                                                           \n"
"               if (voxel.r > 0.0)                                                                                      \n"
"               {                                                                                                       \n"
"                   vec3 tcr = refine(tc - tc_step, tc, 1.0);                                                           \n"
"                                                                                                                       \n"
"                   float dt = length(tcr - tc) / length(tc_step);                                                      \n"
"                   vec3 position = vPosition.xyz + (t - dt * t_step) * raydir;                                         \n"
"                   vec4 clip = gl_ModelViewProjectionMatrix * vec4(position, 1.0);                                     \n"
"                   gl_FragDepth=(gl_DepthRange.diff * clip.z / clip.w + gl_DepthRange.near + gl_DepthRange.far) * 0.5; \n"
"                   flag = false;                                                                                       \n"
"               }                                                                                                       \n"
"           }                                                                                                           \n"
"           if (voxel.r > 0.0)                                                                                          \n"
"           {                                                                                                           \n"
"               sum += (1.0 - sum.a) * colour;                                                                          \n"
"           }                                                                                                           \n"
"       }                                                                                                               \n"
"                                                                                                                       \n"
"       // discard the fragment if no geometry was intersected                                                          \n"
"       if (sum.a <= 0.0) discard;                                                                                      \n"
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
