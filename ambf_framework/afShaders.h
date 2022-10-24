//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2019-2021, AMBF
    (https://github.com/WPI-AIM/ambf)

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

    * Neither the name of authors nor the names of its contributors may
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

    \author    <amunawar@wpi.edu>
    \author    Adnan Munawar
*/
//==============================================================================

#include <string>

using namespace std;

// DEPTH POINT CLOUD COMPUTER SHADERS
static string AF_DEPTH_COMPUTE_VTX =
        " #version 130                                                                          \n"
        " attribute vec3 aPosition;                                                             \n"
        " attribute vec3 aNormal;                                                               \n"
        " attribute vec3 aTexCoord;                                                             \n"
        " attribute vec4 aColor;                                                                \n"
        " attribute vec3 aTangent;                                                              \n"
        " attribute vec3 aBitangent;                                                            \n"
        "                                                                                       \n"
        " varying vec4 vPosition;                                                               \n"
        " varying vec3 vNormal;                                                                 \n"
        " varying vec3 vTexCoord;                                                               \n"
        "                                                                                       \n"
        " void main(void)                                                                       \n"
        " {                                                                                     \n"
        "    vTexCoord = aTexCoord;                                                             \n"
        "    gl_Position = vec4(aPosition, 1.0);                                                \n"
        " }                                                                                     \n";

static string AF_DEPTH_COMPUTE_FRAG =
        " #version 130                                                                          \n"
        " uniform sampler2D diffuseMap;                                                         \n"
        " uniform mat4 invProjection;                                                           \n"
        "                                                                                       \n"
        " varying vec3 vTexCoord;                                                               \n"
        "                                                                                       \n"
        " void main(void)                                                                       \n"
        " {                                                                                     \n"
        "     float z_normalized = texture2D(diffuseMap, vTexCoord.xy).r;                       \n"
        "     float x = vTexCoord.x * 2.0 - 1.0;                                                \n"
        "     float y = vTexCoord.y * 2.0 - 1.0;                                                \n"
        "     float z = z_normalized * 2.0 - 1.0;                                               \n"
        "                                                                                       \n"
        "     vec4 P = vec4(x, y, z, 1.0);                                                      \n"
        "     P = invProjection * P;                                                            \n"
        "     P /= P.w;                                                                         \n"
        "     gl_FragColor = vec4(P.z, P.x, P.y, 1.0);                                          \n"
        " }                                                                                     \n";

// SKYBOX SHADERS
static string AF_SKYBOX_VTX =
        " attribute vec3 aPosition;                                                           \n"
        " attribute vec3 aNormal;                                                             \n"
        " attribute vec3 aTexCoord;                                                           \n"
        " attribute vec4 aColor;                                                              \n"
        " attribute vec3 aTangent;                                                            \n"
        " attribute vec3 aBitangent;                                                          \n"
        "                                                                                     \n"
        " varying vec3 TexCoords;                                                             \n"
        "                                                                                     \n"
        " uniform mat4 viewMat;                                                               \n"
        "                                                                                     \n"
        " uniform mat4 modelMat;                                                              \n"
        "                                                                                     \n"
        " uniform mat4 modelViewMat;                                                          \n"
        "                                                                                     \n"
        " void main()                                                                         \n"
        " {                                                                                   \n"
        "     TexCoords = aPosition;                                                          \n"
        "     // vec4 pos = gl_ProjectionMatrix * gl_ModelViewMatrix * vec4(aPosition, 1.0);  \n"
        "     vec4 pos = gl_ProjectionMatrix * viewMat * vec4(aPosition, 1.0);                \n"
        "     gl_Position = pos.xyww;                                                         \n"
        " }                                                                                   \n";

static string AF_SKYBOX_FRAG =
        " varying vec3 TexCoords;                                                             \n"
        "                                                                                     \n"
        " uniform samplerCube skybox;                                                         \n"
        "                                                                                     \n"
        " void main()                                                                         \n"
        " {                                                                                   \n"
        "     gl_FragColor = textureCube(skybox, TexCoords);                                  \n"
        "     // gl_FragColor = vec4(1.0, 0.0, 0.0, 1.0);                                     \n"
        " }                                                                                   \n";

