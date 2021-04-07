#version 330
layout(location = 1) in vec3 aPosition;
layout(location = 2) in vec3 aNormal;
layout(location = 3) in vec3 aTexCoord;
layout(location = 4) in vec4 aColor;
layout(location = 5) in vec3 aTangent;
layout(location = 6) in vec3 aBitangent;

uniform mat4 projectionMat;
uniform mat4 viewMat;

out vec3 TexCoords;

void main()
{
    TexCoords = aPosition;
    vec4 pos = projectionMat * viewMat * vec4(aPosition, 1.0);
    gl_Position = pos.xyww;
}
