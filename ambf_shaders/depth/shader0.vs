#version 330
layout (location = 0) in vec3 aPosition;
layout (location = 1) in vec3 aTexCoord;
layout (location = 2) in vec4 aColor;
layout (location = 3) in vec3 aTangent;
layout (location = 4) in vec3 aBitangent;

out vec4 vPosition;
out vec3 vNormal;
out vec3 vTexCoord;

void main(void)
{
   vTexCoord = aTexCoord;

   gl_Position = vec4(aPosition.x, aPosition.y, 0.0, 1.0);
}
