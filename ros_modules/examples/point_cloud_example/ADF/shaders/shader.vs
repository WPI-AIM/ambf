#version 150 compatibility

in vec3 aPosition;
in vec3 aNormal;
in vec3 aTexCoord;
in vec4 aColor;
in vec3 aTangent;
in vec3 aBitangent;

out vec4 vPosition;

void main(void)
{
   // pass along a transformed vertex position, normal, and texture
   vPosition = gl_ModelViewMatrix * vec4(aPosition, 1.0);
   gl_Position = gl_ModelViewProjectionMatrix * vec4(aPosition, 1.0);
}
