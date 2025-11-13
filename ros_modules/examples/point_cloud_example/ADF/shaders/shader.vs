
attribute vec3 aPosition;
attribute vec3 aNormal;
attribute vec3 aTexCoord;
attribute vec4 aColor;
attribute vec3 aTangent;
attribute vec3 aBitangent;

varying vec4 vPosition;
varying vec3 vNormal;
varying vec3 vTexCoord;

void main(void)
{
   // pass along a transformed vertex position, normal, and texture
   vPosition = gl_ModelViewProjectionMatrix * vec4(aPosition, 1.0);
   vNormal = gl_NormalMatrix * aNormal;
   gl_Position = vPosition;
   gl_PointSize = 2.0 / vPosition.w;
}
