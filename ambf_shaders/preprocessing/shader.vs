
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
   vPosition = gl_ModelViewMatrix * gl_Vertex;
   vNormal = gl_NormalMatrix * aNormal;
   gl_TexCoord[0] = gl_TextureMatrix[0] * vec4(aTexCoord, 1.0);

   float s = dot(gl_EyePlaneS[1], vPosition);
   float t = dot(gl_EyePlaneT[1], vPosition);
   float r = dot(gl_EyePlaneR[1], vPosition);
   float q = dot(gl_EyePlaneQ[1], vPosition);
   gl_TexCoord[1] = vec4(s, t, r, q);
   vTexCoord = aTexCoord;

   gl_Position = ftransform();
}
