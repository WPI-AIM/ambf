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
   vTexCoord = aTexCoord;
   gl_Position = vec4(aPosition.x, aPosition.y, 0.0, 1.0);
}
