attribute vec3 aPosition;
attribute vec3 aNormal;
attribute vec3 aTexCoord;
attribute vec4 aColor;
attribute vec3 aTangent;
attribute vec3 aBitangent;

varying vec3 TexCoords;

void main()
{
    TexCoords = aPosition;
    vec4 pos = gl_ProjectionMatrix * vec4(aPosition, 1.0);
    gl_Position = pos;
}
