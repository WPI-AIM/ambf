
attribute vec3 aPosition;
attribute vec3 aNormal;
attribute vec3 aTexCoord;
attribute vec4 aColor;
attribute vec3 aTangent;
attribute vec3 aBitangent;

varying vec3 TexCoords;

uniform mat4 viewMat;

uniform mat4 modelMat;

uniform mat4 modelViewMat;

void main()
{
    TexCoords = aPosition;
    // vec4 pos = gl_ProjectionMatrix * gl_ModelViewMatrix * vec4(aPosition, 1.0);
    vec4 pos = gl_ProjectionMatrix * viewMat * vec4(aPosition, 1.0);
    gl_Position = pos.xyww;
}
