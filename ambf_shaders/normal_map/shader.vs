
attribute vec3 aPosition;
attribute vec3 aNormal;
attribute vec3 aTexCoord;
attribute vec4 aColor;
attribute vec3 aTangent;
attribute vec3 aBitangent;

varying vec3 FragPos;
varying vec2 TexCoords;
varying vec3 TangentLightPos;
varying vec3 TangentViewPos;
varying vec3 TangentFragPos;

varying vec3 vNormal;

varying vec3 viewPos;

void main()
{
    viewPos = -normalize(aPosition.xyz);
    FragPos = vec3(gl_ModelViewMatrix * vec4(aPosition, 1.0));
    TexCoords = aTexCoord;

    mat3 normalMatrix = transpose(inverse(mat3(gl_ModelViewMatrix)));
    vec3 T = normalize(normalMatrix * aTangent);
    vec3 N = normalize(normalMatrix * aNormal);
    T = normalize(T - dot(T, N) * N);
    vec3 B = cross(N, T);

    mat3 TBN = transpose(mat3(T, B, N));
    TangentLightPos = TBN * gl_LightSource[0].position.xyz;
    TangentViewPos  = TBN * viewPos;
    TangentFragPos  = TBN * FragPos;

    vNormal = TBN * gl_NormalMatrix * aNormal;

    gl_Position = ftransform();
}
