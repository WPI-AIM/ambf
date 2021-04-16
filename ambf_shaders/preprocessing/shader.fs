varying vec4 vPosition;
varying vec3 vNormal;
varying vec3 vTexCoord;

uniform sampler2DShadow shadowMap;
uniform sampler2D diffuseMap;

void main(void)
{
    vec3 diffuse =  gl_FrontLightProduct[0].ambient.rgb;
    gl_FragColor = vec4(diffuse, 1.0);
}
