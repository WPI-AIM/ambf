uniform sampler2D diffuseMap;
varying vec3 vTexCoord;
uniform vec3 maxWorldDimensions;
uniform float nearPlane;
uniform float farPlane;
void main(void)
{
    vec3 texColor = texture(diffuseMap, vTexCoord.xy).xyz;
    vec4 P = vec4(2.0 * vTexCoord.xy - 1.0, 2.0 * texColor.r - 1.0, 1.0);

    P = inverse(gl_ProjectionMatrix) * P;
    P /= P.w;
    // vec3 halfDimensions = maxWorldDimensions / 2;
    // P.xyz /= maxWorldDimensions;

    float midPoint = (farPlane + nearPlane) / 2.0;
    float deltaZ = farPlane - nearPlane;
    float d = 0.5 + ((P.z - midPoint) / deltaZ);
    // P.z /= maxWorldDimensions.z;
    if (P.z < farPlane ){
      gl_FragColor = vec4(farPlane / P.z, 0.0, 0.0, 1.0);
    }
    else if (P.z > nearPlane){
      gl_FragColor = vec4(0.0, 0.0, nearPlane / P.z, 1.0);
    }
    else{
      gl_FragColor = vec4(d, d, d, 1.0);
    }
    //gl_FragColor = shade(vPosition.xyz, view, normal);
}
