uniform sampler2D diffuseMap;
varying vec3 vTexCoord;
uniform vec3 maxWorldDimensions;
uniform float nearPlane;
uniform float farPlane;

varying mat4 invProjection;
uniform mat4 Projection;
// Linearizes a Z buffer value
float CalcLinearZ(float depth) {
    float zFar = farPlane;
    float zNear = nearPlane;

    // bias it from [0, 1] to [-1, 1]
    float linear = zNear / (zFar - depth * (zFar - zNear)) * zFar;
    return linear;
}

void main(void)
{
    vec3 texColor = texture2D(diffuseMap, vTexCoord.xy).xyz;
    float x = vTexCoord.x * 2.0 - 1.0;
    float y = vTexCoord.y * 2.0 - 1.0;
    // float z = CalcLinearZ(texColor.r) * 2.0 - 1.0;
    float z = texColor.r * 2.0 - 1.0;
    vec4 P = vec4(x, y, z, 1.0);

    // P = invProjection * P;
    P = inverse(gl_ProjectionMatrix) * P;
    P /= P.w;
    // vec3 halfDimensions = maxWorldDimensions / 2;
    // P.xyz /= maxWorldDimensions;

    float midPoint = (farPlane + nearPlane) / 2.0;
    float deltaZ = farPlane - nearPlane;
    float normalized_z = (P.z - nearPlane)/deltaZ;

    // Assuming the frustrum is centered vertically and horizontally
    float normalized_x = (P.x + maxWorldDimensions.x / 2.0)/maxWorldDimensions.x;
    float normalized_y = (P.y + maxWorldDimensions.y / 2.0)/maxWorldDimensions.y;
    // P.z /= maxWorldDimensions.z;
    // if (P.z >= 0.0 ){
    //   gl_FragColor = vec4(0.0, 1.0, 0.0, 1.0);
    // }
    // else if (P.z < farPlane ){
    //   gl_FragColor = vec4(farPlane / P.z, 0.0, 0.0, 1.0);
    // }
    // else if (P.z > nearPlane){
    //   gl_FragColor = vec4(0.0, 0.0, P.z / nearPlane, 1.0);
    // }
    // else{
      gl_FragColor = vec4(normalized_x, normalized_y, normalized_z, 1.0);
    // }
    //gl_FragColor = shade(vPosition.xyz, view, normal);
}
