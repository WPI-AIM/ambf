#version 130
uniform sampler2D diffuseMap;
uniform vec3 maxWorldDimensions;
uniform float nearPlane;
uniform float farPlane;
uniform mat4 invProjection;

varying vec3 vTexCoord;

void main(void)
{
    vec4 texColor = texture2D(diffuseMap, vTexCoord.xy);
    float x = vTexCoord.x * 2.0 - 1.0;
    float y = vTexCoord.y * 2.0 - 1.0;
    int b0 = int(texColor.x * 255.0);
    int b1 = int(texColor.y * 255.0);
    int b2 = int(texColor.z * 255.0);
    int b3 = int(texColor.w * 255.0);

    int depth = int(b3 << 8 | b2);
    float d = float(depth) / float(pow(2.0, 2*8));
    //
    // uint depth = uint(b3 << 8 | b2);
    // float d = float(depth) / float(pow(2.0, 2*8));

    float z = d * 2.0 - 1.0;
    vec4 P = vec4(x, y, z, 1.0);

    P = invProjection * P;
    P /= P.w;

    float deltaZ = farPlane - nearPlane;
    float normalized_z = (P.z - nearPlane)/deltaZ;

    // Assuming the frustrum is centered vertically and horizontally
    float normalized_x = (P.x + maxWorldDimensions.x / 2.0)/maxWorldDimensions.x;
    float normalized_y = (P.y + maxWorldDimensions.y / 2.0)/maxWorldDimensions.y;

    gl_FragColor = vec4(normalized_x, normalized_y, normalized_z, 1.0);
}
