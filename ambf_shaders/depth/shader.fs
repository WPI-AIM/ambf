uniform sampler2D diffuseMap;
varying vec3 vTexCoord;
uniform vec3 maxWorldDimensions;
uniform float nearPlane;
uniform float farPlane;

uniform mat4 invProjection;

void main(void)
{
    vec4 texColor = texture2D(diffuseMap, vTexCoord.xy);
    float x = vTexCoord.x * 2.0 - 1.0;
    float y = vTexCoord.y * 2.0 - 1.0;
    uint b0 = texColor.x * 255.0;
    uint b1 = texColor.y * 255.0;
    uint b2 = texColor.z * 255.0;
    uint b3 = texColor.w * 255.0;

    uint depth = uint(b3 << 24 | b2 << 16 | b1 << 8 | b0 );
    depth = uint(b3 << 24 | b2 << 16 | b1 << 8 | b0 );
    float d = float(depth) / float(pow(2.0, 4*8));

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
