#version 330
uniform sampler2D diffuseMap;
uniform mat4 invProjection;

in vec3 vTexCoord;

out vec4 fragColor;

void main(void)
{
    float z_normalized = texture2D(diffuseMap, vTexCoord.xy).r;
    float x = vTexCoord.x * 2.0 - 1.0;
    float y = vTexCoord.y * 2.0 - 1.0;
    float z = z_normalized * 2.0 - 1.0;

    vec4 P = vec4(x, y, z, 1.0);
    P = invProjection * P;
    P /= P.w;
    // Change from OGL to AMBF coordinates
    fragColor = vec4(P.z, P.x, P.y, 1.0);
}
