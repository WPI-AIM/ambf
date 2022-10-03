#version 130
uniform sampler2D diffuseMap;
uniform mat4 invProjection;

varying vec3 vTexCoord;

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
    gl_FragColor = vec4(P.z, P.x, P.y, 1.0);
}
