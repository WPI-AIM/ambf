#version 150 compatibility

in vec3 vQuadCoord;    // Local quad coordinates (-1 to 1) for sphere impostor
in vec4 vCenterPos;    // Center of sphere in eye space

// Material properties (from gl_FrontMaterial)
uniform vec4 uMaterialDiffuse;
uniform vec4 uMaterialAmbient;
uniform vec4 uMaterialSpecular;
uniform int uMaterialShininess;

void main(void)
{
    
    // Distance from the center of the quad
    float dist_sq = vQuadCoord.x * vQuadCoord.x + vQuadCoord.y * vQuadCoord.y;
    
    // Discard fragments outside the sphere
    if (dist_sq > 1.0) discard;
    
    // Compute the Z offset for the sphere surface (solve x^2 + y^2 + z^2 = r^2)
    // where x, y are normalized coordinates (-1 to 1)
    float z_offset = sqrt(1.0 - dist_sq);
    
    // Reconstruct the sphere surface normal in eye space
    vec3 normal = normalize(vec3(vQuadCoord.xy, z_offset));
    
    // Simple Phong shading with a fixed light direction
    // Light direction (assumed to be pointing towards +Z in eye space for simplicity)
    vec3 lightDir = normalize(vec3(0.3, 0.5, 1.0));
    vec3 viewDir = normalize(-vCenterPos.xyz);
    vec3 lightColor = vec3(1.0);  // White light
    
    // Ambient: material ambient * material diffuse
    vec3 ambient = uMaterialAmbient.rgb * uMaterialDiffuse.rgb;
    
    // Diffuse: light color * material diffuse * cosine
    float diff = max(dot(normal, lightDir), 0.0);
    vec3 diffuse = lightColor * uMaterialDiffuse.rgb * diff;
    
    // Specular: light color * material specular * pow(cosine, shininess)
    vec3 reflectDir = reflect(-lightDir, normal);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), uMaterialShininess);
    vec3 specular = lightColor * uMaterialSpecular.rgb * spec;
    
    // Combine
    vec3 color = ambient + diffuse + specular;
    gl_FragColor = vec4(color, uMaterialDiffuse.a);
}
