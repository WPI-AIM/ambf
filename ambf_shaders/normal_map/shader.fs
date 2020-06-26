varying vec3 FragPos;
varying vec2 TexCoords;
varying vec3 TangentLightPos;
varying vec3 TangentViewPos;
varying vec3 TangentFragPos;

uniform sampler2D diffuseMap;
//uniform sampler2DShadow shadowMap;
uniform sampler2D normalMap;

varying vec3 vNormal;

uniform int vEnableNormalMapping;

void main()
{
    vec3 normal;
    if (vEnableNormalMapping){
      // obtain normal from normal map in range [0,1]
      normal = texture(normalMap, TexCoords).rgb;
      // transform normal vector to range [-1,1]
      normal = normalize(normal * 2.0 - 1.0);  // this normal is in tangent space
    }
    else{
      normal = normalize(vNormal.xyz);
    }


    normal.xy *= 1.0;
    normalize(normal);

    // normal = normalize(vNormal);

    // get diffuse color
    vec3 color = texture(diffuseMap, TexCoords).rgb;
    // ambient
    vec3 ambient = 0.5 * color;
    // diffuse
    vec3 lightDir = normalize(TangentLightPos - TangentFragPos);
    float diff = max(dot(lightDir, normal), 0.0);
    vec3 diffuse = diff * color;
    // specular
    vec3 viewDir = normalize(TangentViewPos - TangentFragPos);
    vec3 reflectDir = reflect(-lightDir, normal);
    vec3 halfwayDir = normalize(lightDir + viewDir);
    float spec = pow(max(dot(normal, halfwayDir), 0.0), 32.0);

    vec3 specular = vec3(0.2) * spec;
    gl_FragColor = vec4(ambient + diffuse + specular, 1.0);
}
