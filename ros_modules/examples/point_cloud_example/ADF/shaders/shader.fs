#version 150 compatibility

in vec3 vQuadCoord;    // Local quad coordinates (-1 to 1) for sphere impostor
in vec4 vCenterPos;    // Center of sphere in eye space

// Material properties (from gl_FrontMaterial)
uniform vec4 uMaterialDiffuse;
uniform vec4 uMaterialAmbient;
uniform vec4 uMaterialSpecular;
uniform int uMaterialShininess;
uniform float uPointSize;

uniform sampler2DShadow shadowMap;

float attenuation(vec3 p, int i)
{
     vec4 p_l = gl_LightSource[i].position;
     if (p_l.w == 0.0) return 1.0;
     float d = distance(p, p_l.xyz);
     float k0 = gl_LightSource[i].constantAttenuation;
     float k1 = gl_LightSource[i].linearAttenuation;
     float k2 = gl_LightSource[i].quadraticAttenuation;
     return 1.0 / (k0 + k1*d + k2*d*d);
}

float spotlight(vec3 p, int i)
{
    if (gl_LightSource[i].spotCosCutoff < 0.0) return 1.0;
    vec4 p_l = gl_LightSource[i].position;
    if (p_l.w == 0.0) return 1.0;
    vec3 v = normalize(p - p_l.xyz);
    vec3 s = normalize(gl_LightSource[i].spotDirection);
    float cosine = max(dot(v, s), 0.0);
    float cutOffOuter = gl_LightSource[i].spotCosCutoff;
    float epsilon = gl_LightSource[i].spotCosCutoff - cutOffOuter;
    float intensity = clamp((cosine - cutOffOuter) / epsilon, 0.0, 1.0);

    if (cosine >= gl_LightSource[i].spotCosCutoff){
      return pow(cosine, gl_LightSource[i].spotExponent);
    }
    else{
      return 0.0;
    }
    return intensity;
}

vec3 shade(vec3 p, vec3 v, vec3 n)
{
    vec3 Il = vec3(0.0, 0.0, 0.0);
    for (int i = 0; i < gl_MaxLights; ++i)
    {
        vec4 p_l = gl_LightSource[i].position;
        vec3 l = normalize(p_l.xyz - p * p_l.w);
        vec3 h = normalize(l + v);
        vec3 r = reflect(-l, n);

        float cosNL = max(dot(n, l), 0.0);
        float cosNH = max(dot(v, r), 0.0);

        float att = attenuation(p, i);
        float spot_intensity = spotlight(p, i);

        vec4 lightAmbient = gl_LightSource[i].ambient;
        vec4 lightDiffuse = gl_LightSource[i].diffuse;
        vec4 lightSpecular = gl_LightSource[i].specular;

        vec3 Iambient = lightAmbient.rgb * uMaterialAmbient.rgb;
        vec3 Idiffuse = cosNL * lightDiffuse.rgb * uMaterialDiffuse.rgb * att * spot_intensity;
        vec3 Ispecular = pow(cosNH, uMaterialShininess) * lightSpecular.rgb * uMaterialSpecular.rgb * att;

        vec4 fragPos = vec4(p, 1.0);

        float es = dot(gl_EyePlaneS[1], fragPos);
        float et = dot(gl_EyePlaneT[1], fragPos);
        float er = dot(gl_EyePlaneR[1], fragPos);
        float eq = dot(gl_EyePlaneQ[1], fragPos);
        vec4 shadowCoord = vec4(es, et, er, eq);

        vec4 shadow = shadow2DProj(shadowMap, shadowCoord);

        float shadeFactor = mix(0.3, 1.0, shadow.a);
        Il += (Iambient + (Idiffuse + Ispecular) * shadeFactor);
    }
    Il = clamp(Il, 0.0, 1.0);
    return Il;
}

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

    vec3 viewDir = normalize(-vCenterPos.xyz);

    vec3 fragPos = vec3(vCenterPos.xyz + normal * uPointSize);

    vec3 finalColor = shade(fragPos, viewDir, normal);
    gl_FragColor = vec4(finalColor, uMaterialDiffuse.a);
}
