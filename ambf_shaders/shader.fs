varying vec4 vPosition;
varying vec3 vNormal;
varying vec3 vTexCoord;

uniform sampler2DShadow shadowMap;
uniform sampler2D diffuseMap;
uniform sampler2D normalMap;

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

vec4 shade(vec3 p, vec3 v, vec3 n)
{
     vec3 Ie = gl_FrontMaterial.emission.rgb;
     vec3 Ia = gl_FrontLightModelProduct.sceneColor.rgb;
     vec3 Il = vec3(0.0);

     for (int i = 0; i < gl_MaxLights; ++i)
     {
         vec4 p_l = gl_LightSource[i].position;
         vec3 l = normalize(p_l.xyz - p * p_l.w);
         vec3 h = normalize(l + v);
         vec3 r = reflect(-l, n);

         float s_m = gl_FrontMaterial.shininess;
         float cosNL = max(dot(n, l), 0.0);
         float cosNH = max(dot(v, r), 0.0);

         float att = attenuation(p, i);
         float intensity = 0.3*spotlight(p, i);

         vec3 texColor = texture(diffuseMap, vTexCoord).xyz;

         vec3 Iambient = gl_FrontLightProduct[i].ambient.rgb;

         vec3 Idiffuse = cosNL * gl_FrontLightProduct[i].diffuse.rgb;

         vec3 Ispecular = pow(cosNH, s_m) * gl_FrontLightProduct[i].specular.rgb;

         Iambient *= texColor * att * intensity;

         Idiffuse *= texColor * att * intensity;

         Ispecular *= texColor * att;

         vec3 phong = Iambient + Idiffuse + Ispecular;

         Il += phong;
         Il = clamp(Il, 0.0, 1.0);
     }
     float alpha = gl_FrontMaterial.diffuse.a;
     return vec4(Ie + Ia + Il, alpha);
}

void main(void)
{
    vec3 view = normalize(-vPosition.xyz);
    vec3 normal = normalize(vNormal);
    vec4 shaded =  shade(vPosition.xyz, view, normal);
    //shaded.rgb += texture(diffuseMap, vTexCoord).rgb;
    vec4 shadow = shadow2DProj(shadowMap, gl_TexCoord[1]);
    gl_FragColor = vec4(shaded.rgb, shadow.a);
    //gl_FragColor = shade(vPosition.xyz, view, normal);
}
