varying vec3 TexCoords;

uniform samplerCube skybox;

void main()
{
    gl_FragColor = texture(skybox, TexCoords);
}
