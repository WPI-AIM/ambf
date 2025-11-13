#version 150

layout (points) in;
layout (triangle_strip, max_vertices = 3) out;

void main() {
    float off = 0.01;
    gl_Position = gl_in[0].gl_Position + vec4(-off, -off, 0.0, 0.0); 
    EmitVertex();

    gl_Position = gl_in[0].gl_Position + vec4( off, -off, 0.0, 0.0);
    EmitVertex();

    gl_Position = gl_in[0].gl_Position + vec4( 0.0, off, 0.0, 0.0);
    EmitVertex();
    
    EndPrimitive();
} 