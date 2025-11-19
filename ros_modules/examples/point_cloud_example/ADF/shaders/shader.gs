#version 150 compatibility

layout (points) in;
layout (triangle_strip, max_vertices = 4) out;

in vec4 vPosition[];

out vec3 vQuadCoord;  // Local quad coordinates for sphere impostor
out vec4 vCenterPos;  // Center of the quad (sphere center) in eye space
uniform float uPointSize; // Radius of the sphere
void main() {
    vec4 center = vPosition[0];  // Sphere center in eye space
    
    // Create a camera-facing quad around the point
    // Quad extends from -uRadius to +uRadius in X and Y (eye space)
    // Vertices: bottom-left, bottom-right, top-left, top-right
    
    vec2 offsets[4] = vec2[](
        vec2(-1.0, -1.0),  // bottom-left
        vec2( 1.0, -1.0),  // bottom-right
        vec2(-1.0,  1.0),  // top-left
        vec2( 1.0,  1.0)   // top-right
    );
    
    for (int i = 0; i < 4; ++i) {
        // Offset from center in screen/eye space (Z=0 plane)
        vec4 quadPos = center + vec4(offsets[i] * uPointSize, 0.0, 0.0);
        gl_Position = gl_ProjectionMatrix * quadPos;
        vQuadCoord = vec3(offsets[i], 0.0);
        vCenterPos = center;
        EmitVertex();
    }
    
    EndPrimitive();
} 