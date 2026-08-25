#version 450

layout(location = 0) in vec3 inPosition;

void main() {
    // Cover clip space with a single triangle. Dummy mesh vertices are ignored.
    vec2 pos = vec2((gl_VertexIndex << 1) & 2, gl_VertexIndex & 2);
    gl_Position = vec4(pos * 2.0 - 1.0, 1.0, 1.0) + vec4(inPosition * 0.0, 0.0);
}
