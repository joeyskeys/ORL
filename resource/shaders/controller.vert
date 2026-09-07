#version 450

layout(binding = 0) uniform CameraUBO {
    mat4 view;
    mat4 proj;
} ubo;

layout(binding = 1) uniform ModelUBO {
    mat4 model;
} model_buf;

layout(binding = 2) uniform ColorUBO {
    vec4 value;
} color_buf;

layout(location = 0) in vec3 inPosition;
layout(location = 0) flat out vec4 fragColor;

void main() {
    fragColor = color_buf.value;
    gl_Position = ubo.proj * ubo.view * model_buf.model * vec4(inPosition, 1.0);
}
