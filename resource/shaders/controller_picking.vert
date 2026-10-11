#version 450

layout(binding = 0) uniform CameraUBO {
    mat4 view;
    mat4 proj;
} camera;

layout(binding = 1) uniform ModelUBO {
    mat4 model;
} model_buf;

layout(location = 0) in vec3 inPosition;
layout(location = 0) flat out uint element_id;

void main() {
    element_id = 0u;
    gl_Position = camera.proj * camera.view * model_buf.model
        * vec4(inPosition, 1.0);
}
