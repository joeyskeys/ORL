#version 450

layout(binding = 0) uniform CameraUBO {
    mat4 view;
    mat4 proj;
} camera;

struct LocatorPickData {
    mat4 model;
    uint token;
    uint pad0;
    uint pad1;
    uint pad2;
};

layout(std430, binding = 1) readonly buffer Locators {
    LocatorPickData values[];
} locator_buf;

layout(location = 0) in vec3 inPosition;
layout(location = 0) flat out uint element_id;

void main() {
    const LocatorPickData locator =
        locator_buf.values[gl_InstanceIndex];
    element_id = locator.token;
    gl_Position = camera.proj * camera.view * locator.model
        * vec4(inPosition, 1.0);
}
