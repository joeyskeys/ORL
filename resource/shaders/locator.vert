#version 450

layout(binding = 0) uniform CameraUBO {
    mat4 view;
    mat4 proj;
} ubo;

struct LocatorDrawData {
    mat4 model;
    vec4 color;
};

layout(std430, binding = 1) readonly buffer Locators {
    LocatorDrawData values[];
} locator_buf;

layout(location = 0) in vec3 inPosition;
layout(location = 0) flat out vec4 fragColor;

void main() {
    const LocatorDrawData locator =
        locator_buf.values[gl_InstanceIndex];
    fragColor = locator.color;
    gl_Position = ubo.proj * ubo.view * locator.model
        * vec4(inPosition, 1.0);
}
