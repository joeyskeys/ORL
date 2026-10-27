#version 450

#extension GL_ARB_gpu_shader_fp64 : require

layout(binding = 0) uniform CameraUBO {
    mat4 view;
    mat4 proj;
} ubo;

struct LocatorTransform {
    double xform[16];
};

struct LocatorDrawData {
    mat4 model;
    vec4 color;
};

layout(std430, binding = 1) readonly buffer LocatorTransforms {
    LocatorTransform values[];
} transform_buf;

layout(std430, binding = 2) readonly buffer Locators {
    LocatorDrawData values[];
} locator_buf;

layout(location = 0) in vec3 inPosition;
layout(location = 0) flat out vec4 fragColor;

mat4 locator_model(const LocatorTransform locator) {
    return mat4(
        vec4(locator.xform[0], locator.xform[4],
            locator.xform[8], locator.xform[12]),
        vec4(locator.xform[1], locator.xform[5],
            locator.xform[9], locator.xform[13]),
        vec4(locator.xform[2], locator.xform[6],
            locator.xform[10], locator.xform[14]),
        vec4(locator.xform[3], locator.xform[7],
            locator.xform[11], locator.xform[15]));
}

void main() {
    const LocatorTransform transform =
        transform_buf.values[gl_InstanceIndex];
    fragColor = locator_buf.values[gl_InstanceIndex].color;
    gl_Position = ubo.proj * ubo.view * locator_model(transform)
        * vec4(inPosition, 1.0);
}
