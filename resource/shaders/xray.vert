#version 450

layout(binding = 0) uniform CameraUBO {
    mat4 view;
    mat4 proj;
} ubo;

struct XRayInstanceAttr {
    mat4 model;
    vec4 color;
};

layout(std430, binding = 1) readonly buffer XRayInstanceAttrs {
    XRayInstanceAttr attrs[];
} instance_attrs;

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inNormal;

layout(location = 0) out vec3 fragPos;
layout(location = 1) out vec3 fragNormal;
layout(location = 2) flat out vec4 fragColor;

void main() {
    XRayInstanceAttr inst = instance_attrs.attrs[gl_InstanceIndex];
    vec4 world_pos = inst.model * vec4(inPosition, 1.0);
    fragPos = world_pos.xyz;
    fragNormal = mat3(transpose(inverse(inst.model))) * inNormal;
    fragColor = inst.color;
    gl_Position = ubo.proj * ubo.view * world_pos;
}
