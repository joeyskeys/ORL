#version 460

layout(binding = 0) uniform CameraUBO {
    mat4 view;
    mat4 proj;
} camera;

struct ObjectPickingInstance {
    mat4 model;
    uint object_id;
    uint pad0;
    uint pad1;
    uint pad2;
};

layout(std430, binding = 2) readonly buffer ObjectPickingInstances {
    ObjectPickingInstance attrs[];
} instances;

layout(location = 0) in vec3 in_position;
layout(location = 0) flat out uint object_id;

void main() {
    const ObjectPickingInstance instance = instances.attrs[gl_InstanceIndex];
    gl_Position = camera.proj * camera.view * instance.model * vec4(in_position, 1.0);
    object_id = instance.object_id;
}
