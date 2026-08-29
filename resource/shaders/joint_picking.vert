#version 450
#extension GL_ARB_gpu_shader_fp64 : require
#extension GL_EXT_shader_explicit_arithmetic_types_int64 : require

layout(binding = 0) uniform CameraUBO {
    mat4 view;
    mat4 proj;
} ubo;

// Keep layout identical to src/orlviewer/comps/joint.glsl (128-byte stride).
struct Joint {
    int64_t parent;
    int64_t selected;
    int64_t pad0;
    int64_t pad1;
    double translation[4];
    double rotation[4];
    double scale[4];
};

layout(std430, binding = 1) readonly buffer Joints {
    Joint joints[];
} joint_buf;

layout(binding = 2) uniform PointSizeUBO {
    vec4 value;
} point_size;

layout(location = 0) flat out uint joint_id;

mat4 joint_local_matrix(Joint joint) {
    vec4 q = vec4(
        float(joint.rotation[0]),
        float(joint.rotation[1]),
        float(joint.rotation[2]),
        float(joint.rotation[3]));
    float qlen = length(q);
    q = qlen > 0.0 ? q / qlen : vec4(0.0, 0.0, 0.0, 1.0);

    float xx = q.x * q.x;
    float yy = q.y * q.y;
    float zz = q.z * q.z;
    float xy = q.x * q.y;
    float xz = q.x * q.z;
    float yz = q.y * q.z;
    float wx = q.w * q.x;
    float wy = q.w * q.y;
    float wz = q.w * q.z;
    float sx = float(joint.scale[0]);
    float sy = float(joint.scale[1]);
    float sz = float(joint.scale[2]);

    return mat4(
        vec4(sx * (1.0 - 2.0 * (yy + zz)), sx * (2.0 * (xy + wz)), sx * (2.0 * (xz - wy)), 0.0),
        vec4(sy * (2.0 * (xy - wz)), sy * (1.0 - 2.0 * (xx + zz)), sy * (2.0 * (yz + wx)), 0.0),
        vec4(sz * (2.0 * (xz + wy)), sz * (2.0 * (yz - wx)), sz * (1.0 - 2.0 * (xx + yy)), 0.0),
        vec4(float(joint.translation[0]), float(joint.translation[1]),
            float(joint.translation[2]), 1.0));
}

void main() {
    int index = int(gl_InstanceIndex);
    mat4 world = joint_local_matrix(joint_buf.joints[index]);
    int parent = int(joint_buf.joints[index].parent);
    for (int i = 0; i < 64 && parent >= 0; ++i) {
        world = joint_local_matrix(joint_buf.joints[parent]) * world;
        parent = int(joint_buf.joints[parent].parent);
    }

    joint_id = uint(index);
    gl_Position = ubo.proj * ubo.view * (world * vec4(0.0, 0.0, 0.0, 1.0));
    gl_PointSize = max(point_size.value.x, 1.0);
}
