#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <glm/geometric.hpp>
#include <glm/gtc/matrix_inverse.hpp>
#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

namespace orlviewer {

// Shared CPU/GPU joint storage. Field order matches resource/stdlib/joint.orl.
// Padding follows ORL's LLVM/host ABI and GLSL std430:
//   i64 parent, then 32-byte slots for <3 x double>, <4 x double>, <3 x double>.
// This buffer is meant to be bound to ORL as Joint joints[] and then drawn
// from the same device memory. Do not introduce a CPU staging copy for that path.
inline constexpr const char *kJointOrlType = "Joint";
inline constexpr std::size_t kJointStride = 128;

// Core fields a user-defined joint must supply so the viewer can draw hierarchy.
// Rotation and scale default to identity when the user type has no pose.
struct JointSample {
    std::int64_t parent = -1;
    double location[3] = {0.0, 0.0, 0.0};
    double rotation[4] = {0.0, 0.0, 0.0, 1.0};
    double scale[3] = {1.0, 1.0, 1.0};
};

struct alignas(32) Joint {
    std::int64_t parent;
    double pad_parent[3];
    double translation[4];
    double rotation[4];
    double scale[4];
};

static_assert(sizeof(Joint) == kJointStride, "Joint stride must stay 128 bytes");
static_assert(offsetof(Joint, translation) == 32, "Joint translation must start at 32");
static_assert(offsetof(Joint, rotation) == 64, "Joint rotation must start at 64");
static_assert(offsetof(Joint, scale) == 96, "Joint scale must start at 96");

inline Joint make_identity_joint() {
    Joint joint{};
    joint.parent = -1;
    joint.rotation[3] = 1.0;
    joint.scale[0] = 1.0;
    joint.scale[1] = 1.0;
    joint.scale[2] = 1.0;
    return joint;
}

inline Joint make_joint(const JointSample &sample) {
    Joint joint = make_identity_joint();
    joint.parent = sample.parent;
    joint.translation[0] = sample.location[0];
    joint.translation[1] = sample.location[1];
    joint.translation[2] = sample.location[2];
    joint.rotation[0] = sample.rotation[0];
    joint.rotation[1] = sample.rotation[1];
    joint.rotation[2] = sample.rotation[2];
    joint.rotation[3] = sample.rotation[3];
    joint.scale[0] = sample.scale[0];
    joint.scale[1] = sample.scale[1];
    joint.scale[2] = sample.scale[2];
    return joint;
}

inline JointSample joint_sample(const Joint &joint) {
    JointSample sample;
    sample.parent = joint.parent;
    sample.location[0] = joint.translation[0];
    sample.location[1] = joint.translation[1];
    sample.location[2] = joint.translation[2];
    sample.rotation[0] = joint.rotation[0];
    sample.rotation[1] = joint.rotation[1];
    sample.rotation[2] = joint.rotation[2];
    sample.rotation[3] = joint.rotation[3];
    sample.scale[0] = joint.scale[0];
    sample.scale[1] = joint.scale[1];
    sample.scale[2] = joint.scale[2];
    return sample;
}

// Import a user joint-like record. ParentOf(user) -> int64 parent.
// LocationOf(user, double[3]) writes the draw location.
template <typename UserJoint, typename ParentOf, typename LocationOf>
inline void import_joints(Joint *destination,
                          const UserJoint *source,
                          std::size_t count,
                          ParentOf parent_of,
                          LocationOf location_of) {
    for (std::size_t i = 0; i < count; ++i) {
        JointSample sample;
        sample.parent = parent_of(source[i]);
        location_of(source[i], sample.location);
        destination[i] = make_joint(sample);
    }
}

inline glm::mat4 joint_local_matrix(const Joint& joint) {
    glm::vec4 q{
        static_cast<float>(joint.rotation[0]),
        static_cast<float>(joint.rotation[1]),
        static_cast<float>(joint.rotation[2]),
        static_cast<float>(joint.rotation[3]),
    };
    const float qlen = glm::length(q);
    q = qlen > 0.0f ? q / qlen : glm::vec4{0.0f, 0.0f, 0.0f, 1.0f};

    const float xx = q.x * q.x;
    const float yy = q.y * q.y;
    const float zz = q.z * q.z;
    const float xy = q.x * q.y;
    const float xz = q.x * q.z;
    const float yz = q.y * q.z;
    const float wx = q.w * q.x;
    const float wy = q.w * q.y;
    const float wz = q.w * q.z;
    const float sx = static_cast<float>(joint.scale[0]);
    const float sy = static_cast<float>(joint.scale[1]);
    const float sz = static_cast<float>(joint.scale[2]);

    return glm::mat4{
        glm::vec4{sx * (1.0f - 2.0f * (yy + zz)), sx * (2.0f * (xy + wz)), sx * (2.0f * (xz - wy)), 0.0f},
        glm::vec4{sy * (2.0f * (xy - wz)), sy * (1.0f - 2.0f * (xx + zz)), sy * (2.0f * (yz + wx)), 0.0f},
        glm::vec4{sz * (2.0f * (xz + wy)), sz * (2.0f * (yz - wx)), sz * (1.0f - 2.0f * (xx + yy)), 0.0f},
        glm::vec4{
            static_cast<float>(joint.translation[0]),
            static_cast<float>(joint.translation[1]),
            static_cast<float>(joint.translation[2]),
            1.0f},
    };
}

inline glm::mat4 joint_world_matrix(const std::vector<Joint>& joints, std::int64_t index) {
    if (index < 0 || static_cast<std::size_t>(index) >= joints.size()) {
        return glm::mat4{1.0f};
    }

    glm::mat4 world = joint_local_matrix(joints[static_cast<std::size_t>(index)]);
    std::int64_t parent = joints[static_cast<std::size_t>(index)].parent;
    for (int i = 0; i < 64 && parent >= 0 && static_cast<std::size_t>(parent) < joints.size(); ++i) {
        world = joint_local_matrix(joints[static_cast<std::size_t>(parent)]) * world;
        parent = joints[static_cast<std::size_t>(parent)].parent;
    }
    return world;
}

inline glm::vec3 world_to_local(const std::vector<Joint>& joints, std::int64_t parent_index,
    const glm::vec3& world)
{
    if (parent_index < 0) {
        return world;
    }
    const glm::vec4 local = glm::inverse(joint_world_matrix(joints, parent_index))
        * glm::vec4{world, 1.0f};
    return glm::vec3{local};
}

} // namespace orlviewer
