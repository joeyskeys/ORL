#pragma once

#include <cstddef>
#include <cstdint>

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

} // namespace orlviewer
