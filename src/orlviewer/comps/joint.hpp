#pragma once

#include "../../orlexec/orlrig/joint.hpp"

namespace orlviewer
{

using orlrig::Joint;
using orlrig::JointSample;
using orlrig::import_joints;
using orlrig::joint_local_matrix;
using orlrig::joint_sample;
using orlrig::joint_world_matrix;
using orlrig::make_identity_joint;
using orlrig::make_joint;
using orlrig::world_to_local;

inline constexpr const char* kJointOrlType = orlrig::kJointOrlType;
inline constexpr std::size_t kJointStride = orlrig::kJointStride;

} // namespace orlviewer
