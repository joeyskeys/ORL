#pragma once

#include <algorithm>
#include <cstdint>
#include <optional>
#include <vector>

#include <glm/geometric.hpp>
#include <glm/vec3.hpp>

#include "joint.hpp"

namespace orlrig
{

struct TwoBoneChain {
    std::int64_t root = -1;
    std::int64_t mid = -1;
    std::int64_t end = -1;
    glm::vec3 root_world{};
    glm::vec3 mid_world{};
    glm::vec3 end_world{};
};

inline std::optional<TwoBoneChain> make_two_bone_chain(
    const std::vector<Joint>& joints,
    std::int64_t end)
{
    if (end < 0 || static_cast<std::size_t>(end) >= joints.size()) {
        return std::nullopt;
    }
    const std::int64_t mid = joints[static_cast<std::size_t>(end)].parent;
    if (mid < 0 || static_cast<std::size_t>(mid) >= joints.size()) {
        return std::nullopt;
    }
    const std::int64_t root = joints[static_cast<std::size_t>(mid)].parent;
    if (root < 0 || static_cast<std::size_t>(root) >= joints.size()) {
        return std::nullopt;
    }

    return TwoBoneChain{
        root,
        mid,
        end,
        glm::vec3{joint_world_matrix(joints, root)[3]},
        glm::vec3{joint_world_matrix(joints, mid)[3]},
        glm::vec3{joint_world_matrix(joints, end)[3]},
    };
}

inline glm::vec3 pole_position(const glm::vec3& root,
    const glm::vec3& mid,
    const glm::vec3& end)
{
    const glm::vec3 bone = end - root;
    const float bone2 = glm::dot(bone, bone);
    glm::vec3 side = mid - root;
    if (bone2 > 1.0e-8f) {
        side = side - bone * (glm::dot(side, bone) / bone2);
    }
    if (glm::length(side) < 1.0e-4f) {
        side = glm::cross(bone, glm::vec3{0.0f, 1.0f, 0.0f});
        if (glm::length(side) < 1.0e-4f) {
            side = glm::cross(bone, glm::vec3{1.0f, 0.0f, 0.0f});
        }
    }
    if (glm::length(side) < 1.0e-8f) {
        return mid + glm::vec3{0.0f, 0.25f, 0.0f};
    }
    const float offset = std::max(glm::length(mid - root), 0.25f);
    return mid + glm::normalize(side) * offset;
}

} // namespace orlrig
