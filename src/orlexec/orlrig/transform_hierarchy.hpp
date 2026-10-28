#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <glm/mat4x4.hpp>

#include "component_store.hpp"
#include "hierarchy.hpp"

namespace orlrig
{

struct TransformNode {
    ComponentId id;
    ComponentKind kind = ComponentKind::Locator;
    std::int64_t packed_index = -1;
    std::int64_t parent_index = -1;
    glm::mat4 local_offset{1.0f};
    bool inherit_translation = true;
    bool inherit_rotation = true;
    bool inherit_scale = true;
};

struct TransformHierarchyPlan {
    std::uint64_t topology_revision = 0;
    std::uint64_t relation_revision = 0;
    std::vector<TransformNode> nodes;
    std::vector<std::size_t> order;
    std::vector<std::uint32_t> level_offsets;
    std::vector<std::size_t> level_nodes;

    std::optional<std::size_t> node_index(ComponentId id) const;
};

struct TransformHierarchyCompileResult {
    std::optional<TransformHierarchyPlan> plan;
    std::vector<std::string> errors;

    explicit operator bool() const {
        return plan.has_value() && errors.empty();
    }
};

// Selects the part of a parent transform that a child inherits.  Keeping
// this operation shared by authoring (maintain-world bind) and runtime
// evaluation avoids subtly different offset semantics for partial
// translation/rotation/scale inheritance.
glm::mat4 inherited_parent_matrix(
    const glm::mat4& parent,
    bool inherit_translation,
    bool inherit_rotation,
    bool inherit_scale);

TransformHierarchyCompileResult compile_transform_hierarchy_plan(
    const ComponentStore& components,
    const HierarchyPlan& joints);

// Materializes the final base transform for every locator.  Joint matrices
// are returned in packed-joint order and locator matrices in
// packed-locator order.  The caller may then refresh the scene input arena;
// no relation metadata is written into either runtime ABI.
bool evaluate_transform_hierarchy(
    const TransformHierarchyPlan& plan,
    const HierarchyPlan& joints,
    const ComponentStore& components,
    std::vector<glm::mat4>* joint_world,
    std::vector<glm::mat4>* locator_world,
    std::string* error = nullptr);

// Variant used by segmented runtime evaluation.  Locators already finalized
// by an earlier constraint segment are preserved as parent inputs while
// their not-yet-written descendants receive a fresh hierarchy base result.
bool evaluate_transform_hierarchy(
    const TransformHierarchyPlan& plan,
    const HierarchyPlan& joints,
    const ComponentStore& components,
    const std::vector<ComponentId>& preserve_locators,
    std::vector<glm::mat4>* joint_world,
    std::vector<glm::mat4>* locator_world,
    std::string* error = nullptr);

} // namespace orlrig
