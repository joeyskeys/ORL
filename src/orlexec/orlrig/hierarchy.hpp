#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "abi.hpp"
#include "component_store.hpp"

namespace orlrig
{

enum class AncestorStorageMode {
    ParentChain,
    Flattened,
};

struct HierarchyCompileOptions {
    AncestorStorageMode ancestor_storage =
        AncestorStorageMode::Flattened;
    std::uint64_t topology_revision = 0;
    bool build_level_buckets = true;
};

struct HierarchyPlan {
    std::uint64_t topology_revision = 0;
    std::size_t joint_count = 0;
    AncestorStorageMode ancestor_storage =
        AncestorStorageMode::Flattened;

    // Stable component IDs in deterministic hierarchy preorder.
    std::vector<ComponentId> preorder_joints;
    std::vector<std::uint32_t> depth;
    std::vector<std::uint32_t> subtree_begin;
    std::vector<std::uint32_t> subtree_end;

    // Optional level buckets. Offsets index level_joints, not preorder_joints.
    std::vector<std::uint32_t> level_offsets;
    std::vector<ComponentId> level_joints;

    // Populated only in ParentChain mode. Root entries are empty IDs.
    std::vector<ComponentId> parent_joints;

    // Populated only in Flattened mode. Each range is root-to-parent.
    std::vector<std::uint32_t> ancestor_offsets;
    std::vector<ComponentId> ancestor_joints;

    std::optional<std::size_t> preorder_position(ComponentId joint) const;
    std::optional<ComponentId> parent_of(ComponentId joint) const;
    bool is_direct_parent(ComponentId parent, ComponentId child) const;
};

struct HierarchyRuntimeData {
    HierarchyContext context;
    std::vector<std::int64_t> data;
};

HierarchyRuntimeData pack_hierarchy_plan(const HierarchyPlan& plan);

struct HierarchyCompileResult {
    std::optional<HierarchyPlan> plan;
    std::vector<std::string> errors;

    explicit operator bool() const {
        return plan.has_value() && errors.empty();
    }
};

HierarchyCompileResult compile_hierarchy_plan(
    const ComponentStore& components,
    HierarchyCompileOptions options = {});

} // namespace orlrig
