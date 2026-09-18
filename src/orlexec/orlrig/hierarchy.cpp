#include "hierarchy.hpp"

#include <algorithm>
#include <cstdint>
#include <iterator>
#include <utility>

namespace orlrig
{
namespace
{

struct TraversalFrame {
    std::size_t packed_index = 0;
    std::size_t next_child = 0;
};

HierarchyCompileResult failure(std::string message) {
    HierarchyCompileResult result;
    result.errors.push_back(std::move(message));
    return result;
}

} // namespace

std::optional<std::size_t> HierarchyPlan::preorder_position(
    ComponentId joint) const
{
    const auto found = std::find(
        preorder_joints.begin(), preorder_joints.end(), joint);
    if (found == preorder_joints.end()) {
        return std::nullopt;
    }
    return static_cast<std::size_t>(
        std::distance(preorder_joints.begin(), found));
}

std::optional<ComponentId> HierarchyPlan::parent_of(
    ComponentId joint) const
{
    const auto position = preorder_position(joint);
    if (!position.has_value()) {
        return std::nullopt;
    }

    if (ancestor_storage == AncestorStorageMode::ParentChain) {
        if (parent_joints.size() != preorder_joints.size()) {
            return std::nullopt;
        }
        return parent_joints[*position];
    }

    if (ancestor_offsets.size() != preorder_joints.size() + 1) {
        return std::nullopt;
    }
    const auto begin = ancestor_offsets[*position];
    const auto end = ancestor_offsets[*position + 1];
    if (begin > end || end > ancestor_joints.size()) {
        return std::nullopt;
    }
    return begin == end
        ? ComponentId{}
        : ancestor_joints[end - 1];
}

bool HierarchyPlan::is_direct_parent(
    ComponentId parent, ComponentId child) const
{
    if (!parent) {
        return false;
    }
    const auto child_parent = parent_of(child);
    return child_parent.has_value() && *child_parent == parent;
}

HierarchyRuntimeData pack_hierarchy_plan(const HierarchyPlan& plan)
{
    HierarchyRuntimeData result;
    result.context.joint_count =
        static_cast<std::int64_t>(plan.joint_count);
    result.context.level_count = plan.level_offsets.empty()
        ? 0
        : static_cast<std::int64_t>(plan.level_offsets.size() - 1);
    result.context.ancestor_count = static_cast<std::int64_t>(
        plan.ancestor_storage == AncestorStorageMode::ParentChain
            ? plan.parent_joints.size()
            : plan.ancestor_joints.size());
    result.context.ancestor_storage =
        plan.ancestor_storage == AncestorStorageMode::ParentChain ? 0 : 1;

    const auto append_ids = [&result](
        const std::vector<ComponentId>& values) {
        const auto offset = static_cast<std::int64_t>(result.data.size());
        result.data.reserve(result.data.size() + values.size());
        for (const ComponentId value : values) {
            result.data.push_back(static_cast<std::int64_t>(value.value));
        }
        return offset;
    };
    const auto append_uints = [&result](
        const std::vector<std::uint32_t>& values) {
        const auto offset = static_cast<std::int64_t>(result.data.size());
        result.data.reserve(result.data.size() + values.size());
        for (const std::uint32_t value : values) {
            result.data.push_back(static_cast<std::int64_t>(value));
        }
        return offset;
    };

    result.context.preorder_offset = append_ids(plan.preorder_joints);
    result.context.depth_offset = append_uints(plan.depth);
    result.context.subtree_begin_offset =
        append_uints(plan.subtree_begin);
    result.context.subtree_end_offset = append_uints(plan.subtree_end);
    result.context.level_offsets_offset =
        append_uints(plan.level_offsets);
    result.context.level_joints_offset =
        append_ids(plan.level_joints);
    result.context.parent_joints_offset =
        append_ids(plan.parent_joints);
    result.context.ancestor_offsets_offset =
        append_uints(plan.ancestor_offsets);
    result.context.ancestor_joints_offset =
        append_ids(plan.ancestor_joints);
    // Keep the CUDA binding non-null for an empty hierarchy. The zero entry is
    // never addressable because all logical counts are zero.
    if (result.data.empty()) {
        result.data.push_back(0);
    }
    result.context.data_count =
        static_cast<std::int64_t>(result.data.size());
    return result;
}

HierarchyCompileResult compile_hierarchy_plan(
    const ComponentStore& components,
    HierarchyCompileOptions options)
{
    const std::vector<ComponentId> packed_ids =
        components.packed_joint_ids();
    const std::vector<Joint> packed_joints =
        components.packed_joints();
    if (packed_ids.size() != packed_joints.size()) {
        return failure("Packed joint IDs and joint data have different sizes");
    }

    const std::size_t joint_count = packed_joints.size();
    HierarchyPlan plan;
    plan.topology_revision = options.topology_revision;
    plan.joint_count = joint_count;
    plan.ancestor_storage = options.ancestor_storage;

    if (joint_count == 0) {
        if (options.build_level_buckets) {
            plan.level_offsets = {0};
        }
        if (options.ancestor_storage == AncestorStorageMode::Flattened) {
            plan.ancestor_offsets = {0};
        }
        HierarchyCompileResult result;
        result.plan = std::move(plan);
        return result;
    }

    std::vector<std::int64_t> parent_indices(joint_count, -1);
    std::vector<std::vector<std::size_t>> children(joint_count);
    std::vector<std::size_t> roots;
    roots.reserve(joint_count);

    for (std::size_t index = 0; index < joint_count; ++index) {
        const std::int64_t parent = packed_joints[index].parent;
        if (parent < -1
            || parent >= static_cast<std::int64_t>(joint_count))
        {
            return failure(
                "Joint parent index is out of range at packed index "
                + std::to_string(index));
        }
        if (parent == static_cast<std::int64_t>(index)) {
            return failure(
                "Joint cannot be its own parent at packed index "
                + std::to_string(index));
        }

        parent_indices[index] = parent;
        if (parent < 0) {
            roots.push_back(index);
        } else {
            children[static_cast<std::size_t>(parent)].push_back(index);
        }
    }

    plan.preorder_joints.reserve(joint_count);
    plan.depth.reserve(joint_count);
    plan.subtree_begin.reserve(joint_count);
    plan.subtree_end.reserve(joint_count);

    std::vector<std::uint8_t> colors(joint_count, 0);
    std::vector<std::size_t> preorder_packed_indices;
    preorder_packed_indices.reserve(joint_count);
    std::vector<std::size_t> preorder_positions(joint_count);

    for (const std::size_t root : roots) {
        if (colors[root] != 0) {
            return failure("Joint hierarchy contains a repeated root");
        }

        colors[root] = 1;
        preorder_positions[root] = plan.preorder_joints.size();
        preorder_packed_indices.push_back(root);
        plan.preorder_joints.push_back(packed_ids[root]);
        plan.depth.push_back(0);
        plan.subtree_begin.push_back(
            static_cast<std::uint32_t>(plan.preorder_joints.size() - 1));
        plan.subtree_end.push_back(0);

        std::vector<TraversalFrame> stack;
        stack.push_back({root, 0});
        while (!stack.empty()) {
            TraversalFrame& frame = stack.back();
            const auto& joint_children = children[frame.packed_index];
            if (frame.next_child == joint_children.size()) {
                colors[frame.packed_index] = 2;
                const std::size_t preorder_position =
                    preorder_positions[frame.packed_index];
                plan.subtree_end[preorder_position] =
                    static_cast<std::uint32_t>(plan.preorder_joints.size());
                stack.pop_back();
                continue;
            }

            const std::size_t child =
                joint_children[frame.next_child++];
            if (colors[child] != 0) {
                return failure("Joint hierarchy contains a cycle");
            }

            colors[child] = 1;
            preorder_positions[child] = plan.preorder_joints.size();
            preorder_packed_indices.push_back(child);
            plan.preorder_joints.push_back(packed_ids[child]);
            const std::size_t parent_preorder =
                preorder_positions[frame.packed_index];
            plan.depth.push_back(
                plan.depth[parent_preorder] + 1);
            plan.subtree_begin.push_back(
                static_cast<std::uint32_t>(plan.preorder_joints.size() - 1));
            plan.subtree_end.push_back(0);
            stack.push_back({child, 0});
        }
    }

    for (const std::uint8_t color : colors) {
        if (color == 0) {
            return failure("Joint hierarchy contains a cycle");
        }
    }

    if (options.build_level_buckets) {
        const auto maximum_depth = *std::max_element(
            plan.depth.begin(), plan.depth.end());
        std::vector<std::uint32_t> level_counts(
            static_cast<std::size_t>(maximum_depth) + 1, 0);
        for (const std::uint32_t depth : plan.depth) {
            ++level_counts[depth];
        }

        plan.level_offsets.resize(level_counts.size() + 1, 0);
        for (std::size_t level = 0; level < level_counts.size(); ++level) {
            plan.level_offsets[level + 1] =
                plan.level_offsets[level] + level_counts[level];
        }

        plan.level_joints.resize(joint_count);
        std::vector<std::uint32_t> next_level = plan.level_offsets;
        for (std::size_t preorder = 0;
             preorder < plan.preorder_joints.size();
             ++preorder)
        {
            const std::uint32_t depth = plan.depth[preorder];
            plan.level_joints[next_level[depth]++] =
                plan.preorder_joints[preorder];
        }
    }

    if (options.ancestor_storage == AncestorStorageMode::ParentChain) {
        plan.parent_joints.resize(joint_count);
        for (std::size_t preorder = 0;
             preorder < preorder_packed_indices.size();
             ++preorder)
        {
            const std::size_t packed_index =
                preorder_packed_indices[preorder];
            const std::int64_t parent = parent_indices[packed_index];
            if (parent >= 0) {
                plan.parent_joints[preorder] =
                    packed_ids[static_cast<std::size_t>(parent)];
            }
        }
    } else {
        plan.ancestor_offsets.assign(joint_count + 1, 0);
        for (std::size_t preorder = 0;
             preorder < preorder_packed_indices.size();
             ++preorder)
        {
            std::int64_t parent = parent_indices[
                preorder_packed_indices[preorder]];
            std::size_t steps = 0;
            std::vector<ComponentId> ancestors;
            while (parent >= 0) {
                if (steps++ >= joint_count) {
                    return failure("Joint hierarchy contains a cycle");
                }
                ancestors.push_back(
                    packed_ids[static_cast<std::size_t>(parent)]);
                parent = parent_indices[static_cast<std::size_t>(parent)];
            }
            std::reverse(ancestors.begin(), ancestors.end());
            plan.ancestor_joints.insert(
                plan.ancestor_joints.end(),
                ancestors.begin(),
                ancestors.end());
            plan.ancestor_offsets[preorder + 1] =
                static_cast<std::uint32_t>(plan.ancestor_joints.size());
        }
    }

    HierarchyCompileResult result;
    result.plan = std::move(plan);
    return result;
}

} // namespace orlrig
