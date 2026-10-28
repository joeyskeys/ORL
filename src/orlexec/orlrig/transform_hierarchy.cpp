#include "transform_hierarchy.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <sstream>
#include <utility>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include "runners.hpp"

namespace orlrig
{
namespace
{

bool finite_matrix(const glm::mat4& value)
{
    for (int column = 0; column < 4; ++column) {
        for (int row = 0; row < 4; ++row) {
            if (!std::isfinite(value[column][row])) {
                return false;
            }
        }
    }
    return true;
}

} // namespace

glm::mat4 inherited_parent_matrix(
    const glm::mat4& parent,
    bool inherit_translation,
    bool inherit_rotation,
    bool inherit_scale)
{
    const glm::vec3 translation = glm::vec3{parent[3]};
    glm::vec3 axes[3] = {
        glm::vec3{parent[0]},
        glm::vec3{parent[1]},
        glm::vec3{parent[2]},
    };
    glm::vec3 scale{
        glm::length(axes[0]),
        glm::length(axes[1]),
        glm::length(axes[2]),
    };
    glm::mat3 rotation_matrix{1.0f};
    for (int axis = 0; axis < 3; ++axis) {
        if (scale[axis] > 1.0e-6f) {
            rotation_matrix[axis] = axes[axis] / scale[axis];
        }
    }
    const glm::quat rotation = glm::normalize(
        glm::quat_cast(rotation_matrix));

    glm::mat4 result{1.0f};
    if (inherit_translation) {
        result[3] = glm::vec4{translation, 1.0f};
    }
    if (inherit_rotation) {
        result *= glm::mat4_cast(rotation);
    }
    if (inherit_scale) {
        result *= glm::scale(glm::mat4{1.0f}, scale);
    }
    return result;
}

std::optional<std::size_t> TransformHierarchyPlan::node_index(
    ComponentId id) const
{
    for (std::size_t index = 0; index < nodes.size(); ++index) {
        if (nodes[index].id == id) {
            return index;
        }
    }
    return std::nullopt;
}

TransformHierarchyCompileResult compile_transform_hierarchy_plan(
    const ComponentStore& components,
    const HierarchyPlan& joints)
{
    TransformHierarchyCompileResult result;
    TransformHierarchyPlan plan;
    plan.topology_revision = joints.topology_revision;
    plan.relation_revision = components.relation_revision();

    const auto joint_ids = components.packed_joint_ids();
    const auto locator_ids = components.packed_locator_ids();
    plan.nodes.reserve(joint_ids.size() + locator_ids.size());
    for (const ComponentId id : joints.preorder_joints) {
        const auto packed = components.joint_index(id);
        if (packed < 0) {
            result.errors.push_back(
                "Transform hierarchy joint is missing from packed storage: "
                + std::to_string(id.value));
            continue;
        }
        plan.nodes.push_back({
            id, ComponentKind::Joint, packed, -1, glm::mat4{1.0f},
            true, true, true});
    }
    for (const ComponentId id : locator_ids) {
        plan.nodes.push_back({
            id, ComponentKind::Locator, components.locator_index(id), -1,
            glm::mat4{1.0f}, true, true, true});
    }
    if (!result.errors.empty()) {
        return result;
    }

    const auto relation_for = [&components](ComponentId child) {
        return components.transform_parent(child);
    };
    for (std::size_t index = 0; index < plan.nodes.size(); ++index) {
        auto& node = plan.nodes[index];
        if (node.kind != ComponentKind::Locator) {
            continue;
        }
        const auto* relation = relation_for(node.id);
        if (relation == nullptr || !relation->enabled
            || relation->parent_kind == TransformParentKind::World)
        {
            continue;
        }
        if (!finite_matrix(relation->local_offset)) {
            result.errors.push_back(
                "Transform parent offset is not finite for locator '"
                + std::to_string(node.id.value) + "'");
            continue;
        }
        const auto* parent = components.find(relation->parent);
        const auto expected = relation->parent_kind == TransformParentKind::Joint
            ? ComponentKind::Joint : ComponentKind::Locator;
        if (parent == nullptr || parent->kind != expected) {
            result.errors.push_back(
                "Transform parent reference is invalid for locator '"
                + std::to_string(node.id.value) + "'");
            continue;
        }
        const auto parent_index = [&plan, relation] {
            for (std::size_t candidate = 0;
                 candidate < plan.nodes.size(); ++candidate)
            {
                if (plan.nodes[candidate].id == relation->parent) {
                    return static_cast<std::int64_t>(candidate);
                }
            }
            return std::int64_t{-1};
        }();
        if (parent_index < 0) {
            result.errors.push_back(
                "Transform parent is not in the compiled hierarchy for "
                "locator '" + std::to_string(node.id.value) + "'");
            continue;
        }
        node.parent_index = parent_index;
        node.local_offset = relation->local_offset;
        node.inherit_translation = relation->inherit_translation;
        node.inherit_rotation = relation->inherit_rotation;
        node.inherit_scale = relation->inherit_scale;
    }
    if (!result.errors.empty()) {
        return result;
    }

    std::vector<std::vector<std::size_t>> children(plan.nodes.size());
    std::vector<std::size_t> indegree(plan.nodes.size(), 0);
    for (std::size_t index = 0; index < plan.nodes.size(); ++index) {
        const auto parent = plan.nodes[index].parent_index;
        if (parent < 0) {
            continue;
        }
        if (static_cast<std::size_t>(parent) >= plan.nodes.size()
            || parent == static_cast<std::int64_t>(index))
        {
            result.errors.push_back(
                "Transform hierarchy contains an invalid parent edge");
            continue;
        }
        children[static_cast<std::size_t>(parent)].push_back(index);
        ++indegree[index];
    }
    if (!result.errors.empty()) {
        return result;
    }

    std::vector<std::size_t> ready;
    for (std::size_t index = 0; index < indegree.size(); ++index) {
        if (indegree[index] == 0) {
            ready.push_back(index);
        }
    }
    const auto stable_ready_order = [&plan](
        std::size_t left, std::size_t right) {
        return plan.nodes[left].id.value < plan.nodes[right].id.value;
    };
    std::sort(ready.begin(), ready.end(), stable_ready_order);
    while (!ready.empty()) {
        std::vector<std::size_t> next;
        for (const std::size_t index : ready) {
            plan.order.push_back(index);
            for (const std::size_t child : children[index]) {
                if (--indegree[child] == 0) {
                    next.push_back(child);
                }
            }
        }
        std::sort(next.begin(), next.end(), stable_ready_order);
        ready = std::move(next);
    }
    if (plan.order.size() != plan.nodes.size()) {
        std::ostringstream message;
        message << "Transform hierarchy cycle involving";
        for (std::size_t index = 0; index < indegree.size(); ++index) {
            if (indegree[index] != 0) {
                message << " '" << plan.nodes[index].id.value << "'";
            }
        }
        result.errors.push_back(message.str());
        return result;
    }

    std::vector<std::size_t> depth(plan.nodes.size(), 0);
    std::size_t maximum_depth = 0;
    for (const std::size_t index : plan.order) {
        const auto parent = plan.nodes[index].parent_index;
        if (parent >= 0) {
            depth[index] = depth[static_cast<std::size_t>(parent)] + 1;
            maximum_depth = std::max(maximum_depth, depth[index]);
        }
    }
    plan.level_offsets.assign(maximum_depth + 2, 0);
    for (const std::size_t index : plan.order) {
        ++plan.level_offsets[depth[index] + 1];
    }
    for (std::size_t level = 0; level <= maximum_depth; ++level) {
        plan.level_offsets[level + 1] += plan.level_offsets[level];
    }
    plan.level_nodes.resize(plan.nodes.size());
    auto next_level = plan.level_offsets;
    for (const std::size_t index : plan.order) {
        plan.level_nodes[next_level[depth[index]]++] = index;
    }

    result.plan = std::move(plan);
    return result;
}

bool evaluate_transform_hierarchy(
    const TransformHierarchyPlan& plan,
    const HierarchyPlan& joints,
    const ComponentStore& components,
    std::vector<glm::mat4>* joint_world,
    std::vector<glm::mat4>* locator_world,
    std::string* error)
{
    return evaluate_transform_hierarchy(
        plan, joints, components, {}, joint_world, locator_world, error);
}

bool evaluate_transform_hierarchy(
    const TransformHierarchyPlan& plan,
    const HierarchyPlan& joints,
    const ComponentStore& components,
    const std::vector<ComponentId>& preserve_locators,
    std::vector<glm::mat4>* joint_world,
    std::vector<glm::mat4>* locator_world,
    std::string* error)
{
    const auto fail = [error](std::string message) {
        if (error != nullptr) {
            *error = std::move(message);
        }
        return false;
    };
    if (joint_world == nullptr || locator_world == nullptr) {
        return fail("Transform hierarchy output is null");
    }
    if (!compute_world_matrices_parallel(joints, components, joint_world)) {
        return fail("Failed to compute joint world transforms");
    }
    const auto joint_ids = components.packed_joint_ids();
    const auto locator_ids = components.packed_locator_ids();
    if (joint_world->size() != joint_ids.size()) {
        return fail("Joint world transform count is inconsistent");
    }
    locator_world->assign(locator_ids.size(), glm::mat4{1.0f});
    std::vector<glm::mat4> node_world(
        plan.nodes.size(), glm::mat4{1.0f});

    for (const std::size_t node_index : plan.order) {
        if (node_index >= plan.nodes.size()) {
            return fail("Transform hierarchy order is out of range");
        }
        const auto& node = plan.nodes[node_index];
        glm::mat4 world{1.0f};
        if (node.kind == ComponentKind::Joint) {
            if (node.packed_index < 0
                || static_cast<std::size_t>(node.packed_index)
                    >= joint_world->size())
            {
                return fail("Transform joint packed index is invalid");
            }
            world = (*joint_world)[static_cast<std::size_t>(
                node.packed_index)];
        } else {
            const auto* locator = components.locator(node.id);
            if (locator == nullptr) {
                return fail("Transform locator is missing");
            }
            const bool preserve = std::find(
                preserve_locators.begin(),
                preserve_locators.end(), node.id)
                != preserve_locators.end();
            if (preserve) {
                world = locator->xform;
            } else if (node.parent_index < 0) {
                world = locator->xform;
            } else {
                const auto parent = static_cast<std::size_t>(
                    node.parent_index);
                if (parent >= node_world.size()) {
                    return fail("Transform locator parent index is invalid");
                }
                const glm::mat4 parent_selected =
                    inherited_parent_matrix(
                        node_world[parent],
                        node.inherit_translation,
                        node.inherit_rotation,
                        node.inherit_scale);
                world = parent_selected * node.local_offset;
            }
            const auto packed = components.locator_index(node.id);
            if (packed < 0
                || static_cast<std::size_t>(packed) >= locator_world->size())
            {
                return fail("Transform locator packed index is invalid");
            }
            (*locator_world)[static_cast<std::size_t>(packed)] = world;
        }
        if (!finite_matrix(world)) {
            return fail("Transform hierarchy produced a non-finite matrix");
        }
        node_world[node_index] = world;
    }
    return true;
}

} // namespace orlrig
