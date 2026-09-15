#pragma once

#include "graph_ir.hpp"
#include "graph_validation.hpp"

#include <string>
#include <string_view>
#include <vector>

namespace orlrig
{

struct RigGraphResourceIds {
    orlgraph::StableId bind_positions;
    orlgraph::StableId posed_positions;
    orlgraph::StableId joints;
    orlgraph::StableId inverse_binds;
    orlgraph::StableId weights;
};

inline constexpr std::string_view kSceneJointsBinding = "scene.rig.joints";
inline constexpr std::string_view kSceneJointCountBinding = "scene.rig.joint_count";
inline constexpr std::string_view kSceneControllersBinding =
    "scene.rig.controllers";
inline constexpr std::string_view kSceneControllersCountBinding =
    "scene.rig.controller_count";
inline constexpr std::string_view kSceneLocatorsBinding =
    "scene.rig.locators";
inline constexpr std::string_view kSceneLocatorsCountBinding =
    "scene.rig.locator_count";
inline constexpr std::string_view kSceneArrayIndexSemantic =
    "scene.array_index";
inline constexpr std::string_view kSceneJointHandleSemantic =
    "scene.joint.handle";
inline constexpr std::string_view kSceneControllerHandleSemantic =
    "scene.controller.handle";
inline constexpr std::string_view kSceneLocatorHandleSemantic =
    "scene.locator.handle";
inline constexpr std::string_view kJointWorldMatrixConversion =
    "orlrig.convert.joint_world_matrix";
inline constexpr std::string_view kJointWorldMatrixWritebackConversion =
    "orlrig.convert.joint_world_matrix_to_trs";
inline constexpr std::string_view kComputedJointsNodeDefinition =
    "orlrig.stage.computed_joints";
inline constexpr std::string_view kComputedJointsBinding =
    "stage.rig.computed_joints";

std::string scene_mesh_positions_binding(std::string_view object_name);
std::string scene_mesh_vertex_count_binding(std::string_view object_name);
std::string scene_weight_buffer_binding(std::string_view component_name);
std::string scene_weight_count_binding(std::string_view component_name);
std::string scene_inverse_bindings_binding(std::string_view component_name);
std::string scene_controller_xform_binding(std::string_view component_name);
std::string scene_controller_count_binding(std::string_view component_name);
std::string scene_locator_xform_binding(std::string_view component_name);
std::string scene_locator_count_binding(std::string_view component_name);

struct RigGraph {
    orlgraph::GraphModule module;
    orlgraph::NodeRegistry registry;
    RigGraphResourceIds resources;

    orlgraph::ValidationResult validate() const;
};

// Describes renderer-independent resources used by the standard LBS graph.
RigGraphResourceIds add_lbs_resources(orlgraph::GraphModule& module);

// Registers standard runtime and stdlib node definitions for rig graphs.
bool register_rig_node_definitions(orlgraph::NodeRegistry& registry,
    std::string* error = nullptr);

// Builds a dependency graph for bind capture followed by LBS deformation.
// Execution is still supplied by the standalone runners; the graph owns the
// ordering and resource contracts.
RigGraph make_lbs_graph();

} // namespace orlrig
