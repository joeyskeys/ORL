#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <vector>

#include "graph_scene_inputs.hpp"
#include "orlgraph/graph_ir.hpp"
#include "orlgraph/graph_validation.hpp"
#include "orlrig/evaluation.hpp"
#include "orlrig/hierarchy.hpp"

namespace ORL
{

struct ChangeSet {
    std::uint64_t generation = 0;
    std::uint64_t topology_revision = 0;
    std::uint64_t pose_revision = 0;
    bool full_evaluation = false;
    bool plan_invalidated = false;
    std::vector<ComponentId> joints;
    std::vector<ComponentId> controllers;
    std::vector<ComponentId> locators;

    bool empty() const {
        return !full_evaluation && !plan_invalidated
            && joints.empty() && controllers.empty() && locators.empty();
    }
};

// Runtime relationship between one viewport scene and its graph document.
// The graph IR stays scene-independent; this object owns the ephemeral
// mapping from graph interface inputs to the current scene. Staged editor
// documents keep the solver module separately while graph() remains the
// deformer/legacy runtime module.
class SceneGraphContext final {
public:
    SceneGraphContext(vkkk::Scene& scene, ComponentManager& components);

    void set_graph(orlgraph::GraphModule module,
        orlgraph::NodeRegistry registry);
    void set_stage_graphs(orlgraph::GraphModule solver,
        orlgraph::GraphModule deformer,
        orlgraph::NodeRegistry registry);
    void ensure_stage_graphs();
    bool has_stage_graphs() const { return staged_graphs_; }
    std::size_t graph_revision() const { return graph_revision_; }
    std::size_t graph_edit_revision() const { return graph_edit_revision_; }
    std::size_t evaluation_revision() const { return evaluation_revision_; }
    std::size_t scene_input_revision() const {
        return scene_inputs_.revision();
    }
    ChangeSet take_change_set();

    orlgraph::GraphModule& graph() { return graph_; }
    const orlgraph::GraphModule& graph() const { return graph_; }
    orlgraph::GraphModule& stage_graph(orlgraph::GraphStage stage);
    const orlgraph::GraphModule& stage_graph(orlgraph::GraphStage stage) const;
    // GraphModule is intentionally a lightweight value object and does not
    // maintain an edit counter. Call this after mutating either stage so
    // runtimes can invalidate their compiled plans.
    void touch_graph() {
        ++graph_revision_;
        ++graph_edit_revision_;
        ++evaluation_revision_;
        mark_plan_invalidated();
    }
    // Request evaluation without changing the graph. This is used when
    // kernels are re-enabled after authoring edits were made while disabled.
    void request_evaluation() {
        ++evaluation_revision_;
        pending_changes_.full_evaluation = true;
        pending_changes_.generation = ++change_generation_;
    }
    orlgraph::NodeRegistry& registry() { return registry_; }
    const orlgraph::NodeRegistry& registry() const { return registry_; }

    vkkk::Scene& scene() { return scene_; }
    const vkkk::Scene& scene() const { return scene_; }
    ComponentManager& components() { return components_; }
    const ComponentManager& components() const { return components_; }

    SceneInputCatalog& scene_inputs() { return scene_inputs_; }
    const SceneInputCatalog& scene_inputs() const { return scene_inputs_; }
    void refresh_scene_inputs();
    bool ensure_hierarchy_plan(std::string* error = nullptr);
    const std::optional<orlrig::HierarchyPlan>& hierarchy_plan() const {
        return compiled_hierarchy_plan;
    }
    const orlrig::HierarchyContext& hierarchy_context() const {
        return hierarchy_context_;
    }
    bool ensure_evaluation_plan(
        orlgraph::GraphStage stage, std::string* error = nullptr);
    std::shared_ptr<const orlrig::EvaluationPlan> evaluation_plan() const;
    exec::OrlBuffer& hierarchy_data() { return hierarchy_data_; }
    const exec::OrlBuffer& hierarchy_data() const {
        return hierarchy_data_;
    }
    void set_computed_joints_device(
        std::optional<exec::DeviceBufferView> view,
        std::size_t element_count);
    void clear_computed_joints_device();
    std::optional<exec::DeviceBufferView> computed_joints_device() const {
        return scene_inputs_.computed_joints_device();
    }
    std::size_t computed_joints_device_count() const {
        return scene_inputs_.computed_joints_device_count();
    }

    // Map a graph interface input to a descriptor supplied by the current
    // scene. The mapping is intentionally explicit because graph bindings
    // such as "joints" are generic while scene bindings are namespaced.
    bool map_input(const orlgraph::StableId& graph_input,
        const orlgraph::StableId& scene_input,
        std::string* error = nullptr);
    bool map_input_by_binding(std::string_view graph_binding,
        std::string scene_binding, std::string* error = nullptr);
    void clear_input_mappings();
    const std::map<orlgraph::StableId, orlgraph::StableId>& input_mappings() const {
        return input_mappings_;
    }

    bool resolve_graph_input(const orlgraph::InterfacePort& graph_input,
        exec::GraphInputBinding& binding, std::string* error = nullptr);
    bool bind_graph_inputs(exec::OrlGraphExecution& execution);
    bool bind_graph_inputs(exec::OrlGraphExecution& execution,
        const orlgraph::GraphModule& module);
    bool commit_scene_writes(bool host_readback_complete,
        std::string* error = nullptr);
    bool commit_scene_writes(const exec::GraphEvaluationResult& result,
        bool host_readback_complete, std::string* error = nullptr);
    void acknowledge_pose_changes();

    orlgraph::ValidationResult validate() const;
    orlgraph::ValidationResult validate(orlgraph::GraphStage stage) const;
    bool has_runtime_node(std::string_view runtime_name) const;

    // Viewport operations are requested by input handlers and consumed by
    // graph runtime adapters during the ordered viewport update phase.
    bool request_operation(std::string operation);
    bool take_operation(std::string_view operation);
    // Rewrite find-node name parameters after a component rename so graph
    // nodes keep pointing at the same element.
    void retarget_element_name(std::string_view qualified_name,
        std::string_view old_name, std::string_view new_name);

private:
    bool set_error(std::string* error, std::string message) const;
    std::size_t hierarchy_signature() const;
    std::size_t evaluation_signature(orlgraph::GraphStage stage) const;
    std::map<std::uint64_t, std::size_t> capture_pose_snapshot() const;
    void invalidate_evaluation_plan();
    void update_change_set();
    void mark_plan_invalidated();

    vkkk::Scene& scene_;
    ComponentManager& components_;
    SceneInputCatalog scene_inputs_;
    exec::OrlBuffer hierarchy_data_;
    orlrig::HierarchyContext hierarchy_context_;
    orlgraph::GraphModule graph_;
    orlgraph::GraphModule solver_graph_;
    orlgraph::NodeRegistry registry_;
    std::map<orlgraph::StableId, orlgraph::StableId> input_mappings_;
    std::set<std::string> pending_operations_;
    std::size_t graph_revision_ = 0;
    std::size_t graph_edit_revision_ = 0;
    std::size_t evaluation_revision_ = 0;
    bool staged_graphs_ = false;
    std::optional<orlrig::HierarchyPlan> compiled_hierarchy_plan;
    std::size_t compiled_hierarchy_signature = 0;
    std::shared_ptr<const orlrig::EvaluationPlan> compiled_evaluation_plan_;
    std::size_t compiled_evaluation_signature_ = 0;
    std::uint64_t topology_revision_ = 0;
    std::uint64_t pose_revision_ = 0;
    std::uint64_t change_generation_ = 0;
    ChangeSet pending_changes_;
    std::map<std::uint64_t, std::size_t> pose_snapshots_;
    std::size_t observed_topology_signature_ = 0;
    std::size_t observed_scene_input_revision_ = 0;
};

} // namespace ORL
