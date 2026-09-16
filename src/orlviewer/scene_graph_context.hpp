#pragma once

#include <cstddef>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <string_view>

#include "graph_scene_inputs.hpp"
#include "orlgraph/graph_ir.hpp"
#include "orlgraph/graph_validation.hpp"

namespace ORL
{

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
    }
    // Request evaluation without changing the graph. This is used when
    // kernels are re-enabled after authoring edits were made while disabled.
    void request_evaluation() { ++evaluation_revision_; }
    orlgraph::NodeRegistry& registry() { return registry_; }
    const orlgraph::NodeRegistry& registry() const { return registry_; }

    vkkk::Scene& scene() { return scene_; }
    const vkkk::Scene& scene() const { return scene_; }
    ComponentManager& components() { return components_; }
    const ComponentManager& components() const { return components_; }

    SceneInputCatalog& scene_inputs() { return scene_inputs_; }
    const SceneInputCatalog& scene_inputs() const { return scene_inputs_; }
    void refresh_scene_inputs();
    void set_computed_joints_device(
        std::optional<exec::DeviceBufferView> view,
        std::size_t element_count);
    void clear_computed_joints_device();

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

    orlgraph::ValidationResult validate() const;
    orlgraph::ValidationResult validate(orlgraph::GraphStage stage) const;
    bool has_runtime_node(std::string_view runtime_name) const;

    // Viewport operations are requested by input handlers and consumed by
    // graph runtime adapters during the ordered viewport update phase.
    bool request_operation(std::string operation);
    bool take_operation(std::string_view operation);

private:
    bool set_error(std::string* error, std::string message) const;

    vkkk::Scene& scene_;
    ComponentManager& components_;
    SceneInputCatalog scene_inputs_;
    orlgraph::GraphModule graph_;
    orlgraph::GraphModule solver_graph_;
    orlgraph::NodeRegistry registry_;
    std::map<orlgraph::StableId, orlgraph::StableId> input_mappings_;
    std::set<std::string> pending_operations_;
    std::size_t graph_revision_ = 0;
    std::size_t graph_edit_revision_ = 0;
    std::size_t evaluation_revision_ = 0;
    bool staged_graphs_ = false;
};

} // namespace ORL
