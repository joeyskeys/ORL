#pragma once

#include <cstddef>
#include <map>
#include <set>
#include <string>
#include <string_view>

#include "graph_scene_inputs.hpp"
#include "orlgraph/graph_ir.hpp"
#include "orlgraph/graph_validation.hpp"

namespace ORL
{

// Runtime relationship between one viewport scene and one active graph
// document. The graph IR stays scene-independent; this object owns the
// ephemeral mapping from graph interface inputs to the current scene.
class SceneGraphContext final {
public:
    SceneGraphContext(vkkk::Scene& scene, ComponentManager& components);

    void set_graph(orlgraph::GraphModule module,
        orlgraph::NodeRegistry registry);
    std::size_t graph_revision() const { return graph_revision_; }
    std::size_t scene_input_revision() const {
        return scene_inputs_.revision();
    }

    orlgraph::GraphModule& graph() { return graph_; }
    const orlgraph::GraphModule& graph() const { return graph_; }
    orlgraph::NodeRegistry& registry() { return registry_; }
    const orlgraph::NodeRegistry& registry() const { return registry_; }

    vkkk::Scene& scene() { return scene_; }
    const vkkk::Scene& scene() const { return scene_; }
    ComponentManager& components() { return components_; }
    const ComponentManager& components() const { return components_; }

    SceneInputCatalog& scene_inputs() { return scene_inputs_; }
    const SceneInputCatalog& scene_inputs() const { return scene_inputs_; }
    void refresh_scene_inputs();

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
    orlgraph::NodeRegistry registry_;
    std::map<orlgraph::StableId, orlgraph::StableId> input_mappings_;
    std::set<std::string> pending_operations_;
    std::size_t graph_revision_ = 0;
};

} // namespace ORL
