#pragma once

#include <cstddef>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <vector>

#include "scene_graph_context.hpp"
#include "selection.hpp"
#include "../orlexec/orl_graph_exec.hpp"
#include "../orlexec/orlrig/runners.hpp"

namespace vkkk
{
class Context;
}

namespace ORL
{

// Executes the active scene graph through backend-selected ORL segments and
// registered runtime adapters, then publishes the resulting scene state.
class GraphSceneRuntime final {
public:
    GraphSceneRuntime(SceneGraphContext& graph_context,
        const Selection& selection, ComponentId deformer_id,
        ComponentId weight_id);

    bool set_type(std::string_view name);
    void set_mesh(std::string name);
    void request_bind();
    void unbind();
    void on_update(vkkk::Context& context);

private:
    struct OrlSegment {
        orlgraph::GraphModule graph;
        std::optional<exec::OrlGraphProgram> program;
        std::optional<exec::OrlGraphExecution> execution;
    };

    struct ExecutionStep {
        enum class Kind {
            OrlSegment,
            RuntimeNode,
        };

        Kind kind = Kind::RuntimeNode;
        std::size_t index = 0;
        orlgraph::StableId node_id;
    };

    // The viewport stores features in a vector and moves them. A member
    // pointer remains valid after those moves; a lambda capturing this would
    // retain the address of the temporary feature.
    using RuntimeAdapter = bool (GraphSceneRuntime::*)(
        vkkk::Context&, const orlgraph::NodeInstance&, bool);

    bool ensure_lbs_graph();
    bool setup(vkkk::Context& context);
    bool evaluate(vkkk::Context& context);
    bool has_lbs_nodes() const;
    bool has_orl_nodes() const;
    bool ensure_execution_plan(
        const orlgraph::ValidationResult& validation,
        std::string* error);
    bool build_orl_segment(
        const std::vector<orlgraph::StableId>& node_ids,
        std::size_t segment_index,
        std::string* error);
    bool execute_orl_segment(
        vkkk::Context& context, std::size_t segment_index);
    bool execute_runtime_node(
        vkkk::Context& context,
        const orlgraph::NodeInstance& instance,
        bool capture);
    bool execute_scene_input_adapter(
        vkkk::Context& context,
        const orlgraph::NodeInstance& instance,
        bool capture);
    bool execute_lbs_capture_adapter(
        vkkk::Context& context,
        const orlgraph::NodeInstance& instance,
        bool capture);
    bool execute_lbs_evaluate_adapter(
        vkkk::Context& context,
        const orlgraph::NodeInstance& instance,
        bool capture);
    bool resolve_scene_input_node(
        const orlgraph::NodeDefinition& definition,
        const orlgraph::NodeInstance& instance);
    bool prepare_lbs_inputs(std::string* error);
    bool dispatch_graph(vkkk::Context& context, bool capture);
    bool add_scene_execution_input(
        orlgraph::GraphModule& module,
        std::string_view binding,
        std::string_view name,
        const orlgraph::LogicalType& element_type,
        const orlgraph::Domain& domain,
        const orlgraph::Shape& shape,
        std::string_view semantic,
        std::string_view coordinate_space,
        std::string* expression,
        std::string* error);
    bool prepare_runtime_output_expressions(
        orlgraph::GraphModule& module,
        const std::set<orlgraph::StableId>& node_ids,
        std::map<std::string, std::string>* expressions,
        std::string* error);
    bool add_segment_connection(
        const orlgraph::GraphModule& source,
        orlgraph::GraphModule& destination,
        const orlgraph::Connection& connection,
        const std::set<orlgraph::StableId>& node_ids,
        std::string* error);
    std::size_t graph_fingerprint() const;
    std::string runtime_output_key(
        const orlgraph::StableId& node,
        const orlgraph::StableId& port) const;
    void register_runtime_adapters();

    SceneGraphContext& graph_context_;
    const Selection& selection_;
    ComponentId deformer_id;
    ComponentId weight_id;
    std::string type_name{"lbs"};
    orlrig::LbsRunner runner_;
    std::map<std::string, RuntimeAdapter> runtime_adapters_;
    std::vector<OrlSegment> orl_segments_;
    std::vector<ExecutionStep> execution_plan_;
    std::size_t planned_graph_revision_ = 0;
    std::size_t planned_scene_revision_ = 0;
    std::size_t planned_graph_fingerprint_ = 0;
    exec::Backend planned_backend_ = exec::Backend::Cpu;
    bool execution_plan_ready_ = false;
    bool graph_active_ = false;
    bool logged_rest = false;
    bool logged_move = false;
};

} // namespace ORL
