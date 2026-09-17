#include "scene_graph_context.hpp"

#include <cstdint>
#include <utility>

namespace ORL
{

SceneGraphContext::SceneGraphContext(vkkk::Scene& scene,
    ComponentManager& components)
    : scene_(scene)
    , components_(components)
    , scene_inputs_(scene, components)
{
}

void SceneGraphContext::set_graph(orlgraph::GraphModule module,
    orlgraph::NodeRegistry registry)
{
    graph_ = std::move(module);
    solver_graph_ = {};
    registry_ = std::move(registry);
    scene_inputs_.clear_computed_joints_device();
    input_mappings_.clear();
    pending_operations_.clear();
    staged_graphs_ = false;
    ++graph_revision_;
    ++graph_edit_revision_;
    ++evaluation_revision_;
}

void SceneGraphContext::set_stage_graphs(
    orlgraph::GraphModule solver,
    orlgraph::GraphModule deformer,
    orlgraph::NodeRegistry registry)
{
    solver_graph_ = std::move(solver);
    graph_ = std::move(deformer);
    registry_ = std::move(registry);
    scene_inputs_.clear_computed_joints_device();
    input_mappings_.clear();
    pending_operations_.clear();
    staged_graphs_ = true;
    ++graph_revision_;
    ++graph_edit_revision_;
    ++evaluation_revision_;
}

void SceneGraphContext::ensure_stage_graphs()
{
    if (staged_graphs_) {
        return;
    }
    solver_graph_ = {};
    solver_graph_.module_id = graph_.module_id.empty()
        ? "orlrig.solver" : graph_.module_id + ".solver";
    solver_graph_.language_version = graph_.language_version;
    solver_graph_.logical_abi_version = graph_.logical_abi_version;
    staged_graphs_ = true;
    ++graph_revision_;
    ++graph_edit_revision_;
    ++evaluation_revision_;
}

orlgraph::GraphModule& SceneGraphContext::stage_graph(
    orlgraph::GraphStage stage)
{
    if (stage == orlgraph::GraphStage::Solver) {
        ensure_stage_graphs();
        return solver_graph_;
    }
    return graph_;
}

const orlgraph::GraphModule& SceneGraphContext::stage_graph(
    orlgraph::GraphStage stage) const
{
    return stage == orlgraph::GraphStage::Solver
        ? solver_graph_ : graph_;
}

void SceneGraphContext::refresh_scene_inputs()
{
    const auto previous_revision = scene_inputs_.revision();
    scene_inputs_.refresh();
    if (scene_inputs_.revision() != previous_revision) {
        ++graph_revision_;
    }
}

void SceneGraphContext::set_computed_joints_device(
    std::optional<exec::DeviceBufferView> view,
    std::size_t element_count)
{
    scene_inputs_.set_computed_joints_device(
        std::move(view), element_count);
}

void SceneGraphContext::clear_computed_joints_device()
{
    scene_inputs_.clear_computed_joints_device();
}

bool SceneGraphContext::map_input(
    const orlgraph::StableId& graph_input,
    const orlgraph::StableId& scene_input,
    std::string* error)
{
    if (graph_.input(graph_input) == nullptr) {
        return set_error(error,
            "Unknown graph input: " + graph_input.value);
    }
    input_mappings_[graph_input] = scene_input;
    return true;
}

bool SceneGraphContext::map_input_by_binding(
    std::string_view graph_binding,
    std::string scene_binding,
    std::string* error)
{
    for (const auto& [id, input] : graph_.inputs()) {
        if (id.value == graph_binding || input.binding == graph_binding) {
            return map_input(id, orlgraph::StableId{std::move(scene_binding)},
                error);
        }
    }
    return set_error(error,
        "No graph input has binding '" + std::string{graph_binding} + "'");
}

void SceneGraphContext::clear_input_mappings()
{
    input_mappings_.clear();
}

bool SceneGraphContext::resolve_graph_input(
    const orlgraph::InterfacePort& graph_input,
    exec::GraphInputBinding& binding,
    std::string* error)
{
    const auto previous_scene_revision = scene_inputs_.revision();
    const auto mapped = input_mappings_.find(graph_input.id);
    if (mapped == input_mappings_.end()) {
        const bool resolved = scene_inputs_.resolve(graph_input, binding, error);
        if (scene_inputs_.revision() != previous_scene_revision) {
            ++graph_revision_;
        }
        return resolved;
    }

    orlgraph::InterfacePort scene_input;
    if (!scene_inputs_.make_interface_port(
            mapped->second, &scene_input, error))
    {
        return false;
    }
    const bool resolved = scene_inputs_.resolve(scene_input, binding, error);
    if (scene_inputs_.revision() != previous_scene_revision) {
        ++graph_revision_;
    }
    return resolved;
}

bool SceneGraphContext::bind_graph_inputs(exec::OrlGraphExecution& execution)
{
    const auto previous_scene_revision = scene_inputs_.revision();
    scene_inputs_.refresh();
    if (scene_inputs_.revision() != previous_scene_revision) {
        ++graph_revision_;
    }
    return bind_graph_inputs(execution, graph_);
}

bool SceneGraphContext::bind_graph_inputs(
    exec::OrlGraphExecution& execution,
    const orlgraph::GraphModule& module)
{
    scene_inputs_.set_cuda_evaluation(
        execution.backend() == exec::Backend::Cuda);
    if (!scene_inputs_.ensure_cuda_inputs()) {
        return false;
    }
    if (!execution.bind_graph_inputs(module,
        [this](const orlgraph::InterfacePort& graph_input,
            exec::GraphInputBinding& binding, std::string& error) {
            return resolve_graph_input(graph_input, binding, &error);
        }))
    {
        return false;
    }
    return execution.set_solver_context(
        static_cast<std::int64_t>(
            components_.size(ComponentKind::Joint)),
        static_cast<std::int64_t>(
            components_.size(ComponentKind::Controller)));
}

bool SceneGraphContext::commit_scene_writes(
    bool host_readback_complete, std::string* error)
{
    return scene_inputs_.commit_joints(host_readback_complete, error);
}

bool SceneGraphContext::commit_scene_writes(
    const exec::GraphEvaluationResult& result,
    bool host_readback_complete, std::string* error)
{
    if (!result.ok) {
        return set_error(error,
            result.errors.empty()
                ? "Graph evaluation did not complete"
                : result.errors.front());
    }
    return commit_scene_writes(host_readback_complete, error);
}

orlgraph::ValidationResult SceneGraphContext::validate() const
{
    return orlgraph::validate(graph_, registry_);
}

orlgraph::ValidationResult SceneGraphContext::validate(
    orlgraph::GraphStage stage) const
{
    return orlgraph::validate(stage_graph(stage), registry_, stage);
}

bool SceneGraphContext::has_runtime_node(
    std::string_view runtime_name) const
{
    for (const auto& [_, instance] : graph_.nodes()) {
        const auto* definition = registry_.find(instance.definition);
        if (definition != nullptr
            && definition->implementation.runtime_name == runtime_name)
        {
            return true;
        }
    }
    return false;
}

bool SceneGraphContext::request_operation(std::string operation)
{
    if (operation.empty()) {
        return false;
    }
    pending_operations_.insert(std::move(operation));
    return true;
}

bool SceneGraphContext::take_operation(std::string_view operation)
{
    const auto found = pending_operations_.find(std::string{operation});
    if (found == pending_operations_.end()) {
        return false;
    }
    pending_operations_.erase(found);
    return true;
}

bool SceneGraphContext::set_error(
    std::string* error, std::string message) const
{
    if (error != nullptr) {
        *error = std::move(message);
    }
    return false;
}

} // namespace ORL
