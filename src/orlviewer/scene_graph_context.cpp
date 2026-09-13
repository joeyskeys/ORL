#include "scene_graph_context.hpp"

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
    registry_ = std::move(registry);
    input_mappings_.clear();
    pending_operations_.clear();
    ++graph_revision_;
}

void SceneGraphContext::refresh_scene_inputs()
{
    const auto previous_revision = scene_inputs_.revision();
    scene_inputs_.refresh();
    if (scene_inputs_.revision() != previous_revision) {
        ++graph_revision_;
    }
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
    return execution.bind_graph_inputs(module,
        [this](const orlgraph::InterfacePort& graph_input,
            exec::GraphInputBinding& binding, std::string& error) {
            return resolve_graph_input(graph_input, binding, &error);
        });
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
