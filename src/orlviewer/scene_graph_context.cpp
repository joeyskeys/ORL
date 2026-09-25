#include "scene_graph_context.hpp"

#include <algorithm>
#include <cstdint>
#include <functional>
#include <utility>
#include <variant>

namespace ORL
{

SceneGraphContext::SceneGraphContext(vkkk::Scene& scene,
    ComponentManager& components)
    : scene_(scene)
    , components_(components)
    , scene_inputs_(scene, components)
    , hierarchy_data_(
        orlrig::kHierarchyDataOrlType,
        orlrig::kHierarchyDataStride)
{
    observed_topology_signature_ = hierarchy_signature();
    observed_scene_input_revision_ = scene_inputs_.revision();
    update_change_set();
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
    pending_changes_ = {};
    mark_plan_invalidated();
    pose_snapshots_.clear();
    observed_topology_signature_ = hierarchy_signature();
    update_change_set();
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
    pending_changes_ = {};
    mark_plan_invalidated();
    pose_snapshots_.clear();
    observed_topology_signature_ = hierarchy_signature();
    update_change_set();
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
    mark_plan_invalidated();
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
        mark_plan_invalidated();
    }
    update_change_set();
}

ChangeSet SceneGraphContext::take_change_set()
{
    ChangeSet result = pending_changes_;
    pending_changes_ = {};
    return result;
}

std::size_t SceneGraphContext::hierarchy_signature() const
{
    const auto ids = components_.rigging().packed_joint_ids();
    const auto joints = components_.rigging().packed_joints();
    std::size_t signature = ids.size();
    for (std::size_t index = 0;
         index < ids.size() && index < joints.size();
         ++index)
    {
        signature ^= std::hash<std::uint64_t>{}(ids[index].value)
            + static_cast<std::size_t>(0x9e3779b9)
            + (signature << 6)
            + (signature >> 2);
        signature ^= std::hash<std::int64_t>{}(joints[index].parent)
            + static_cast<std::size_t>(0x9e3779b9)
            + (signature << 6)
            + (signature >> 2);
    }
    return signature;
}

std::size_t SceneGraphContext::evaluation_signature(
    orlgraph::GraphStage stage) const
{
    const auto& graph = stage_graph(stage);
    std::size_t signature = hierarchy_signature()
        ^ (static_cast<std::size_t>(stage) << 24);
    signature ^= std::hash<std::string>{}(graph.module_id)
        + static_cast<std::size_t>(0x9e3779b9)
        + (signature << 6) + (signature >> 2);
    for (const auto& [id, node] : graph.nodes()) {
        signature ^= std::hash<std::string>{}(id.value)
            + std::hash<std::string>{}(node.definition.value)
            + (signature << 6) + (signature >> 2);
        for (const auto& [name, value] : node.parameter_values) {
            signature ^= std::hash<std::string>{}(name)
                + (signature << 6) + (signature >> 2);
            if (const auto* text = std::get_if<std::string>(&value.value)) {
                signature ^= std::hash<std::string>{}(*text)
                    + (signature << 6) + (signature >> 2);
            }
        }
    }
    for (const auto& connection : graph.connections()) {
        signature ^= std::hash<std::string>{}(
                connection.source.owner.value
                + connection.source.port.value
                + connection.destination.owner.value
                + connection.destination.port.value)
            + (signature << 6) + (signature >> 2);
    }
    return signature;
}

void SceneGraphContext::invalidate_evaluation_plan()
{
    std::atomic_store(&compiled_evaluation_plan_,
        std::shared_ptr<const orlrig::EvaluationPlan>{});
    compiled_evaluation_signature_ = 0;
}

void SceneGraphContext::mark_plan_invalidated()
{
    invalidate_evaluation_plan();
    ++topology_revision_;
    pending_changes_.topology_revision = topology_revision_;
    pending_changes_.plan_invalidated = true;
    pending_changes_.full_evaluation = true;
    pending_changes_.generation = ++change_generation_;
}

namespace
{

template <typename T>
void hash_value(std::size_t* result, T value)
{
    const auto bits = std::hash<T>{}(value);
    *result ^= bits + static_cast<std::size_t>(0x9e3779b9)
        + (*result << 6) + (*result >> 2);
}

void hash_matrix(std::size_t* result, const glm::mat4& matrix)
{
    for (int column = 0; column < 4; ++column) {
        for (int row = 0; row < 4; ++row) {
            hash_value(result, matrix[column][row]);
        }
    }
}

} // namespace

std::map<std::uint64_t, std::size_t>
SceneGraphContext::capture_pose_snapshot() const
{
    std::map<std::uint64_t, std::size_t> current;
    components_.rigging().for_each([&](const orlrig::Component& component) {
        std::size_t value = 0;
        if (component.kind == ComponentKind::Joint) {
            const auto* joint = components_.rigging().joint(component.id);
            if (joint == nullptr) {
                return;
            }
            hash_value(&value, joint->parent);
            for (int index = 0; index < 4; ++index) {
                hash_value(&value, joint->translation[index]);
                hash_value(&value, joint->rotation[index]);
                hash_value(&value, joint->scale[index]);
            }
        } else if (component.kind == ComponentKind::Controller) {
            const auto* controller =
                components_.rigging().controller(component.id);
            if (controller == nullptr) {
                return;
            }
            hash_matrix(&value, controller->xform);
            hash_matrix(&value, controller->input_xform);
        } else if (component.kind == ComponentKind::Locator) {
            const auto* locator = components_.rigging().locator(component.id);
            if (locator == nullptr) {
                return;
            }
            hash_matrix(&value, locator->xform);
        } else {
            return;
        }
        current.emplace(component.id.value, value);
    });
    return current;
}

void SceneGraphContext::update_change_set()
{
    const auto current_topology = hierarchy_signature();
    if (observed_topology_signature_ != 0
        && current_topology != observed_topology_signature_)
    {
        mark_plan_invalidated();
    }
    observed_topology_signature_ = current_topology;

    if (observed_scene_input_revision_ != scene_inputs_.revision()) {
        observed_scene_input_revision_ = scene_inputs_.revision();
        mark_plan_invalidated();
    }

    const auto current = capture_pose_snapshot();
    for (const auto& [component_value, value] : current) {
        const ComponentId id{component_value};
        const auto previous = pose_snapshots_.find(id.value);
        if (previous == pose_snapshots_.end() || previous->second == value) {
            continue;
        }
        const auto* component = components_.rigging().find(id);
        if (component == nullptr) {
            continue;
        }
        if (component->kind == ComponentKind::Joint) {
            if (std::find(pending_changes_.joints.begin(),
                    pending_changes_.joints.end(), id)
                == pending_changes_.joints.end())
            {
                pending_changes_.joints.push_back(id);
            }
        } else if (component->kind == ComponentKind::Controller) {
            if (std::find(pending_changes_.controllers.begin(),
                    pending_changes_.controllers.end(), id)
                == pending_changes_.controllers.end())
            {
                pending_changes_.controllers.push_back(id);
            }
        } else {
            if (std::find(pending_changes_.locators.begin(),
                    pending_changes_.locators.end(), id)
                == pending_changes_.locators.end())
            {
                pending_changes_.locators.push_back(id);
            }
        }
    }

    if (current.size() != pose_snapshots_.size()) {
        mark_plan_invalidated();
    }
    pose_snapshots_ = std::move(current);
    if (!pending_changes_.joints.empty()
        || !pending_changes_.controllers.empty()
        || !pending_changes_.locators.empty())
    {
        ++pose_revision_;
        pending_changes_.pose_revision = pose_revision_;
        pending_changes_.generation = ++change_generation_;
    }
    pending_changes_.topology_revision = topology_revision_;
}

void SceneGraphContext::acknowledge_pose_changes()
{
    pose_snapshots_ = capture_pose_snapshot();
}

bool SceneGraphContext::ensure_evaluation_plan(
    orlgraph::GraphStage stage, std::string* error)
{
    if (!ensure_hierarchy_plan(error)) {
        return false;
    }
    const auto signature = evaluation_signature(stage);
    if (compiled_evaluation_plan_ != nullptr
        && compiled_evaluation_signature_ == signature)
    {
        return true;
    }
    const auto result = orlrig::compile_evaluation_plan(
        stage_graph(stage), registry_, components_.rigging(),
        *compiled_hierarchy_plan);
    if (!result.plan.has_value() || !result.errors.empty()) {
        std::atomic_store(&compiled_evaluation_plan_,
            std::shared_ptr<const orlrig::EvaluationPlan>{});
        compiled_evaluation_signature_ = 0;
        std::string message = result.plan.has_value()
            ? "Evaluation plan contains invalid regions"
            : "Evaluation plan compilation failed";
        for (const auto& detail : result.errors) {
            message += ": " + detail;
        }
        return set_error(error, std::move(message));
    }
    auto plan = std::make_shared<const orlrig::EvaluationPlan>(
        std::move(*result.plan));
    std::atomic_store(&compiled_evaluation_plan_, std::move(plan));
    compiled_evaluation_signature_ = signature;
    return true;
}

std::shared_ptr<const orlrig::EvaluationPlan>
SceneGraphContext::evaluation_plan() const
{
    return std::atomic_load(&compiled_evaluation_plan_);
}

bool SceneGraphContext::ensure_hierarchy_plan(std::string* error)
{
    const auto signature = hierarchy_signature();
    if (compiled_hierarchy_plan.has_value()
        && compiled_hierarchy_signature == signature)
    {
        return true;
    }
    if (compiled_hierarchy_plan.has_value()) {
        mark_plan_invalidated();
    }

    orlrig::HierarchyCompileOptions options;
    options.topology_revision = topology_revision_;
    const auto result = orlrig::compile_hierarchy_plan(
        components_.rigging(), options);
    if (!result) {
        compiled_hierarchy_plan.reset();
        compiled_hierarchy_signature = 0;
        hierarchy_context_ = {};
        hierarchy_data_.clear();
        std::string message = "Hierarchy compilation failed";
        for (const auto& detail : result.errors) {
            message += ": " + detail;
        }
        return set_error(error, std::move(message));
    }

    compiled_hierarchy_plan = result.plan;
    const auto packed = orlrig::pack_hierarchy_plan(*compiled_hierarchy_plan);
    hierarchy_context_ = packed.context;
    if (!hierarchy_data_.resize(packed.data.size())) {
        compiled_hierarchy_plan.reset();
        compiled_hierarchy_signature = 0;
        hierarchy_context_ = {};
        hierarchy_data_.clear();
        return set_error(error,
            "Failed to allocate packed hierarchy data buffer");
    }
    for (std::size_t index = 0; index < packed.data.size(); ++index) {
        if (!hierarchy_data_.write(index, packed.data[index])) {
            compiled_hierarchy_plan.reset();
            compiled_hierarchy_signature = 0;
            hierarchy_context_ = {};
            hierarchy_data_.clear();
            return set_error(error,
                "Failed to populate packed hierarchy data buffer");
        }
    }
    compiled_hierarchy_signature = signature;
    return true;
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
    if (!scene_inputs_.bind_solver_context(execution)) {
        return false;
    }
    return execution.bind_handle_view_context(
        scene_inputs_.handle_view_context());
}

bool SceneGraphContext::commit_scene_writes(
    bool host_readback_complete, std::string* error)
{
    if (!scene_inputs_.commit_joints(host_readback_complete, error)) {
        return false;
    }
    acknowledge_pose_changes();
    return true;
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

void SceneGraphContext::retarget_element_name(
    std::string_view qualified_name,
    std::string_view old_name, std::string_view new_name)
{
    if (old_name.empty() || old_name == new_name) {
        return;
    }
    ensure_stage_graphs();
    bool changed = false;
    const auto retarget = [&](orlgraph::GraphModule& graph) {
        for (auto& [_, node] : graph.mutable_nodes()) {
            const auto* definition = registry_.find(node.definition);
            if (definition == nullptr
                || definition->qualified_name != qualified_name)
            {
                continue;
            }
            const auto parameter = node.parameter_values.find("name");
            if (parameter == node.parameter_values.end()) {
                continue;
            }
            auto* value = std::get_if<std::string>(&parameter->second.value);
            if (value == nullptr || *value != old_name) {
                continue;
            }
            *value = std::string{new_name};
            changed = true;
        }
    };
    retarget(solver_graph_);
    retarget(graph_);
    if (changed) {
        touch_graph();
    }
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
