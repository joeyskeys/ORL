#include "graph_scene_inputs.hpp"

#include <algorithm>
#include <cstring>
#include <limits>
#include <utility>

#include <glm/vec4.hpp>

#include "asset_mgr/drawable_mgr.h"
#include "concepts/mesh.h"
#include "orlrig/abi.hpp"
#include "orlrig/controller.hpp"
#include "orlrig/graph_resources.hpp"
#include "orlrig/locator.hpp"
#include "orl_runtime_signature.h"

namespace ORL
{

std::string scene_joint_handle_binding(std::string_view name) {
    return "scene.rig.joint." + std::string{name} + ".handle";
}

std::string scene_locator_handle_binding(std::string_view name) {
    return "scene.rig.locator." + std::string{name} + ".handle";
}

namespace
{

int vertex_float_offset(const vkkk::Mesh& mesh) {
    int offset = 0;
    for (const auto component : mesh.comps) {
        if (component == vkkk::VERTEX) {
            return offset;
        }
        offset += static_cast<int>(vkkk::comp_sizes[component]);
    }
    return -1;
}

std::size_t align_up(std::size_t value, std::size_t alignment) {
    const auto remainder = value % alignment;
    if (remainder == 0) {
        return value;
    }
    const auto padding = alignment - remainder;
    return value > std::numeric_limits<std::size_t>::max() - padding
        ? std::numeric_limits<std::size_t>::max()
        : value + padding;
}

} // namespace

SceneInputCatalog::SceneInputCatalog(vkkk::Scene& scene,
    ComponentManager& components)
    : scene_(scene)
    , components_(components)
    , joints_(orlrig::kJointOrlType, orlrig::kJointStride)
    , locators_(orlrig::kLocatorOrlType, orlrig::kLocatorStride)
    , controllers_(orlrig::kMatrixOrlType, orlrig::kMatrixStride)
{
    refresh();
}

void SceneInputCatalog::refresh() {
    descriptors_.clear();
    sources_.clear();
    // Keep per-binding buffers alive across refreshes. CUDA execution caches
    // device allocations by OrlBuffer address; destroying these buffers here
    // can make a new locator binding reuse another locator's device storage.
    pack_joints();
    pack_locators();
    pack_controllers();
    prepare_packed_inputs();
    add_descriptors();
}

const SceneInputDescriptor* SceneInputCatalog::find(
    const orlgraph::StableId& id) const
{
    const auto found = std::find_if(descriptors_.begin(), descriptors_.end(),
        [&id](const SceneInputDescriptor& descriptor) {
            return descriptor.id == id;
        });
    return found == descriptors_.end() ? nullptr : &*found;
}

std::vector<std::string> SceneInputCatalog::element_names(
    SceneElementKind kind) const
{
    std::vector<std::string> result;
    if (kind == SceneElementKind::Mesh) {
        scene_.for_each_object(
            [&result](const std::string& name, const vkkk::SceneObject&) {
                result.push_back(name);
            });
        std::sort(result.begin(), result.end());
        return result;
    }

    components_.for_each([&](const Component& component) {
        const bool matches = kind == SceneElementKind::Joint
            ? component.kind == ComponentKind::Joint
            : kind == SceneElementKind::Controller
                ? component.kind == ComponentKind::Controller
                : component.kind == ComponentKind::Locator;
        if (matches) {
            result.push_back(component.name);
        }
    });
    std::sort(result.begin(), result.end());
    return result;
}

std::optional<std::int64_t> SceneInputCatalog::resolve_element_handle(
    SceneElementKind kind, std::string_view name) const
{
    if (kind == SceneElementKind::Mesh) {
        std::vector<std::string> names = element_names(kind);
        const auto found = std::find(names.begin(), names.end(), name);
        if (found == names.end()) {
            return std::nullopt;
        }
        return static_cast<std::int64_t>(
            std::distance(names.begin(), found));
    }

    const auto* component = components_.find(name);
    if (component == nullptr) {
        return std::nullopt;
    }
    if (kind == SceneElementKind::Joint) {
        return component->kind == ComponentKind::Joint
            ? std::optional<std::int64_t>{
                static_cast<std::int64_t>(component->id.value)}
            : std::nullopt;
    }
    if (kind == SceneElementKind::Controller
        && component->kind != ComponentKind::Controller)
    {
        return std::nullopt;
    }
    if (kind == SceneElementKind::Locator
        && component->kind != ComponentKind::Locator)
    {
        return std::nullopt;
    }
    return static_cast<std::int64_t>(component->id.value);
}

bool SceneInputCatalog::make_interface_port(const orlgraph::StableId& id,
    orlgraph::InterfacePort* port, std::string* error) const
{
    if (port == nullptr) {
        return set_error(error, "Scene input destination is null");
    }
    const auto* descriptor = find(id);
    if (descriptor == nullptr) {
        return set_error(error, "Unknown scene input: " + id.value);
    }
    *port = descriptor->port;
    return true;
}

bool SceneInputCatalog::set_cuda_evaluation(bool enabled)
{
    if (cuda_evaluation == enabled) {
        return true;
    }
    cuda_evaluation = enabled;
    packed_solver_inputs.ready = false;
    return true;
}

bool SceneInputCatalog::ensure_cuda_inputs()
{
    if (!cuda_evaluation || packed_solver_inputs.ready) {
        return true;
    }
    refresh();
    return packed_solver_inputs.ready;
}

orlrig::HandleViewContext& SceneInputCatalog::handle_view_context()
{
    if (cuda_evaluation) {
        ensure_cuda_inputs();
    }
    if (packed_solver_inputs.ready) {
        handle_view_context_.joints = {
            packed_solver_inputs.storage.data()
                + packed_solver_inputs.joints_offset,
            packed_solver_inputs.joint_ids.size(),
            orlrig::kJointStride,
            true,
        };
        handle_view_context_.locators = {
            packed_solver_inputs.storage.data()
                + packed_solver_inputs.locators_offset,
            packed_solver_inputs.locator_ids.size(),
            orlrig::kLocatorStride,
            false,
        };
    } else {
        handle_view_context_.joints = {
            joints_.data(), joints_.count(), orlrig::kJointStride, true};
        handle_view_context_.locators = {
            locators_.data(), locators_.count(), orlrig::kLocatorStride, false};
    }
    handle_view_context_.topology_revision = revision_ == 0 ? 1 : revision_;
    return handle_view_context_;
}

bool SceneInputCatalog::resolve(const orlgraph::InterfacePort& port,
    exec::GraphInputBinding& binding, std::string* error)
{
    binding = {};
    if (cuda_evaluation && !ensure_cuda_inputs()) {
        return set_error(error, "Unable to prepare packed CUDA scene inputs");
    }
    const std::string key = port.binding.empty() ? port.id.value : port.binding;
    const auto descriptor = std::find_if(descriptors_.begin(), descriptors_.end(),
        [&key](const SceneInputDescriptor& candidate) {
            return candidate.port.binding == key;
        });
    if (descriptor != descriptors_.end()) {
        if (descriptor->port.type != port.type
            || descriptor->port.domain != port.domain
            || descriptor->port.shape != port.shape
            || (!descriptor->port.semantic.empty()
                && !port.semantic.empty()
                && descriptor->port.semantic != port.semantic))
        {
            return set_error(error,
                "Scene input '" + key + "' has incompatible graph metadata");
        }
        if (!port.coordinate_space.empty()
            && !descriptor->port.coordinate_space.empty()
            && port.coordinate_space != descriptor->port.coordinate_space)
        {
            return set_error(error,
                "Scene input '" + key + "' has incompatible coordinate space");
        }
    }
    const auto source = sources_.find(key);
    if (source == sources_.end()) {
        return set_error(error, "No scene input is registered for binding '" + key + "'");
    }

    const auto bind_packed = [&](std::size_t offset,
        std::size_t bytes, std::size_t element_count)
    {
        if (!packed_solver_inputs.ready) {
            return false;
        }
        binding.kind = exec::ParameterKind::Buffer;
        binding.element_count = element_count;
        binding.packed = packed_view(offset, bytes);
        return true;
    };

    switch (source->second.kind) {
    case SourceKind::MeshPositions: {
        if (!pack_mesh_positions(source->second.object_name, key)) {
            return set_error(error, "Unable to pack scene mesh positions for '" + key + "'");
        }
        const auto found = mesh_positions_.find(key);
        if (found == mesh_positions_.end()) {
            return set_error(error, "Scene mesh position buffer is unavailable for '" + key + "'");
        }
        binding.kind = exec::ParameterKind::Buffer;
        binding.buffer = &found->second;
        binding.element_count = found->second.count();
        return true;
    }
    case SourceKind::MeshVertexCount: {
        const auto* object = scene_.find_object(source->second.object_name);
        if (object == nullptr || scene_.drawable_mgr == nullptr) {
            return set_error(error, "Scene mesh object is unavailable for '" + key + "'");
        }
        const auto* mesh = scene_.drawable_mgr->find_mesh(object->mesh_name);
        if (mesh == nullptr) {
            return set_error(error, "Scene mesh is unavailable for '" + key + "'");
        }
        binding.kind = exec::ParameterKind::Int64;
        binding.int_value = static_cast<std::int64_t>(mesh->vcnt);
        return true;
    }
    case SourceKind::Joints:
        if (!pack_joints()) {
            return set_error(error, "Unable to pack scene joints");
        }
        if (cuda_evaluation
            && bind_packed(
                packed_solver_inputs.joints_offset,
                packed_solver_inputs.joint_ids.size() * orlrig::kJointStride,
                packed_solver_inputs.joint_ids.size()))
        {
            return true;
        }
        binding.kind = exec::ParameterKind::Buffer;
        binding.buffer = &joints_;
        binding.element_count = joints_.count();
        return true;
    case SourceKind::ComputedJoints:
        if (computed_joints_device_.has_value()) {
            binding.kind = exec::ParameterKind::Buffer;
            binding.device_ptr = computed_joints_device_->device_ptr;
            binding.bytes = computed_joints_device_->bytes;
            binding.element_count = computed_joints_device_count_;
            return true;
        }
        if (joints_.count() == 0 && !pack_joints()) {
            return set_error(error, "Unable to pack computed scene joints");
        }
        if (cuda_evaluation
            && bind_packed(
                packed_solver_inputs.joints_offset,
                packed_solver_inputs.joint_ids.size() * orlrig::kJointStride,
                packed_solver_inputs.joint_ids.size()))
        {
            return true;
        }
        binding.kind = exec::ParameterKind::Buffer;
        binding.buffer = &joints_;
        binding.element_count = joints_.count();
        return true;
    case SourceKind::JointCount:
        binding.kind = exec::ParameterKind::Int64;
        binding.int_value = static_cast<std::int64_t>(
            components_.packed_joints().size());
        return true;
    case SourceKind::JointHandle: {
        const auto index = components_.joint_index(source->second.component);
        if (index < 0) {
            return set_error(error,
                "Joint handle identity is no longer packed");
        }
        binding.kind = exec::ParameterKind::Handle;
        binding.handle_value = {
            orlcomp::HandleTypeIdFor("orlrig::joint_handle"), index};
        return true;
    }
    case SourceKind::Locators:
        if (!pack_locators()) {
            return set_error(error, "Unable to pack scene locators");
        }
        if (cuda_evaluation
            && bind_packed(
                packed_solver_inputs.locators_offset,
                packed_solver_inputs.locator_ids.size() * orlrig::kLocatorStride,
                packed_solver_inputs.locator_ids.size()))
        {
            return true;
        }
        binding.kind = exec::ParameterKind::Buffer;
        binding.buffer = &locators_;
        binding.element_count = locators_.count();
        return true;
    case SourceKind::LocatorCount:
        binding.kind = exec::ParameterKind::Int64;
        binding.int_value = source->second.component
            ? 1
            : static_cast<std::int64_t>(
                components_.packed_locators().size());
        return true;
    case SourceKind::LocatorHandle: {
        const auto found = std::find(
            locator_ids_.begin(), locator_ids_.end(),
            source->second.component);
        if (found == locator_ids_.end()) {
            return set_error(error,
                "Locator handle identity is no longer packed");
        }
        binding.kind = exec::ParameterKind::Handle;
        binding.handle_value = {
            orlcomp::HandleTypeIdFor("orlrig::locator_handle"),
            static_cast<std::int64_t>(
                std::distance(locator_ids_.begin(), found))};
        return true;
    }
    case SourceKind::LocatorXform:
        if (!pack_locator(source->second.component, key)) {
            return set_error(error, "Unable to pack locator transform for '" + key + "'");
        }
        if (cuda_evaluation) {
            const auto found_id = std::find(
                packed_solver_inputs.locator_ids.begin(),
                packed_solver_inputs.locator_ids.end(),
                source->second.component);
            if (found_id != packed_solver_inputs.locator_ids.end()
                && bind_packed(
                    packed_solver_inputs.locators_offset
                        + static_cast<std::size_t>(
                            std::distance(
                                packed_solver_inputs.locator_ids.begin(),
                                found_id))
                            * orlrig::kLocatorStride,
                    orlrig::kLocatorStride,
                    1))
            {
                return true;
            }
        }
        if (const auto found = locator_xforms_.find(key);
            found != locator_xforms_.end())
        {
            binding.kind = exec::ParameterKind::Buffer;
            binding.buffer = &found->second;
            binding.element_count = found->second.count();
            return true;
        }
        return set_error(error, "Locator transform is unavailable for '" + key + "'");
    case SourceKind::Controllers:
        if (!pack_controllers()) {
            return set_error(error, "Unable to pack scene controllers");
        }
        if (cuda_evaluation
            && bind_packed(
                packed_solver_inputs.controllers_offset,
                packed_solver_inputs.controller_ids.size()
                    * orlrig::kMatrixStride,
                packed_solver_inputs.controller_ids.size()))
        {
            return true;
        }
        binding.kind = exec::ParameterKind::Buffer;
        binding.buffer = &controllers_;
        binding.element_count = controllers_.count();
        return true;
    case SourceKind::ControllersCount:
        binding.kind = exec::ParameterKind::Int64;
        binding.int_value = static_cast<std::int64_t>(
            components_.size(ComponentKind::Controller));
        return true;
    case SourceKind::WeightBuffer: {
        auto* weights = components_.weight(source->second.component);
        if (weights == nullptr) {
            return set_error(error, "Scene weight component is unavailable for '" + key + "'");
        }
        binding.kind = exec::ParameterKind::Buffer;
        binding.buffer = &weights->weights;
        binding.element_count = weights->weights.count();
        return true;
    }
    case SourceKind::WeightCount: {
        const auto* weights = components_.weight(source->second.component);
        if (weights == nullptr) {
            return set_error(error, "Scene weight component is unavailable for '" + key + "'");
        }
        binding.kind = exec::ParameterKind::Int64;
        binding.int_value = weights->weight_cnt;
        return true;
    }
    case SourceKind::InverseBinds: {
        auto* deformer = components_.deformer(source->second.component);
        if (deformer == nullptr) {
            return set_error(error,
                "Scene deformer component is unavailable for '" + key + "'");
        }
        binding.kind = exec::ParameterKind::Buffer;
        binding.buffer = &deformer->inverse_binds;
        binding.element_count = deformer->inverse_binds.count();
        return true;
    }
    case SourceKind::ControllerXform: {
        if (!pack_controller(source->second.component, key)) {
            return set_error(error, "Unable to pack controller transform for '" + key + "'");
        }
        if (cuda_evaluation) {
            const auto found_id = std::find(
                packed_solver_inputs.controller_ids.begin(),
                packed_solver_inputs.controller_ids.end(),
                source->second.component);
            if (found_id != packed_solver_inputs.controller_ids.end()
                && bind_packed(
                    packed_solver_inputs.controllers_offset
                        + static_cast<std::size_t>(
                            std::distance(
                                packed_solver_inputs.controller_ids.begin(),
                                found_id))
                            * orlrig::kMatrixStride,
                    orlrig::kMatrixStride,
                    1))
            {
                return true;
            }
        }
        const auto found = controller_xforms_.find(key);
        if (found == controller_xforms_.end()) {
            return set_error(error, "Controller transform is unavailable for '" + key + "'");
        }
        binding.kind = exec::ParameterKind::Buffer;
        binding.buffer = &found->second;
        binding.element_count = found->second.count();
        return true;
    }
    case SourceKind::ControllerCount:
        if (components_.controller(source->second.component) == nullptr) {
            return set_error(error,
                "Scene controller component is unavailable for '" + key + "'");
        }
        binding.kind = exec::ParameterKind::Int64;
        binding.int_value = 1;
        return true;
    }
    return set_error(error, "Unsupported scene input binding: " + key);
}

bool SceneInputCatalog::resolve_binding(std::string_view binding,
    exec::GraphInputBinding& result, std::string* error)
{
    const auto descriptor = std::find_if(descriptors_.begin(), descriptors_.end(),
        [binding](const SceneInputDescriptor& candidate) {
            return candidate.port.binding == binding;
        });
    if (descriptor == descriptors_.end()) {
        return set_error(error,
            "No scene input is registered for binding '"
                + std::string{binding} + "'");
    }
    return resolve(descriptor->port, result, error);
}

void SceneInputCatalog::set_computed_joints_device(
    std::optional<exec::DeviceBufferView> view,
    std::size_t element_count)
{
    computed_joints_device_ = view;
    computed_joints_device_count_ = view.has_value() ? element_count : 0;
}

void SceneInputCatalog::clear_computed_joints_device()
{
    computed_joints_device_.reset();
    computed_joints_device_count_ = 0;
}

bool SceneInputCatalog::bind_graph_inputs(exec::OrlGraphExecution& execution,
    const orlgraph::GraphModule& module)
{
    refresh();
    if (!execution.bind_graph_inputs(module,
        [this](const orlgraph::InterfacePort& port,
            exec::GraphInputBinding& binding, std::string& error) {
            return resolve(port, binding, &error);
        }))
    {
        return false;
    }
    return bind_solver_context(execution);
}

bool SceneInputCatalog::bind_solver_context(
    exec::OrlGraphExecution& execution, std::string* error)
{
    if (!packed_solver_inputs.ready && !prepare_packed_inputs()) {
        return set_error(error,
            "Unable to prepare packed solver context storage");
    }
    const auto view = exec::PackedBufferView{
        packed_solver_inputs.storage.data(),
        packed_solver_inputs.storage.size(),
        0,
        packed_solver_inputs.storage.size(),
        packed_solver_inputs.version};
    if (!execution.bind_solver_context(view)) {
        return set_error(error,
            execution.errors().empty()
                ? "Unable to bind packed solver context"
                : execution.errors().front());
    }
    if (!execution.set_solver_context(solver_context())) {
        return set_error(error,
            execution.errors().empty()
                ? "Unable to update solver context"
                : execution.errors().front());
    }
    return true;
}

orlrig::SolverContext SceneInputCatalog::solver_context() const
{
    return orlrig::SolverContext{
        static_cast<std::int64_t>(joint_ids_.size()),
        static_cast<std::int64_t>(controller_ids_.size()),
        static_cast<std::int64_t>(locator_ids_.size()),
        static_cast<std::int64_t>(packed_solver_inputs.joints_offset),
        static_cast<std::int64_t>(packed_solver_inputs.controllers_offset),
        static_cast<std::int64_t>(packed_solver_inputs.locators_offset),
    };
}

std::optional<exec::DeviceBufferView>
SceneInputCatalog::solver_joints_device_view(
    exec::OrlGraphExecution& execution) const
{
    const auto context = solver_context();
    const auto base = execution.device_buffer_view(
        orlcomp::kSolverContextParameterName);
    if (!base.has_value() || context.joint_count <= 0) {
        return std::nullopt;
    }
    return exec::DeviceBufferView{
        base->device_ptr + static_cast<std::uint64_t>(
            context.joints_offset),
        static_cast<std::size_t>(context.joint_count)
            * orlrig::kJointStride};
}

bool SceneInputCatalog::commit_joints(
    bool host_readback_complete, std::string* error)
{
    if (!host_readback_complete) {
        return set_error(error,
            "Joint writeback requires a host-readback graph evaluation");
    }
    // The packed buffer is keyed by the stable IDs captured when it was
    // packed. Resolve those IDs again instead of assuming the current array
    // order is unchanged since evaluation.
    const auto& ids = packed_joint_ids_;
    if (joints_.count() < ids.size()) {
        return set_error(error,
            "Packed joint buffer has fewer elements than the component store");
    }
    const std::byte* source_bytes = nullptr;
    const auto current_joints = components_.packed_joints();
    const bool host_buffer_was_edited =
        joints_.count() != current_joints.size()
        || (!current_joints.empty()
            && std::memcmp(
                joints_.data(), current_joints.data(),
                current_joints.size() * orlrig::kJointStride) != 0);
    if (!host_buffer_was_edited
        && packed_solver_inputs.ready
        && packed_solver_inputs.storage.size()
            >= packed_solver_inputs.joints_offset
                + ids.size() * orlrig::kJointStride)
    {
        source_bytes = packed_solver_inputs.storage.data()
            + packed_solver_inputs.joints_offset;
    } else {
        source_bytes = static_cast<const std::byte*>(joints_.data());
    }
    const auto* source =
        reinterpret_cast<const orlviewer::Joint*>(source_bytes);
    if (source == nullptr && !ids.empty()) {
        return set_error(error, "Packed joint buffer has no host data");
    }
    for (std::size_t index = 0; index < ids.size(); ++index) {
        auto* destination = components_.joint(ids[index]);
        if (destination == nullptr) {
            // A removed component has no valid writeback target. Other
            // stable IDs in the evaluated buffer can still be committed
            // safely after the packed ordering changed.
            continue;
        }
        *destination = source[index];
    }
    return true;
}

void SceneInputCatalog::add_descriptors() {
    if (joints_.count() != 0) {
        add_buffer_descriptor(
            orlgraph::StableId{std::string{orlrig::kSceneJointsBinding}},
            "Scene Joints",
            orlgraph::LogicalType::struct_type("Joint"),
            orlgraph::Domain::joint(),
            orlgraph::Shape::one("joint_count"),
            "joints", "world", orlrig::kJointStride,
            std::string{orlrig::kSceneJointCountBinding},
            Source{SourceKind::Joints, {}, {}});
        add_scalar_descriptor(
            orlgraph::StableId{std::string{orlrig::kSceneJointCountBinding}},
            "Joint Count", "joint_count",
            Source{SourceKind::JointCount, {}, {}});
        add_buffer_descriptor(
            orlgraph::StableId{std::string{orlrig::kComputedJointsBinding}},
            "Computed Joints",
            orlgraph::LogicalType::struct_type("Joint"),
            orlgraph::Domain::joint(),
            orlgraph::Shape::one("joint_count"),
            "joints", "world", orlrig::kJointStride,
            std::string{orlrig::kSceneJointCountBinding},
            Source{SourceKind::ComputedJoints, {}, {}});
    }

    if (pack_locators() && locators_.count() != 0) {
        add_buffer_descriptor(
            orlgraph::StableId{
                std::string{orlrig::kSceneLocatorsBinding}},
            "Scene Locators",
            orlgraph::LogicalType::struct_type("Locator"),
            orlgraph::Domain::rig(),
            orlgraph::Shape::one("locator_count"),
            "locators", "world", orlrig::kLocatorStride,
            std::string{orlrig::kSceneLocatorsCountBinding},
            Source{SourceKind::Locators, {}, {}});
        add_scalar_descriptor(
            orlgraph::StableId{
                std::string{orlrig::kSceneLocatorsCountBinding}},
            "Locator Count", "locator_count",
            Source{SourceKind::LocatorCount, {}, {}});
    }

    if (pack_controllers() && controllers_.count() != 0) {
        add_buffer_descriptor(
            orlgraph::StableId{
                std::string{orlrig::kSceneControllersBinding}},
            "Scene Controllers",
            orlgraph::LogicalType::matrix(),
            orlgraph::Domain::rig(),
            orlgraph::Shape::one("controller_count"),
            "controllers", "world", orlrig::kMatrixStride,
            std::string{orlrig::kSceneControllersCountBinding},
            Source{SourceKind::Controllers, {}, {}});
        add_scalar_descriptor(
            orlgraph::StableId{
                std::string{orlrig::kSceneControllersCountBinding}},
            "Controller Count", "controller_count",
            Source{SourceKind::ControllersCount, {}, {}});
    }

    if (scene_.drawable_mgr != nullptr) {
        scene_.for_each_object([this](const std::string& object_name,
            const vkkk::SceneObject&) {
            const auto binding = orlrig::scene_mesh_positions_binding(object_name);
            if (!pack_mesh_positions(object_name, binding)) {
                return;
            }
            add_buffer_descriptor(
                orlgraph::StableId{binding},
                "Mesh " + object_name + " Positions",
                orlgraph::LogicalType::point(),
                orlgraph::Domain::vertex(),
                orlgraph::Shape::one("vertex_count"),
                "mesh.positions", "world", orlrig::kPointStride,
                orlrig::scene_mesh_vertex_count_binding(object_name),
                Source{SourceKind::MeshPositions, {}, object_name});
            add_scalar_descriptor(
                orlgraph::StableId{
                    orlrig::scene_mesh_vertex_count_binding(object_name)},
                "Mesh " + object_name + " Vertex Count",
                "vertex_count",
                Source{SourceKind::MeshVertexCount, {}, object_name});
        });
    }

    components_.for_each([this](const Component& meta) {
        if (meta.kind == ComponentKind::Weight) {
            const auto* weights = components_.weight(meta.id);
            if (weights == nullptr || weights->weights.count() == 0) {
                return;
            }
            const auto binding = orlrig::scene_weight_buffer_binding(meta.name);
            add_buffer_descriptor(
                orlgraph::StableId{binding},
                "Weights " + meta.name,
                orlgraph::LogicalType::struct_type("Weight"),
                orlgraph::Domain::vertex(),
                orlgraph::Shape::one("weight_count"),
                "weights", "world", orlrig::kWeightStride,
                orlrig::scene_weight_count_binding(meta.name),
                Source{SourceKind::WeightBuffer, meta.id, {}});
            add_scalar_descriptor(
                orlgraph::StableId{orlrig::scene_weight_count_binding(meta.name)},
                "Weights " + meta.name + " Influence Count",
                "weight_count",
                Source{SourceKind::WeightCount, meta.id, {}});
        } else if (meta.kind == ComponentKind::Deformer) {
            const auto* deformer = components_.deformer(meta.id);
            if (deformer == nullptr || deformer->inverse_binds.count() == 0) {
                return;
            }
            const auto binding = orlrig::scene_inverse_bindings_binding(meta.name);
            add_buffer_descriptor(
                orlgraph::StableId{binding},
                "Deformer " + meta.name + " Inverse Binds",
                orlgraph::LogicalType::matrix(),
                orlgraph::Domain::joint(),
                orlgraph::Shape::one("joint_count"),
                "inverse_binds", "joint", orlrig::kMatrixStride,
                std::string{orlrig::kSceneJointCountBinding},
                Source{SourceKind::InverseBinds, meta.id, {}});
        } else if (meta.kind == ComponentKind::Joint) {
            add_handle_descriptor(
                orlgraph::StableId{
                    scene_joint_handle_binding(meta.name)},
                "Joint " + meta.name + " Handle",
                "orlrig::joint_handle", "scene.joint.handle",
                Source{SourceKind::JointHandle, meta.id, {}});
        } else if (meta.kind == ComponentKind::Locator) {
            const auto binding = orlrig::scene_locator_xform_binding(meta.name);
            if (!pack_locator(meta.id, binding)) {
                return;
            }
            add_buffer_descriptor(
                orlgraph::StableId{binding},
                "Locator " + meta.name + " Transform",
                orlgraph::LogicalType::struct_type("Locator"),
                orlgraph::Domain::rig(),
                orlgraph::Shape::one("one"),
                "locator.xform", "world", orlrig::kLocatorStride,
                orlrig::scene_locator_count_binding(meta.name),
                Source{SourceKind::LocatorXform, meta.id, {}});
            add_scalar_descriptor(
                orlgraph::StableId{
                    orlrig::scene_locator_count_binding(meta.name)},
                "Locator " + meta.name + " Count",
                "locator_count",
                Source{SourceKind::LocatorCount, meta.id, {}});
            add_handle_descriptor(
                orlgraph::StableId{
                    scene_locator_handle_binding(meta.name)},
                "Locator " + meta.name + " Handle",
                "orlrig::locator_handle", "scene.locator.handle",
                Source{SourceKind::LocatorHandle, meta.id, {}});
        } else if (meta.kind == ComponentKind::Controller) {
            const auto binding = orlrig::scene_controller_xform_binding(meta.name);
            if (!pack_controller(meta.id, binding)) {
                return;
            }
            add_buffer_descriptor(
                orlgraph::StableId{binding},
                "Controller " + meta.name + " Transform",
                orlgraph::LogicalType::matrix(),
                orlgraph::Domain::rig(),
                orlgraph::Shape::one("controller_count"),
                "controller.xform", "world", orlrig::kMatrixStride,
                orlrig::scene_controller_count_binding(meta.name),
                Source{SourceKind::ControllerXform, meta.id, {}});
            add_scalar_descriptor(
                orlgraph::StableId{
                    orlrig::scene_controller_count_binding(meta.name)},
                "Controller " + meta.name + " Count",
                "controller_count",
                Source{SourceKind::ControllerCount, meta.id, {}});
        }
    });

    std::sort(descriptors_.begin(), descriptors_.end(),
        [](const SceneInputDescriptor& left, const SceneInputDescriptor& right) {
            return left.id.value < right.id.value;
        });
}

void SceneInputCatalog::add_buffer_descriptor(orlgraph::StableId id,
    std::string label, orlgraph::LogicalType element,
    orlgraph::Domain domain, orlgraph::Shape shape,
    std::string semantic, std::string coordinate_space,
    std::size_t element_stride, std::string count_binding, Source source)
{
    orlgraph::InterfacePort port;
    port.id = id;
    port.name = label;
    port.direction = orlgraph::PortDirection::Input;
    port.type = orlgraph::LogicalType::buffer(std::move(element));
    port.domain = std::move(domain);
    port.shape = std::move(shape);
    port.required = true;
    port.binding = id.value;
    port.semantic = std::move(semantic);
    port.coordinate_space = std::move(coordinate_space);
    SceneInputDescriptor descriptor;
    descriptor.id = id;
    descriptor.label = label;
    descriptor.port = port;
    descriptor.kind = exec::ParameterKind::Buffer;
    descriptor.element_stride = element_stride;
    descriptor.count_binding = std::move(count_binding);
    descriptors_.push_back(std::move(descriptor));
    sources_[port.binding] = std::move(source);
}

void SceneInputCatalog::add_scalar_descriptor(orlgraph::StableId id,
    std::string label, std::string semantic, Source source)
{
    orlgraph::InterfacePort port;
    port.id = id;
    port.name = label;
    port.direction = orlgraph::PortDirection::Input;
    port.type = orlgraph::LogicalType::int64();
    port.domain = orlgraph::Domain::constant();
    port.shape = orlgraph::Shape::scalar();
    port.required = true;
    port.binding = id.value;
    port.semantic = std::move(semantic);
    SceneInputDescriptor descriptor;
    descriptor.id = id;
    descriptor.label = label;
    descriptor.port = port;
    descriptor.kind = exec::ParameterKind::Int64;
    descriptor.element_stride = sizeof(std::int64_t);
    descriptors_.push_back(std::move(descriptor));
    sources_[port.binding] = std::move(source);
}

void SceneInputCatalog::add_handle_descriptor(
    orlgraph::StableId id, std::string label,
    std::string canonical_type, std::string semantic, Source source)
{
    orlgraph::InterfacePort port;
    port.id = id;
    port.name = std::move(label);
    port.direction = orlgraph::PortDirection::Input;
    port.type = orlgraph::LogicalType::handle(canonical_type);
    port.domain = orlgraph::Domain::rig();
    port.shape = orlgraph::Shape::scalar();
    port.required = true;
    port.binding = id.value;
    port.semantic = std::move(semantic);
    SceneInputDescriptor descriptor;
    descriptor.id = id;
    descriptor.label = port.name;
    descriptor.port = port;
    descriptor.kind = exec::ParameterKind::Handle;
    descriptor.canonical_handle_type = std::move(canonical_type);
    descriptors_.push_back(std::move(descriptor));
    sources_[port.binding] = std::move(source);
}

bool SceneInputCatalog::pack_mesh_positions(const std::string& object_name,
    const std::string& binding)
{
    if (scene_.drawable_mgr == nullptr) {
        return false;
    }
    const auto* object = scene_.find_object(object_name);
    if (object == nullptr) {
        return false;
    }
    const auto* mesh = scene_.drawable_mgr->find_mesh(object->mesh_name);
    if (mesh == nullptr || mesh->vbuf == nullptr || mesh->vcnt == 0) {
        return false;
    }
    const int offset = vertex_float_offset(*mesh);
    if (offset < 0) {
        return false;
    }

    auto [found, inserted] = mesh_positions_.try_emplace(
        binding, orlrig::kPointOrlType, orlrig::kPointStride);
    (void)inserted;
    auto& positions = found->second;
    if (!positions.resize(mesh->vcnt)) {
        return false;
    }
    auto* destination = static_cast<double*>(positions.data());
    for (std::uint32_t vertex = 0; vertex < mesh->vcnt; ++vertex) {
        const float* source = mesh->vbuf + vertex * mesh->comp_size + offset;
        const glm::vec4 world = object->model * glm::vec4{
            source[0], source[1], source[2], 1.0f};
        destination[vertex * 4 + 0] = world.x;
        destination[vertex * 4 + 1] = world.y;
        destination[vertex * 4 + 2] = world.z;
        destination[vertex * 4 + 3] = 0.0;
    }
    return true;
}

bool SceneInputCatalog::prepare_packed_inputs()
{
    packed_solver_inputs.ready = false;
    constexpr std::size_t alignment = 16;
    const std::size_t joint_bytes = joints_.byte_size();
    const std::size_t locator_bytes = locators_.byte_size();
    const std::size_t controller_bytes = controllers_.byte_size();

    std::size_t offset = sizeof(orlrig::SolverContext);
    offset = align_up(offset, alignment);
    if (offset == std::numeric_limits<std::size_t>::max()
        || joint_bytes > std::numeric_limits<std::size_t>::max() - offset)
    {
        return false;
    }
    const std::size_t joints_offset = offset;
    offset += joint_bytes;

    offset = align_up(offset, alignment);
    if (offset == std::numeric_limits<std::size_t>::max()
        || locator_bytes > std::numeric_limits<std::size_t>::max() - offset)
    {
        return false;
    }
    const std::size_t locators_offset = offset;
    offset += locator_bytes;

    offset = align_up(offset, alignment);
    if (offset == std::numeric_limits<std::size_t>::max()
        || controller_bytes > std::numeric_limits<std::size_t>::max() - offset)
    {
        return false;
    }
    const std::size_t controllers_offset = offset;
    offset += controller_bytes;

    const bool layout_changed =
        packed_solver_inputs.storage.size() != offset
        || packed_solver_inputs.joints_offset != joints_offset
        || packed_solver_inputs.locators_offset != locators_offset
        || packed_solver_inputs.controllers_offset != controllers_offset
        || packed_solver_inputs.joint_ids != joint_ids_
        || packed_solver_inputs.locator_ids != locator_ids_
        || packed_solver_inputs.controller_ids != controller_ids_;

    if (layout_changed) {
        packed_solver_inputs.storage.resize(offset);
        packed_solver_inputs.joint_ids = joint_ids_;
        packed_solver_inputs.locator_ids = locator_ids_;
        packed_solver_inputs.controller_ids = controller_ids_;
        packed_solver_inputs.joints_offset = joints_offset;
        packed_solver_inputs.locators_offset = locators_offset;
        packed_solver_inputs.controllers_offset = controllers_offset;
    }

    if (packed_solver_inputs.storage.size() < sizeof(orlrig::SolverContext)) {
        return false;
    }
    const orlrig::SolverContext context{
        static_cast<std::int64_t>(joint_ids_.size()),
        static_cast<std::int64_t>(controller_ids_.size()),
        static_cast<std::int64_t>(locator_ids_.size()),
        static_cast<std::int64_t>(joints_offset),
        static_cast<std::int64_t>(controllers_offset),
        static_cast<std::int64_t>(locators_offset)};
    std::memcpy(
        packed_solver_inputs.storage.data(),
        &context,
        sizeof(context));

    const auto copy_buffer = [](
        const exec::OrlBuffer& source,
        std::vector<std::byte>& destination,
        std::size_t destination_offset)
    {
        if (source.byte_size() == 0) {
            return;
        }
        std::memcpy(
            destination.data() + destination_offset,
            source.data(),
            source.byte_size());
    };
    copy_buffer(joints_, packed_solver_inputs.storage, joints_offset);
    copy_buffer(locators_, packed_solver_inputs.storage, locators_offset);
    copy_buffer(
        controllers_,
        packed_solver_inputs.storage,
        controllers_offset);

    ++packed_solver_inputs.version;
    packed_solver_inputs.ready = true;
    return true;
}

exec::PackedBufferView SceneInputCatalog::packed_view(
    std::size_t offset, std::size_t bytes)
{
    return exec::PackedBufferView{
        packed_solver_inputs.storage.data(),
        packed_solver_inputs.storage.size(),
        offset,
        bytes,
        packed_solver_inputs.version};
}

bool SceneInputCatalog::pack_joints() {
    const auto previous_ids = joint_ids_;
    joint_ids_ = components_.packed_joint_ids();
    packed_joint_ids_ = joint_ids_;
    if (joint_ids_ != previous_ids) {
        ++revision_;
        clear_computed_joints_device();
    }
    const auto packed = components_.packed_joints();
    if (!joints_.resize(packed.size())) {
        return false;
    }
    if (packed.empty()) {
        return true;
    }
    const std::size_t bytes = packed.size() * orlrig::kJointStride;
    if (joints_.count() == packed.size()
        && std::memcmp(
            static_cast<const exec::OrlBuffer&>(joints_).data(),
            packed.data(), bytes) == 0)
    {
        return true;
    }
    std::memcpy(joints_.data(), packed.data(), bytes);
    return true;
}

bool SceneInputCatalog::pack_locators() {
    const auto previous_ids = locator_ids_;
    locator_ids_ = components_.packed_locator_ids();
    if (locator_ids_ != previous_ids) {
        ++revision_;
    }
    const auto packed = components_.packed_locators();
    if (!locators_.resize(packed.size())) {
        return false;
    }
    std::vector<double> values(
        packed.size() * (orlrig::kLocatorStride / sizeof(double)));
    for (std::size_t index = 0; index < packed.size(); ++index) {
        orlrig::pack_xform(packed[index],
            values.data() + index
                * (orlrig::kLocatorStride / sizeof(double)));
    }
    const std::size_t bytes = values.size() * sizeof(double);
    if (bytes == 0
        || (locators_.count() == packed.size()
            && std::memcmp(
                static_cast<const exec::OrlBuffer&>(locators_).data(),
                values.data(), bytes) == 0))
    {
        return true;
    }
    std::memcpy(locators_.data(), values.data(), bytes);
    return true;
}

bool SceneInputCatalog::pack_controllers() {
    std::vector<std::pair<std::string, ComponentId>> controllers;
    components_.for_each([&controllers](const Component& component) {
        if (component.kind == ComponentKind::Controller) {
            controllers.emplace_back(component.name, component.id);
        }
    });
    std::sort(controllers.begin(), controllers.end(),
        [](const auto& left, const auto& right) {
            return left.first < right.first;
        });
    const auto previous_ids = controller_ids_;
    controller_ids_.clear();
    controller_ids_.reserve(controllers.size());
    for (const auto& [_, id] : controllers) {
        controller_ids_.push_back(id);
    }
    if (controller_ids_ != previous_ids) {
        ++revision_;
    }

    if (!controllers_.resize(controllers.size())) {
        return false;
    }
    std::vector<double> values(controllers.size() * 16);
    for (std::size_t index = 0; index < controllers.size(); ++index) {
        const auto* controller = components_.controller(controllers[index].second);
        if (controller == nullptr) {
            return false;
        }
        auto resolved = *controller;
        resolved.xform = components_.controller_world_xform(
            controllers[index].second);
        orlrig::pack_xform(resolved, values.data() + index * 16);
    }
    const std::size_t bytes = values.size() * sizeof(double);
    if (bytes == 0
        || (controllers_.count() == controllers.size()
            && std::memcmp(
                static_cast<const exec::OrlBuffer&>(controllers_).data(),
                values.data(), bytes) == 0))
    {
        return true;
    }
    std::memcpy(controllers_.data(), values.data(), bytes);
    return true;
}

bool SceneInputCatalog::pack_locator(ComponentId id,
    const std::string& binding)
{
    const auto* locator = components_.locator(id);
    if (locator == nullptr) {
        return false;
    }
    auto [found, inserted] = locator_xforms_.try_emplace(
        binding, orlrig::kLocatorOrlType, orlrig::kLocatorStride);
    (void)inserted;
    auto& buffer = found->second;
    if (!buffer.resize(1)) {
        return false;
    }
    orlrig::pack_xform(*locator,
        static_cast<double*>(buffer.data()));
    return true;
}

bool SceneInputCatalog::pack_controller(ComponentId id,
    const std::string& binding)
{
    const auto* controller = components_.controller(id);
    if (controller == nullptr) {
        return false;
    }
    auto [found, inserted] = controller_xforms_.try_emplace(
        binding, orlrig::kMatrixOrlType, orlrig::kMatrixStride);
    (void)inserted;
    auto& buffer = found->second;
    if (!buffer.resize(1)) {
        return false;
    }
    auto resolved = *controller;
    resolved.xform = components_.controller_world_xform(id);
    orlrig::pack_xform(resolved,
        static_cast<double*>(buffer.data()));
    return true;
}

bool SceneInputCatalog::set_error(std::string* error,
    std::string message) const
{
    if (error != nullptr) {
        *error = std::move(message);
    }
    return false;
}

} // namespace ORL
