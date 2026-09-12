#include "graph_scene_inputs.hpp"

#include <algorithm>
#include <cstring>
#include <utility>

#include <glm/vec4.hpp>

#include "asset_mgr/drawable_mgr.h"
#include "concepts/mesh.h"
#include "orlrig/abi.hpp"
#include "orlrig/controller.hpp"
#include "orlrig/graph_resources.hpp"

namespace ORL
{
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

} // namespace

SceneInputCatalog::SceneInputCatalog(vkkk::Scene& scene,
    ComponentManager& components)
    : scene_(scene)
    , components_(components)
    , joints_(orlrig::kJointOrlType, orlrig::kJointStride)
{
    refresh();
}

void SceneInputCatalog::refresh() {
    descriptors_.clear();
    sources_.clear();
    mesh_positions_.clear();
    controller_xforms_.clear();
    pack_joints();
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
    components_.for_each([&](const Component& component) {
        const bool matches = kind == SceneElementKind::Joint
            ? component.kind == ComponentKind::Joint
            : component.kind == ComponentKind::Controller;
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
    const auto* component = components_.find(name);
    if (component == nullptr) {
        return std::nullopt;
    }
    if (kind == SceneElementKind::Joint) {
        if (component->kind != ComponentKind::Joint) {
            return std::nullopt;
        }
        const auto index = components_.joint_index(component->id);
        return index < 0 ? std::nullopt
                        : std::optional<std::int64_t>{index};
    }
    if (component->kind != ComponentKind::Controller) {
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

bool SceneInputCatalog::resolve(const orlgraph::InterfacePort& port,
    exec::GraphInputBinding& binding, std::string* error)
{
    binding = {};
    const std::string key = port.binding.empty() ? port.id.value : port.binding;
    const auto descriptor = std::find_if(descriptors_.begin(), descriptors_.end(),
        [&key](const SceneInputDescriptor& candidate) {
            return candidate.port.binding == key;
        });
    if (descriptor != descriptors_.end()) {
        if (descriptor->port.type != port.type
            || descriptor->port.domain != port.domain
            || descriptor->port.shape != port.shape)
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
        binding.kind = exec::ParameterKind::Buffer;
        binding.buffer = &joints_;
        binding.element_count = joints_.count();
        return true;
    case SourceKind::JointCount:
        binding.kind = exec::ParameterKind::Int64;
        binding.int_value = static_cast<std::int64_t>(
            components_.packed_joints().size());
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

bool SceneInputCatalog::bind_graph_inputs(exec::OrlGraphExecution& execution,
    const orlgraph::GraphModule& module)
{
    refresh();
    return execution.bind_graph_inputs(module,
        [this](const orlgraph::InterfacePort& port,
            exec::GraphInputBinding& binding, std::string& error) {
            return resolve(port, binding, &error);
        });
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
    descriptors_.push_back(SceneInputDescriptor{
        id, label, port, exec::ParameterKind::Buffer, element_stride,
        std::move(count_binding)});
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
    descriptors_.push_back(SceneInputDescriptor{
        id, label, port, exec::ParameterKind::Int64, sizeof(std::int64_t)});
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

bool SceneInputCatalog::pack_joints() {
    const auto packed = components_.packed_joints();
    if (!joints_.resize(packed.size())) {
        return false;
    }
    if (!packed.empty()) {
        std::memcpy(joints_.data(), packed.data(),
            packed.size() * orlrig::kJointStride);
    }
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
    orlrig::pack_xform(*controller,
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
