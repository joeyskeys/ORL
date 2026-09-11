#include "vp/deformer_feature.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include "comps/joint.hpp"
#include "concepts/mesh.h"
#include "runtime_config.hpp"
#include "vk_ins/types.h"
#include "asset_mgr/drawable_mgr.h"

#include <glm/gtc/matrix_inverse.hpp>
#include <glm/mat4x4.hpp>
#include <glm/vec4.hpp>

namespace ORL
{
namespace
{

exec::Backend backend_from_config() {
    return runtime_config.device == ComputeDevice::Gpu
        ? exec::Backend::Cuda
        : exec::Backend::Cpu;
}

bool known_type(std::string_view name) {
    return name == "lbs";
}

void print_runner_errors(const std::vector<std::string>& errors) {
    for (const auto& error : errors) {
        std::cerr << "Deformer: " << error << '\n';
    }
}

int vertex_float_offset(const vkkk::Mesh& mesh) {
    int packed = 0;
    for (const auto component : mesh.comps) {
        if (component == vkkk::VERTEX) {
            return packed;
        }
        packed += static_cast<int>(vkkk::comp_sizes[component]);
    }
    return -1;
}

bool extract_mesh(orlrig::MeshData& destination,
    const vkkk::Mesh& source,
    const glm::mat4& model)
{
    const int offset = vertex_float_offset(source);
    if (offset < 0 || source.vbuf == nullptr || source.vcnt == 0) {
        return false;
    }
    destination.positions.resize(source.vcnt);
    destination.model = model;
    for (std::uint32_t vertex = 0; vertex < source.vcnt; ++vertex) {
        const float* source_position =
            source.vbuf + vertex * source.comp_size + offset;
        destination.positions[vertex] = {
            source_position[0], source_position[1], source_position[2]};
    }
    if (source.ibuf != nullptr && source.icnt != 0) {
        destination.indices.assign(source.ibuf, source.ibuf + source.icnt * 3);
    }
    return true;
}

bool write_positions(vkkk::Mesh& mesh,
    const exec::OrlBuffer& positions,
    const glm::mat4& bind_model)
{
    const int offset = vertex_float_offset(mesh);
    if (offset < 0 || mesh.vbuf == nullptr || positions.count() < mesh.vcnt) {
        return false;
    }
    const glm::mat4 to_object = glm::inverse(bind_model);
    const auto* source = static_cast<const double*>(positions.data());
    for (std::uint32_t vertex = 0; vertex < mesh.vcnt; ++vertex) {
        float* destination = mesh.vbuf + vertex * mesh.comp_size + offset;
        const glm::vec4 world{
            static_cast<float>(source[vertex * 4 + 0]),
            static_cast<float>(source[vertex * 4 + 1]),
            static_cast<float>(source[vertex * 4 + 2]),
            1.0f};
        const glm::vec4 local = to_object * world;
        destination[0] = local.x;
        destination[1] = local.y;
        destination[2] = local.z;
    }
    return true;
}

void dump_bind_snapshot(const std::vector<orlviewer::Joint>& joints,
    const exec::OrlBuffer& inverse_binds)
{
    const auto* matrices = static_cast<const double*>(inverse_binds.data());
    std::ofstream output("orl_debug_bind.txt", std::ios::trunc);
    if (!output) {
        std::cerr << "Deformer: could not write orl_debug_bind.txt\n";
        return;
    }
    output << "# joint parent selected tx ty tz cpp_world_xyz inv_row_tx ty tz\n";
    std::cout << "Deformer: bind snapshot\n";
    for (std::size_t index = 0; index < joints.size(); ++index) {
        const auto& joint = joints[index];
        const glm::vec3 world{
            orlviewer::joint_world_matrix(joints,
                static_cast<std::int64_t>(index))[3]};
        double inverse_tx = 0.0;
        double inverse_ty = 0.0;
        double inverse_tz = 0.0;
        if (matrices != nullptr && index < inverse_binds.count()) {
            const auto* matrix = matrices + index * 16;
            inverse_tx = matrix[3];
            inverse_ty = matrix[7];
            inverse_tz = matrix[11];
        }
        output << index << '\t' << joint.parent << '\t' << joint.selected << '\t'
            << joint.translation[0] << '\t' << joint.translation[1] << '\t'
            << joint.translation[2] << '\t' << world.x << '\t' << world.y << '\t'
            << world.z << '\t' << inverse_tx << '\t' << inverse_ty << '\t'
            << inverse_tz << '\n';
    }
    std::cout << "Deformer: wrote orl_debug_bind.txt\n";
}

double max_position_delta(const exec::OrlBuffer& bind,
    const exec::OrlBuffer& posed)
{
    const auto count = std::min(bind.count(), posed.count());
    const auto* first = static_cast<const double*>(bind.data());
    const auto* second = static_cast<const double*>(posed.data());
    if (first == nullptr || second == nullptr || count == 0) {
        return 0.0;
    }
    double maximum = 0.0;
    for (std::size_t vertex = 0; vertex < count; ++vertex) {
        const double dx = first[vertex * 4 + 0] - second[vertex * 4 + 0];
        const double dy = first[vertex * 4 + 1] - second[vertex * 4 + 1];
        const double dz = first[vertex * 4 + 2] - second[vertex * 4 + 2];
        maximum = std::max(maximum, std::sqrt(dx * dx + dy * dy + dz * dz));
    }
    return maximum;
}

void dump_posed(const exec::OrlBuffer& bind,
    const exec::OrlBuffer& posed,
    const vkkk::Mesh& mesh)
{
    const auto count = std::min(bind.count(), posed.count());
    const auto* first = static_cast<const double*>(bind.data());
    const auto* second = static_cast<const double*>(posed.data());
    const int offset = vertex_float_offset(mesh);
    std::ofstream output("orl_debug_posed.txt", std::ios::trunc);
    if (!output || first == nullptr || second == nullptr) {
        std::cerr << "Deformer: could not write orl_debug_posed.txt\n";
        return;
    }
    output << "# vertex\tbind_xyz\tposed_xyz\tcpu_vbuf_xyz\n";
    const std::size_t preview = std::min<std::size_t>(count, 8);
    for (std::size_t vertex = 0; vertex < preview; ++vertex) {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
        if (mesh.vbuf != nullptr && offset >= 0) {
            const float* source = mesh.vbuf + vertex * mesh.comp_size + offset;
            x = source[0];
            y = source[1];
            z = source[2];
        }
        output << vertex << '\t'
            << first[vertex * 4 + 0] << '\t' << first[vertex * 4 + 1] << '\t'
            << first[vertex * 4 + 2] << '\t' << second[vertex * 4 + 0] << '\t'
            << second[vertex * 4 + 1] << '\t' << second[vertex * 4 + 2] << '\t'
            << x << '\t' << y << '\t' << z << '\n';
    }
    std::cout << "Deformer: wrote orl_debug_posed.txt\n";
}

} // namespace

DeformerFeature::DeformerFeature(vkkk::Scene& scene, ComponentManager& components,
    ComponentId deformer_id, ComponentId weight_id, const Selection& selection)
    : scene(scene)
    , components(components)
    , deformer_id(deformer_id)
    , weight_id(weight_id)
    , selection(selection)
    , runner(backend_from_config())
{
}

bool DeformerFeature::set_type(std::string_view name) {
    if (!known_type(name)) {
        std::cerr << "Deformer: unknown type '" << name << "'\n";
        return false;
    }
    type_name = std::string{name};
    if (auto* deformer = components.deformer(deformer_id)) {
        deformer->type = type_name;
        deformer->bound = false;
    }
    return true;
}

void DeformerFeature::set_mesh(std::string name) {
    if (auto* deformer = components.deformer(deformer_id)) {
        deformer->mesh_name = std::move(name);
        deformer->bound = false;
    }
}

void DeformerFeature::request() {
    if (!selection.valid_for_bind()) {
        std::cerr << "Deformer: select a mesh and joints first\n";
        return;
    }
    pending = true;
}

void DeformerFeature::unbind() {
    pending = false;
    if (auto* deformer = components.deformer(deformer_id)) {
        deformer->bound = false;
        deformer->mesh_name.clear();
        deformer->bind_positions.clear();
        deformer->inverse_binds.clear();
        deformer->bind_model = glm::mat4{1.0f};
    }
}

void DeformerFeature::on_update(vkkk::Context& context, const vkkk::Context::Frame&) {
    if (pending) {
        pending = false;
        setup(context);
    }
    if (const auto* deformer = components.deformer(deformer_id);
        deformer != nullptr && deformer->bound)
    {
        evaluate(context);
    }
}

bool DeformerFeature::setup(vkkk::Context& context) {
    auto* deformer = components.deformer(deformer_id);
    const auto* weight = components.weight(weight_id);
    if (deformer == nullptr || weight == nullptr) {
        std::cerr << "Deformer: missing deformer or weight component\n";
        return false;
    }

    std::string mesh_name = selection.selected_mesh_name();
    if (mesh_name.empty()) {
        mesh_name = deformer->mesh_name;
    }
    if (mesh_name.empty() || !selection.has_selected_joint()) {
        std::cerr << "Deformer: select a mesh and joints first\n";
        return false;
    }
    if (scene.drawable_mgr == nullptr) {
        std::cerr << "Deformer: no mesh loaded\n";
        return false;
    }
    const auto* mesh = scene.drawable_mgr->find_mesh(mesh_name);
    if (mesh == nullptr) {
        std::cerr << "Deformer: mesh '" << mesh_name << "' not found\n";
        return false;
    }
    const auto packed = components.packed_joints();
    if (packed.empty()) {
        std::cerr << "Deformer: no joints\n";
        return false;
    }
    if (weight->weights.count() == 0) {
        std::cerr << "Deformer: skipped, auto-weight did not fill weight buffers\n";
        return false;
    }
    if (runner.backend() == exec::Backend::Cuda
        && !context.make_mesh_deformable(mesh_name, *mesh))
    {
        std::cerr << "Deformer: failed to create rest/draw GPU buffers for '"
            << mesh_name << "'\n";
        return false;
    }

    orlrig::MeshData input;
    if (!extract_mesh(input, *mesh, selection.selected_mesh_model())) {
        std::cerr << "Deformer: failed to pack bind pose\n";
        return false;
    }
    const auto status = runner.capture_bind(*deformer, input, packed);
    if (!status) {
        print_runner_errors(status.errors);
        return false;
    }
    deformer->mesh_name = mesh_name;
    deformer->type = type_name;
    logged_rest = false;
    logged_move = false;
    dump_bind_snapshot(packed, deformer->inverse_binds);
    std::cout << "Deformer: setup " << type_name << " '" << mesh_name << "' "
        << mesh->vcnt << " verts, " << packed.size() << " joints\n";
    return true;
}

bool DeformerFeature::evaluate(vkkk::Context& context) {
    auto* deformer = components.deformer(deformer_id);
    auto* weight = components.weight(weight_id);
    if (deformer == nullptr || weight == nullptr || !deformer->bound) {
        return false;
    }
    const std::string mesh_name = deformer->mesh_name;
    if (mesh_name.empty() || scene.drawable_mgr == nullptr) {
        return false;
    }
    auto* mesh = scene.drawable_mgr->find_mesh(mesh_name);
    if (mesh == nullptr) {
        return false;
    }
    const auto packed = components.packed_joints();
    if (packed.empty()
        || packed.size() != deformer->inverse_binds.count())
    {
        return false;
    }

    const bool device_only = runner.backend() == exec::Backend::Cuda;
    const auto status = runner.evaluate(*deformer, *weight, packed, device_only);
    if (!status) {
        print_runner_errors(status.errors);
        deformer->bound = false;
        return false;
    }

    if (device_only) {
        const auto output_device = runner.output_device();
        if (!output_device.has_value()) {
            std::cerr << "Deformer: CUDA output buffer is unavailable\n";
            deformer->bound = false;
            return false;
        }
        const glm::mat4 to_object = glm::inverse(deformer->bind_model);
        float matrix_float[16] = {};
        std::memcpy(matrix_float, &to_object, sizeof(matrix_float));
        double world_to_object[16] = {};
        for (int index = 0; index < 16; ++index) {
            world_to_object[index] = static_cast<double>(matrix_float[index]);
        }
        const int offset = vertex_float_offset(*mesh);
        if (offset < 0 || !context.write_mesh_positions_from_cuda(
                mesh_name, output_device->device_ptr, output_device->bytes,
                mesh->vcnt, mesh->comp_size,
                static_cast<std::uint32_t>(offset), world_to_object))
        {
            std::cerr << "Deformer: CUDA-Vulkan mesh update failed for '"
                << mesh_name << "'\n";
            deformer->bound = false;
            return false;
        }
        if (!logged_rest) {
            std::cout << "Deformer: GPU-resident output committed to mesh\n";
            logged_rest = true;
        }
        return true;
    }

    const auto& output = runner.output_positions();
    if (!write_positions(*mesh, output, deformer->bind_model)) {
        std::cerr << "Deformer: failed to write mesh positions\n";
        return false;
    }
    const double delta = max_position_delta(deformer->bind_positions, output);
    if (!logged_rest) {
        std::cout << "Deformer: rest evaluate max |posed-bind|=" << delta << '\n';
        logged_rest = true;
    } else if (!logged_move && delta > 1.0e-3) {
        std::cout << "Deformer: mesh moved, max |posed-bind|=" << delta << '\n';
        dump_posed(deformer->bind_positions, output, *mesh);
        logged_move = true;
    }
    if (!context.update_mesh(mesh_name, *mesh)) {
        std::cerr << "Deformer: GPU mesh update failed for '" << mesh_name << "'\n";
        return false;
    }
    return true;
}

} // namespace ORL
