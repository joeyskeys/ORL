#include "vp/auto_weight_feature.hpp"

#include <algorithm>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>

#include <glm/mat4x4.hpp>

#include "comps/joint.hpp"
#include "concepts/mesh.h"
#include "runtime_config.hpp"
#include "asset_mgr/drawable_mgr.h"

namespace ORL
{
namespace
{

exec::Backend backend_from_config() {
    return runtime_config.device == ComputeDevice::Gpu
        ? exec::Backend::Cuda
        : exec::Backend::Cpu;
}

void print_runner_errors(const std::vector<std::string>& errors) {
    for (const auto& error : errors) {
        std::cerr << "Auto weight: " << error << '\n';
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

void dump_weights(const WeightData& weight,
    std::size_t vertex_count,
    std::size_t joint_count)
{
    const auto weight_count = std::max<std::int64_t>(1, weight.weight_cnt);
    const auto output_count = weight.weights.count();
    const auto* cells = static_cast<const orlviewer::Weight*>(
        weight.weights.data());
    if (cells == nullptr || output_count == 0) {
        std::cerr << "Auto weight: weight buffers are empty, skip dump\n";
        return;
    }

    std::vector<std::size_t> counts(joint_count, 0);
    std::size_t invalid = 0;
    for (std::size_t index = 0; index < output_count; ++index) {
        const auto joint = cells[index].joint;
        if (joint < 0 || static_cast<std::size_t>(joint) >= joint_count) {
            ++invalid;
            continue;
        }
        ++counts[static_cast<std::size_t>(joint)];
    }
    std::cout << "Auto weight: dump weight_cnt=" << weight_count
        << " slots=" << output_count << " invalid=" << invalid << '\n';
    for (std::size_t joint = 0; joint < counts.size(); ++joint) {
        std::cout << "  bone " << joint << ": " << counts[joint]
            << " influences\n";
    }

    const std::size_t preview = std::min<std::size_t>(output_count, 8);
    for (std::size_t index = 0; index < preview; ++index) {
        const auto vertex = index / static_cast<std::size_t>(weight_count);
        const auto slot = index % static_cast<std::size_t>(weight_count);
        std::cout << "  vert " << vertex << " slot " << slot
            << " joint=" << cells[index].joint
            << " w=" << cells[index].weight << '\n';
    }

    std::ofstream output("orl_debug_weights.txt", std::ios::trunc);
    if (!output) {
        std::cerr << "Auto weight: could not write orl_debug_weights.txt\n";
        return;
    }
    output << "# vertex\tslot\tjoint\tweight\n";
    for (std::size_t index = 0; index < output_count; ++index) {
        const auto vertex = vertex_count == 0
            ? index
            : index / static_cast<std::size_t>(weight_count);
        const auto slot = vertex_count == 0
            ? index
            : index % static_cast<std::size_t>(weight_count);
        output << vertex << '\t' << slot << '\t' << cells[index].joint
            << '\t' << cells[index].weight << '\n';
    }
    std::cout << "Auto weight: wrote orl_debug_weights.txt ("
        << output_count << " rows)\n";
}

} // namespace

AutoWeightFeature::AutoWeightFeature(vkkk::Scene& scene,
    ComponentManager& components,
    ComponentId weight_id,
    const Selection& selection)
    : scene(scene)
    , components(components)
    , weight_id(weight_id)
    , selection(selection)
    , runner("closest_distance", backend_from_config())
{
}

void AutoWeightFeature::set_csr(MeshCsrFeature& feature) {
    csr = &feature;
}

bool AutoWeightFeature::set_algorithm(std::string_view name) {
    if (!runner.set_algorithm(name)) {
        std::cerr << "Auto weight: unknown algorithm '" << name << "'\n";
        return false;
    }
    algorithm_name = std::string{name};
    return true;
}

bool AutoWeightFeature::cycle_algorithm() {
    static constexpr const char* algorithms[] = {
        "closest_distance",
        "closest_hierarchy",
        "heat",
        "geodesic",
    };
    constexpr int count = static_cast<int>(std::size(algorithms));
    int index = 0;
    for (; index < count; ++index) {
        if (algorithm_name == algorithms[index]) {
            break;
        }
    }
    index = (index + 1) % count;
    const bool changed = set_algorithm(algorithms[index]);
    if (changed) {
        std::cout << "Auto weight: algorithm '" << algorithm_name << "'\n";
    }
    return changed;
}

void AutoWeightFeature::request() {
    if (!selection.valid_for_bind()) {
        std::cerr << "Auto weight: select a mesh and joints first\n";
        return;
    }
    pending = true;
}

void AutoWeightFeature::on_update(vkkk::Context& context,
    const vkkk::Context::Frame&)
{
    if (!pending) {
        return;
    }
    pending = false;
    run(context);
}

bool AutoWeightFeature::run(vkkk::Context& context) {
    WeightData* weight = components.weight(weight_id);
    if (weight == nullptr) {
        std::cerr << "Auto weight: missing weight component\n";
        return false;
    }
    const std::string mesh_name = selection.selected_mesh_name();
    if (mesh_name.empty() || !selection.has_selected_joint()) {
        std::cerr << "Auto weight: select a mesh and joints first\n";
        return false;
    }
    if (scene.drawable_mgr == nullptr) {
        std::cerr << "Auto weight: no mesh loaded\n";
        return false;
    }
    const auto* mesh = scene.drawable_mgr->find_mesh(mesh_name);
    if (mesh == nullptr) {
        std::cerr << "Auto weight: mesh '" << mesh_name << "' not found\n";
        return false;
    }

    const auto packed = components.packed_joints();
    if (packed.empty()) {
        std::cerr << "Auto weight: no joints\n";
        return false;
    }
    orlrig::MeshData input;
    if (!extract_mesh(input, *mesh, selection.selected_mesh_model())) {
        std::cerr << "Auto weight: failed to pack mesh\n";
        return false;
    }

    orlrig::MeshCsrData core_csr;
    const orlrig::MeshCsrData* csr_data = nullptr;
    if (csr != nullptr) {
        if (!csr->build(context, mesh_name, *mesh)) {
            std::cerr << "Auto weight: CSR build failed for '" << mesh_name
                << "'\n";
        }
        if (const auto* built = csr->find(mesh_name); built != nullptr) {
            core_csr.offsets = built->offsets;
            core_csr.neighbors = built->neighbors;
            csr_data = &core_csr;
        }
    }

    const auto status = runner.run(input, packed, csr_data, *weight, dropoff);
    if (!status) {
        print_runner_errors(status.errors);
        return false;
    }

    const std::size_t output_count = weight->weights.count();
    std::size_t used_joints = 0;
    if (const auto* cells = static_cast<const orlviewer::Weight*>(
            weight->weights.data()))
    {
        std::vector<char> used(packed.size(), 0);
        for (std::size_t index = 0; index < output_count; ++index) {
            const auto joint = cells[index].joint;
            if (joint < 0 || static_cast<std::size_t>(joint) >= packed.size()) {
                continue;
            }
            auto& flag = used[static_cast<std::size_t>(joint)];
            if (flag == 0) {
                flag = 1;
                ++used_joints;
            }
        }
    }

    std::cout << "Auto weight: " << algorithm_name << " '" << mesh_name
        << "' " << mesh->vcnt << " verts, " << packed.size()
        << " joints, wrote " << output_count << " influences across "
        << used_joints << " joints\n";
    if (used_joints <= 1 && mesh->vcnt > 1) {
        std::cerr << "Auto weight: all vertices bound to one joint; the mesh "
            "will move rigidly\n";
    }
    dump_weights(*weight, mesh->vcnt, packed.size());
    return true;
}

} // namespace ORL
