#include "vp/auto_weight_feature.hpp"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include "comps/joint.hpp"
#include "concepts/mesh.h"
#include "orl_exec.hpp"
#include "vk_ins/types.h"
#include "asset_mgr/drawable_mgr.h"

namespace ORL
{
namespace
{

constexpr const char* kAlgorithms[] = {
    "closest_bone",
    "closest_joint",
    "envelope",
    "heat",
    "geodesic",
    "harmonic",
    "bounded_biharmonic",
};

bool known_algorithm(std::string_view name) {
    for (const char* algorithm : kAlgorithms) {
        if (name == algorithm) {
            return true;
        }
    }
    return false;
}

void print_exec_errors(const char* stage, const std::vector<std::string>& errors) {
    for (const auto& error : errors) {
        std::cerr << "Auto weight: " << stage << ": " << error << '\n';
    }
}

int vertex_float_offset(const vkkk::Mesh& mesh) {
    int packed = 0;
    for (const auto comp : mesh.comps) {
        if (comp == vkkk::VERTEX) {
            return packed;
        }
        packed += static_cast<int>(vkkk::comp_sizes[comp]);
    }
    return -1;
}

bool fill_positions(exec::OrlBuffer& positions, const vkkk::Mesh& mesh) {
    const int offset = vertex_float_offset(mesh);
    if (offset < 0 || mesh.vbuf == nullptr || mesh.vcnt == 0) {
        return false;
    }
    if (!positions.resize(mesh.vcnt)) {
        return false;
    }
    auto* dst = static_cast<double*>(positions.data());
    for (std::uint32_t v = 0; v < mesh.vcnt; ++v) {
        const float* src = mesh.vbuf + v * mesh.comp_size + offset;
        dst[v * 4 + 0] = src[0];
        dst[v * 4 + 1] = src[1];
        dst[v * 4 + 2] = src[2];
        dst[v * 4 + 3] = 0.0;
    }
    return true;
}

bool fill_joints(exec::OrlBuffer& joints, const std::vector<orlviewer::Joint>& packed) {
    if (!joints.resize(packed.size())) {
        return false;
    }
    if (!packed.empty()) {
        std::memcpy(joints.data(), packed.data(), packed.size() * orlviewer::kJointStride);
    }
    return true;
}

bool fill_uints_as_int(exec::OrlBuffer& dst, const std::vector<std::uint32_t>& src) {
    if (!dst.resize(src.size())) {
        return false;
    }
    auto* out = static_cast<std::int64_t*>(dst.data());
    for (std::size_t i = 0; i < src.size(); ++i) {
        out[i] = static_cast<std::int64_t>(src[i]);
    }
    return true;
}

bool fill_radii(exec::OrlBuffer& radii, std::size_t joint_count) {
    if (!radii.resize(joint_count)) {
        return false;
    }
    auto* out = static_cast<double*>(radii.data());
    for (std::size_t i = 0; i < joint_count; ++i) {
        out[i] = 1.0;
    }
    return true;
}

} // namespace

AutoWeightFeature::AutoWeightFeature(vkkk::Scene& scene, ComponentManager& components,
    ComponentId weight_id, const Selection& selection)
    : scene(scene)
    , components(components)
    , weight_id(weight_id)
    , selection(selection)
{
}

void AutoWeightFeature::set_csr(MeshCsrFeature& feature) {
    csr = &feature;
}

bool AutoWeightFeature::set_algorithm(std::string_view name) {
    if (!known_algorithm(name)) {
        std::cerr << "Auto weight: unknown algorithm '" << name << "'\n";
        return false;
    }
    algorithm_name = std::string{name};
    return true;
}

void AutoWeightFeature::request() {
    if (!selection.valid_for_bind()) {
        std::cerr << "Auto weight: select a mesh and joints first\n";
        return;
    }
    pending = true;
}

void AutoWeightFeature::on_update(vkkk::Context& context, const vkkk::Context::Frame&) {
    if (!pending) {
        return;
    }
    pending = false;
    run(context);
}

bool AutoWeightFeature::ensure_program() {
    if (compiled == algorithm_name && program.has_value() && program->valid()
        && execution.has_value() && execution->valid())
    {
        execution->clear_bindings();
        return true;
    }

    program.reset();
    execution.reset();
    compiled.clear();

    const std::string source = "use auto_weight/" + algorithm_name + ";\n";
    auto compiled_program = exec::OrlProgram::Compile(source, {
        .entry_function = "auto_weight_" + algorithm_name,
        .source_name = "orl_auto_weight",
    });
    if (!compiled_program.valid()) {
        print_exec_errors("compile", compiled_program.errors());
        return false;
    }

    auto compiled_execution = exec::OrlExecution::Create(compiled_program, exec::Backend::Cpu);
    if (!compiled_execution.valid()) {
        print_exec_errors("jit", compiled_execution.errors());
        return false;
    }

    program = std::move(compiled_program);
    execution = std::move(compiled_execution);
    compiled = algorithm_name;
    return true;
}

bool AutoWeightFeature::run(vkkk::Context& context) {
    WeightData* weight = components.weight(weight_id);
    if (weight == nullptr) {
        std::cerr << "Auto weight: missing weight component\n";
        return false;
    }

    std::string mesh_name = selection.selected_mesh_name();
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

    const MeshCsr* built = nullptr;
    if (csr != nullptr) {
        if (!csr->build(context, mesh_name, *mesh)) {
            std::cerr << "Auto weight: CSR build failed for '" << mesh_name << "'\n";
        }
        built = csr->find(mesh_name);
    }

    if (!ensure_program()) {
        return false;
    }

    exec::OrlBuffer positions("point", sizeof(double) * 4);
    exec::OrlBuffer joints("Joint", orlviewer::kJointStride);
    exec::OrlBuffer offsets("int", sizeof(std::int64_t));
    exec::OrlBuffer neighbors("int", sizeof(std::int64_t));
    exec::OrlBuffer radii("float", sizeof(double));

    if (!fill_positions(positions, *mesh) || !fill_joints(joints, packed)) {
        std::cerr << "Auto weight: failed to pack mesh or joints\n";
        return false;
    }

    bool needs_csr_buffers = false;
    bool needs_influences = false;
    for (const auto& parameter : program->parameters()) {
        if (parameter.name == "offsets" || parameter.name == "neighbors") {
            needs_csr_buffers = true;
        }
        if (parameter.name == "influences_per_vertex") {
            needs_influences = true;
        }
    }
    if (needs_csr_buffers && built == nullptr) {
        std::cerr << "Auto weight: '" << algorithm_name << "' needs CSR\n";
        return false;
    }
    if (built != nullptr) {
        if (!fill_uints_as_int(offsets, built->offsets)
            || !fill_uints_as_int(neighbors, built->neighbors))
        {
            std::cerr << "Auto weight: failed to pack CSR\n";
            return false;
        }
    }
    if (!fill_radii(radii, packed.size())) {
        std::cerr << "Auto weight: failed to pack radii\n";
        return false;
    }

    const auto influences = std::max<std::int64_t>(1, weight->influences_per_vertex);
    const auto out_count = needs_influences
        ? static_cast<std::size_t>(mesh->vcnt) * static_cast<std::size_t>(influences)
        : static_cast<std::size_t>(mesh->vcnt);
    if (!weight->bone_indices.resize(out_count) || !weight->weights.resize(out_count)) {
        std::cerr << "Auto weight: failed to resize weight buffers\n";
        return false;
    }

    const auto vertex_count = static_cast<std::int64_t>(mesh->vcnt);
    const auto joint_count = static_cast<std::int64_t>(packed.size());
    for (const auto& parameter : program->parameters()) {
        bool ok = true;
        if (parameter.name == "positions") {
            ok = execution->bind_buffer("positions", positions);
        }
        else if (parameter.name == "joints") {
            ok = execution->bind_buffer("joints", joints);
        }
        else if (parameter.name == "bone_indices") {
            ok = execution->bind_buffer("bone_indices", weight->bone_indices);
        }
        else if (parameter.name == "weights") {
            ok = execution->bind_buffer("weights", weight->weights);
        }
        else if (parameter.name == "offsets") {
            ok = execution->bind_buffer("offsets", offsets);
        }
        else if (parameter.name == "neighbors") {
            ok = execution->bind_buffer("neighbors", neighbors);
        }
        else if (parameter.name == "radii") {
            ok = execution->bind_buffer("radii", radii);
        }
        else if (parameter.name == "vertex_count") {
            ok = execution->bind_int("vertex_count", vertex_count);
        }
        else if (parameter.name == "joint_count") {
            ok = execution->bind_int("joint_count", joint_count);
        }
        else if (parameter.name == "influences_per_vertex") {
            ok = execution->bind_int("influences_per_vertex", influences);
        }
        else {
            std::cerr << "Auto weight: unhandled parameter '" << parameter.name << "'\n";
            return false;
        }
        if (!ok) {
            print_exec_errors("bind", execution->errors());
            return false;
        }
    }

    const auto result = execution->evaluate(static_cast<std::uint32_t>(mesh->vcnt));
    if (!result.has_value()) {
        print_exec_errors("evaluate", execution->errors());
        return false;
    }

    std::cout << "Auto weight: " << algorithm_name << " '" << mesh_name << "' "
        << mesh->vcnt << " verts, " << packed.size() << " joints, wrote "
        << out_count << " influences\n";
    return true;
}

} // namespace ORL
