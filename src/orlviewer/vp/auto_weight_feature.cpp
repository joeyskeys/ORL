#include "vp/auto_weight_feature.hpp"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <glm/geometric.hpp>
#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

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

bool fill_positions(exec::OrlBuffer& positions, const vkkk::Mesh& mesh, const glm::mat4& model) {
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
        const glm::vec4 world = model * glm::vec4{src[0], src[1], src[2], 1.0f};
        dst[v * 4 + 0] = world.x;
        dst[v * 4 + 1] = world.y;
        dst[v * 4 + 2] = world.z;
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

bool fill_radii(exec::OrlBuffer& radii, const std::vector<orlviewer::Joint>& packed) {
    if (!radii.resize(packed.size())) {
        return false;
    }
    double acc = 0.0;
    int bones = 0;
    for (std::size_t i = 0; i < packed.size(); ++i) {
        if (packed[i].parent < 0) {
            continue;
        }
        const glm::vec3 child{orlviewer::joint_world_matrix(packed, static_cast<std::int64_t>(i))[3]};
        const glm::vec3 parent{orlviewer::joint_world_matrix(packed, packed[i].parent)[3]};
        acc += static_cast<double>(glm::length(child - parent));
        ++bones;
    }
    double radius = bones > 0 ? acc / static_cast<double>(bones) : 1.0;
    if (radius < 0.25) {
        radius = 0.25;
    }
    radius *= 1.25;
    auto* out = static_cast<double*>(radii.data());
    for (std::size_t i = 0; i < packed.size(); ++i) {
        out[i] = radius;
    }
    return true;
}

void write_text_file(const char* path, const std::string& text) {
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    if (!out) {
        std::cerr << "Auto weight: could not write " << path << '\n';
        return;
    }
    out << text;
    std::cout << "Auto weight: wrote " << path << " (" << text.size() << " bytes)\n";
}

void dump_weights(const WeightData& weight, std::size_t vertex_count, std::size_t joint_count) {
    const auto weight_cnt = std::max<std::int64_t>(1, weight.weight_cnt);
    const auto out_count = weight.weights.count();
    const auto* cells = static_cast<const orlviewer::Weight*>(weight.weights.data());
    if (cells == nullptr || out_count == 0) {
        std::cerr << "Auto weight: weight buffers are empty, skip dump\n";
        return;
    }

    std::vector<std::size_t> counts(joint_count, 0);
    std::size_t invalid = 0;
    for (std::size_t i = 0; i < out_count; ++i) {
        const auto bone = cells[i].joint;
        if (bone < 0 || static_cast<std::size_t>(bone) >= joint_count) {
            ++invalid;
            continue;
        }
        ++counts[static_cast<std::size_t>(bone)];
    }

    std::cout << "Auto weight: dump weight_cnt=" << weight_cnt
        << " slots=" << out_count << " invalid=" << invalid << '\n';
    for (std::size_t j = 0; j < counts.size(); ++j) {
        std::cout << "  bone " << j << ": " << counts[j] << " influences\n";
    }

    const std::size_t preview = std::min<std::size_t>(out_count, 8);
    for (std::size_t i = 0; i < preview; ++i) {
        const auto vertex = i / static_cast<std::size_t>(weight_cnt);
        const auto slot = i % static_cast<std::size_t>(weight_cnt);
        std::cout << "  vert " << vertex << " slot " << slot
            << " joint=" << cells[i].joint << " w=" << cells[i].weight << '\n';
    }

    std::ofstream out("orl_debug_weights.txt", std::ios::trunc);
    if (!out) {
        std::cerr << "Auto weight: could not write orl_debug_weights.txt\n";
        return;
    }
    out << "# vertex\tslot\tjoint\tweight\n";
    for (std::size_t i = 0; i < out_count; ++i) {
        const auto vertex = vertex_count == 0
            ? i
            : i / static_cast<std::size_t>(weight_cnt);
        const auto slot = vertex_count == 0
            ? i
            : i % static_cast<std::size_t>(weight_cnt);
        out << vertex << '\t' << slot << '\t' << cells[i].joint << '\t' << cells[i].weight << '\n';
    }
    std::cout << "Auto weight: wrote orl_debug_weights.txt (" << out_count << " rows)\n";
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

bool AutoWeightFeature::cycle_algorithm() {
    constexpr int count = static_cast<int>(sizeof(kAlgorithms) / sizeof(kAlgorithms[0]));
    int index = 0;
    for (; index < count; ++index) {
        if (algorithm_name == kAlgorithms[index]) {
            break;
        }
    }
    index = (index + 1) % count;
    algorithm_name = kAlgorithms[index];
    std::cout << "Auto weight: algorithm '" << algorithm_name << "'\n";
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
    write_text_file("orl_debug_auto_weight.ll", execution->ir());
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
    exec::OrlBuffer scratch("float", sizeof(double));

    if (!fill_positions(positions, *mesh, selection.selected_mesh_model()) || !fill_joints(joints, packed)) {
        std::cerr << "Auto weight: failed to pack mesh or joints\n";
        return false;
    }

    bool needs_csr_buffers = false;
    bool needs_scratch = false;
    for (const auto& parameter : program->parameters()) {
        if (parameter.name == "offsets" || parameter.name == "neighbors") {
            needs_csr_buffers = true;
        }
        if (parameter.name == "scratch") {
            needs_scratch = true;
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
    if (!fill_radii(radii, packed)) {
        std::cerr << "Auto weight: failed to pack radii\n";
        return false;
    }

    auto weight_cnt = weight->weight_cnt;
    if (weight_cnt <= 0) {
        weight_cnt = orlviewer::kDefaultWeightCount;
    }
    weight->weight_cnt = weight_cnt;
    if (needs_scratch
        && !scratch.resize(static_cast<std::size_t>(mesh->vcnt) * packed.size()))
    {
        std::cerr << "Auto weight: failed to resize scratch\n";
        return false;
    }
    const auto out_count = static_cast<std::size_t>(mesh->vcnt) * static_cast<std::size_t>(weight_cnt);
    if (!weight->weights.resize(out_count)) {
        std::cerr << "Auto weight: failed to resize weight buffer\n";
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
        else if (parameter.name == "scratch") {
            ok = execution->bind_buffer("scratch", scratch);
        }
        else if (parameter.name == "vertex_count") {
            ok = execution->bind_int("vertex_count", vertex_count);
        }
        else if (parameter.name == "joint_count") {
            ok = execution->bind_int("joint_count", joint_count);
        }
        else if (parameter.name == "weight_cnt") {
            ok = execution->bind_int("weight_cnt", weight_cnt);
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

    std::size_t used_joints = 0;
    if (const auto* cells = static_cast<const orlviewer::Weight*>(weight->weights.data())) {
        std::vector<char> used(static_cast<std::size_t>(joint_count), 0);
        for (std::size_t i = 0; i < out_count; ++i) {
            const auto bone = cells[i].joint;
            if (bone < 0 || bone >= joint_count) {
                continue;
            }
            auto& flag = used[static_cast<std::size_t>(bone)];
            if (flag == 0) {
                flag = 1;
                ++used_joints;
            }
        }
    }

    std::cout << "Auto weight: " << algorithm_name << " '" << mesh_name << "' "
        << mesh->vcnt << " verts, " << packed.size() << " joints, wrote "
        << out_count << " influences across " << used_joints << " joints\n";
    if (used_joints <= 1 && mesh->vcnt > 1) {
        std::cerr << "Auto weight: all vertices bound to one joint; the mesh will move rigidly\n";
    }
    dump_weights(*weight, static_cast<std::size_t>(mesh->vcnt), packed.size());
    return true;
}

} // namespace ORL
