#include "vp/deformer_feature.hpp"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include "comps/joint.hpp"
#include "concepts/mesh.h"
#include "vk_ins/types.h"
#include "asset_mgr/drawable_mgr.h"

namespace ORL
{
namespace
{

constexpr const char* kTypes[] = {
    "lbs",
};

bool known_type(std::string_view name) {
    for (const char* type : kTypes) {
        if (name == type) {
            return true;
        }
    }
    return false;
}

void print_exec_errors(const char* stage, const std::vector<std::string>& errors) {
    for (const auto& error : errors) {
        std::cerr << "Deformer: " << stage << ": " << error << '\n';
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

bool write_positions(vkkk::Mesh& mesh, const exec::OrlBuffer& positions) {
    const int offset = vertex_float_offset(mesh);
    if (offset < 0 || mesh.vbuf == nullptr || positions.count() < mesh.vcnt) {
        return false;
    }
    const auto* src = static_cast<const double*>(positions.data());
    for (std::uint32_t v = 0; v < mesh.vcnt; ++v) {
        float* dst = mesh.vbuf + v * mesh.comp_size + offset;
        dst[0] = static_cast<float>(src[v * 4 + 0]);
        dst[1] = static_cast<float>(src[v * 4 + 1]);
        dst[2] = static_cast<float>(src[v * 4 + 2]);
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

} // namespace

DeformerFeature::DeformerFeature(vkkk::Scene& scene, ComponentManager& components,
    ComponentId deformer_id, ComponentId weight_id, const Selection& selection)
    : scene(scene)
    , components(components)
    , deformer_id(deformer_id)
    , weight_id(weight_id)
    , selection(selection)
    , joints("Joint", orlviewer::kJointStride)
    , output_positions("point", sizeof(double) * 4)
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

void DeformerFeature::on_update(vkkk::Context& context, const vkkk::Context::Frame&) {
    if (pending) {
        pending = false;
        setup();
    }
    if (const auto* deformer = components.deformer(deformer_id); deformer != nullptr && deformer->bound) {
        evaluate(context);
    }
}

bool DeformerFeature::ensure_programs() {
    if (compiled == type_name && capture_program.has_value() && capture_program->valid()
        && capture_execution.has_value() && capture_execution->valid()
        && deform_program.has_value() && deform_program->valid()
        && deform_execution.has_value() && deform_execution->valid())
    {
        capture_execution->clear_bindings();
        deform_execution->clear_bindings();
        return true;
    }

    capture_program.reset();
    capture_execution.reset();
    deform_program.reset();
    deform_execution.reset();
    compiled.clear();

    const std::string source = "use deformer/" + type_name + ";\n";
    auto compiled_capture = exec::OrlProgram::Compile(source, {
        .entry_function = "deformer_" + type_name + "_capture_bind",
        .source_name = "orl_deformer_capture",
    });
    if (!compiled_capture.valid()) {
        print_exec_errors("compile capture", compiled_capture.errors());
        return false;
    }
    auto compiled_deform = exec::OrlProgram::Compile(source, {
        .entry_function = "deformer_" + type_name,
        .source_name = "orl_deformer",
    });
    if (!compiled_deform.valid()) {
        print_exec_errors("compile deform", compiled_deform.errors());
        return false;
    }

    auto capture_exec = exec::OrlExecution::Create(compiled_capture, exec::Backend::Cpu);
    if (!capture_exec.valid()) {
        print_exec_errors("jit capture", capture_exec.errors());
        return false;
    }
    auto deform_exec = exec::OrlExecution::Create(compiled_deform, exec::Backend::Cpu);
    if (!deform_exec.valid()) {
        print_exec_errors("jit deform", deform_exec.errors());
        return false;
    }

    capture_program = std::move(compiled_capture);
    capture_execution = std::move(capture_exec);
    deform_program = std::move(compiled_deform);
    deform_execution = std::move(deform_exec);
    compiled = type_name;
    return true;
}

bool DeformerFeature::setup() {
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
    if (weight->bone_indices.count() == 0 || weight->weights.count() == 0) {
        std::cerr << "Deformer: skipped, auto-weight did not fill weight buffers\n";
        return false;
    }

    if (!ensure_programs()) {
        return false;
    }
    if (!fill_positions(deformer->bind_positions, *mesh)
        || !fill_joints(joints, packed)
        || !deformer->inverse_binds.resize(packed.size()))
    {
        std::cerr << "Deformer: failed to pack bind pose\n";
        return false;
    }

    if (!capture_execution->bind_buffer("joints", joints)
        || !capture_execution->bind_buffer("inverse_binds", deformer->inverse_binds)
        || !capture_execution->bind_int("joint_count", static_cast<std::int64_t>(packed.size())))
    {
        print_exec_errors("bind capture", capture_execution->errors());
        return false;
    }

    const auto captured = capture_execution->evaluate(static_cast<std::uint32_t>(packed.size()));
    if (!captured.has_value()) {
        print_exec_errors("capture", capture_execution->errors());
        return false;
    }

    deformer->mesh_name = mesh_name;
    deformer->type = type_name;
    deformer->bound = true;
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

    std::string mesh_name = deformer->mesh_name;
    if (mesh_name.empty() || scene.drawable_mgr == nullptr) {
        return false;
    }
    auto* mesh = scene.drawable_mgr->find_mesh(mesh_name);
    if (mesh == nullptr) {
        return false;
    }

    const auto packed = components.packed_joints();
    if (packed.empty() || packed.size() != deformer->inverse_binds.count()) {
        return false;
    }
    if (!ensure_programs()) {
        return false;
    }
    if (!fill_joints(joints, packed) || !output_positions.resize(mesh->vcnt)) {
        return false;
    }

    const auto influences = std::max<std::int64_t>(1, weight->influences_per_vertex);
    const auto vertex_count = static_cast<std::int64_t>(mesh->vcnt);
    const auto joint_count = static_cast<std::int64_t>(packed.size());
    for (const auto& parameter : deform_program->parameters()) {
        bool ok = true;
        if (parameter.name == "bind_positions") {
            ok = deform_execution->bind_buffer("bind_positions", deformer->bind_positions);
        }
        else if (parameter.name == "output_positions") {
            ok = deform_execution->bind_buffer("output_positions", output_positions);
        }
        else if (parameter.name == "joints") {
            ok = deform_execution->bind_buffer("joints", joints);
        }
        else if (parameter.name == "inverse_binds") {
            ok = deform_execution->bind_buffer("inverse_binds", deformer->inverse_binds);
        }
        else if (parameter.name == "bone_indices") {
            ok = deform_execution->bind_buffer("bone_indices", weight->bone_indices);
        }
        else if (parameter.name == "weights") {
            ok = deform_execution->bind_buffer("weights", weight->weights);
        }
        else if (parameter.name == "vertex_count") {
            ok = deform_execution->bind_int("vertex_count", vertex_count);
        }
        else if (parameter.name == "joint_count") {
            ok = deform_execution->bind_int("joint_count", joint_count);
        }
        else if (parameter.name == "influences_per_vertex") {
            ok = deform_execution->bind_int("influences_per_vertex", influences);
        }
        else {
            std::cerr << "Deformer: unhandled parameter '" << parameter.name << "'\n";
            return false;
        }
        if (!ok) {
            print_exec_errors("bind deform", deform_execution->errors());
            return false;
        }
    }

    const auto result = deform_execution->evaluate(static_cast<std::uint32_t>(mesh->vcnt));
    if (!result.has_value()) {
        print_exec_errors("evaluate", deform_execution->errors());
        deformer->bound = false;
        return false;
    }

    if (!write_positions(*mesh, output_positions)) {
        std::cerr << "Deformer: failed to write mesh positions\n";
        return false;
    }
    if (!context.load_mesh(mesh_name, *mesh)) {
        std::cerr << "Deformer: GPU mesh upload failed for '" << mesh_name << "'\n";
        return false;
    }
    return true;
}

} // namespace ORL
