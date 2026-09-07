#include "vp/solver_feature.hpp"

#include <cstdint>
#include <cstring>
#include <iostream>
#include <vector>

#include "comps/joint.hpp"

namespace ORL
{
namespace
{

void print_exec_errors(const char* stage, const std::vector<std::string>& errors) {
    for (const auto& error : errors) {
        std::cerr << "Solver: " << stage << ": " << error << '\n';
    }
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

void write_rotation(orlviewer::Joint& dst, const orlviewer::Joint& src) {
    dst.rotation[0] = src.rotation[0];
    dst.rotation[1] = src.rotation[1];
    dst.rotation[2] = src.rotation[2];
    dst.rotation[3] = src.rotation[3];
}

} // namespace

SolverFeature::SolverFeature(ComponentManager& components)
    : components(components)
    , joints("Joint", orlviewer::kJointStride)
{
}

void SolverFeature::on_update(vkkk::Context&, const vkkk::Context::Frame&) {
    if (components.size(ComponentKind::Constraint) == 0) {
        return;
    }
    if (!ensure_program()) {
        return;
    }

    std::vector<ComponentId> ids;
    components.for_each([&](const Component& meta) {
        if (meta.kind == ComponentKind::Constraint) {
            ids.push_back(meta.id);
        }
    });
    for (const ComponentId id : ids) {
        auto* constraint = components.constraint(id);
        if (constraint == nullptr || !constraint->bound || constraint->type != "ik_two_bone") {
            continue;
        }
        evaluate_two_bone(*constraint);
    }
}

bool SolverFeature::ensure_program() {
    if (program.has_value() && program->valid()
        && execution.has_value() && execution->valid()
        && execution->backend() == exec::Backend::Cpu)
    {
        execution->clear_bindings();
        return true;
    }

    program.reset();
    execution.reset();

    auto compiled = exec::OrlProgram::Compile("use solver/ik_two_bone;\n", {
        .entry_function = "solver_ik_two_bone",
        .source_name = "orl_solver_ik_two_bone",
    });
    if (!compiled.valid()) {
        print_exec_errors("compile", compiled.errors());
        return false;
    }
    auto created = exec::OrlExecution::Create(compiled, exec::Backend::Cpu);
    if (!created.valid()) {
        print_exec_errors("jit", created.errors());
        return false;
    }
    program = std::move(compiled);
    execution = std::move(created);
    return true;
}

bool SolverFeature::evaluate_two_bone(ConstraintData& constraint) {
    if (components.joint(constraint.root) == nullptr
        || components.joint(constraint.mid) == nullptr
        || components.joint(constraint.end) == nullptr
        || components.joint(constraint.target) == nullptr)
    {
        constraint.bound = false;
        return false;
    }

    const auto packed = components.packed_joints();
    const auto root = components.joint_index(constraint.root);
    const auto mid = components.joint_index(constraint.mid);
    const auto end = components.joint_index(constraint.end);
    const auto target = components.joint_index(constraint.target);
    const auto pole = constraint.pole
        ? components.joint_index(constraint.pole)
        : static_cast<std::int64_t>(-1);
    if (root < 0 || mid < 0 || end < 0 || target < 0) {
        constraint.bound = false;
        return false;
    }
    if (!fill_joints(joints, packed)) {
        return false;
    }

    const auto joint_count = static_cast<std::int64_t>(packed.size());
    if (!execution->bind_buffer("joints", joints)
        || !execution->bind_int("root", root)
        || !execution->bind_int("mid", mid)
        || !execution->bind_int("end", end)
        || !execution->bind_int("target", target)
        || !execution->bind_int("pole", pole)
        || !execution->bind_int("joint_count", joint_count))
    {
        print_exec_errors("bind", execution->errors());
        return false;
    }

    const auto result = execution->evaluate(1);
    if (!result.has_value()) {
        print_exec_errors("evaluate", execution->errors());
        constraint.bound = false;
        return false;
    }

    const auto* solved = static_cast<const orlviewer::Joint*>(joints.data());
    if (solved == nullptr) {
        return false;
    }
    if (auto* joint = components.joint(constraint.root)) {
        write_rotation(*joint, solved[static_cast<std::size_t>(root)]);
    }
    if (auto* joint = components.joint(constraint.mid)) {
        write_rotation(*joint, solved[static_cast<std::size_t>(mid)]);
    }
    return true;
}

} // namespace ORL
