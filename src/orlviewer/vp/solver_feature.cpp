#include "vp/solver_feature.hpp"

#include <cstring>
#include <iostream>
#include <vector>

#include "comps/controller.hpp"
#include "comps/joint.hpp"
#include "runtime_config.hpp"

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
        std::cerr << "Solver: " << error << '\n';
    }
}

void write_rotation(orlviewer::Joint& destination,
    const orlviewer::Joint& source)
{
    std::memcpy(destination.rotation, source.rotation, sizeof(destination.rotation));
}

} // namespace

SolverFeature::SolverFeature(ComponentManager& components)
    : components(components)
    , runner(backend_from_config())
{
}

void SolverFeature::on_update(vkkk::Context&, const vkkk::Context::Frame&) {
    if (components.size(ComponentKind::Constraint) == 0) {
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
        if (constraint == nullptr || !constraint->bound
            || constraint->type != "ik_two_bone")
        {
            continue;
        }
        evaluate_two_bone(*constraint);
    }
}

bool SolverFeature::evaluate_two_bone(ConstraintData& constraint) {
    if (components.joint(constraint.root) == nullptr
        || components.joint(constraint.mid) == nullptr
        || components.joint(constraint.end) == nullptr)
    {
        constraint.bound = false;
        return false;
    }
    const auto* target = components.controller(constraint.target);
    const auto* pole = components.controller(constraint.pole);
    if (target == nullptr || pole == nullptr) {
        constraint.bound = false;
        return false;
    }

    auto packed = components.packed_joints();
    const auto root = components.joint_index(constraint.root);
    const auto mid = components.joint_index(constraint.mid);
    const auto end = components.joint_index(constraint.end);
    if (root < 0 || mid < 0 || end < 0) {
        constraint.bound = false;
        return false;
    }

    const auto status = runner.evaluate_two_bone(
        packed, root, mid, end, *target, *pole);
    if (!status) {
        print_runner_errors(status.errors);
        constraint.bound = false;
        return false;
    }

    if (auto* joint = components.joint(constraint.root)) {
        write_rotation(*joint, packed[static_cast<std::size_t>(root)]);
    }
    if (auto* joint = components.joint(constraint.mid)) {
        write_rotation(*joint, packed[static_cast<std::size_t>(mid)]);
    }
    return true;
}

} // namespace ORL
