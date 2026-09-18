#include "vp/solver_feature.hpp"

#include <iostream>
#include <string>
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

} // namespace

SolverFeature::SolverFeature(SceneGraphContext& graph_context,
    ComponentManager& components, const Selection& selection)
    : graph_context(graph_context)
    , components(components)
    , runner(backend_from_config())
    , graph_runtime(
        graph_context, selection, {}, {},
        orlgraph::GraphStage::Solver)
{
    graph_context.ensure_stage_graphs();
}

void SolverFeature::on_update(
    vkkk::Context& context, const vkkk::Context::Frame&)
{
    if (!runtime_config.evaluate_orl) {
        graph_context.scene_inputs().set_cuda_evaluation(false);
        graph_context.clear_computed_joints_device();
        return;
    }
    // A populated solver-stage graph is authoritative. Keep the component
    // constraint path as a compatibility fallback for scenes authored before
    // staged graphs existed.
    if (graph_runtime.has_active_graph_content()) {
        graph_runtime.on_update(context);
        return;
    }
    std::string hierarchy_error;
    if (!graph_context.ensure_hierarchy_plan(&hierarchy_error)) {
        std::cerr << "Solver: " << hierarchy_error << '\n';
        return;
    }
    if (const auto status = runner.set_hierarchy_plan(
            *graph_context.hierarchy_plan());
        !status)
    {
        print_runner_errors(status.errors);
        return;
    }
    graph_context.clear_computed_joints_device();
    graph_context.scene_inputs().set_cuda_evaluation(false);
    std::string input_error;
    if (!components.apply_controller_inputs(&input_error)) {
        std::cerr << "Solver: controller input application failed: "
                  << input_error << '\n';
        return;
    }
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
    const auto target_is_chain_joint = [&](ComponentId target) {
        return target == constraint.root
            || target == constraint.mid
            || target == constraint.end;
    };
    const auto attachment_creates_cycle = [&](ComponentId source) {
        const auto* attachment = components.controller_attachment(source);
        return attachment != nullptr
            && attachment->target_kind == AttachmentTargetKind::Joint
            && target_is_chain_joint(attachment->target);
    };
    if (attachment_creates_cycle(constraint.target)
        || attachment_creates_cycle(constraint.pole))
    {
        std::cerr << "SolverFeature: rejected cyclic controller attachment\n";
        constraint.bound = false;
        return false;
    }
    const auto* target = components.locator(constraint.target);
    const auto* pole = components.locator(constraint.pole);
    orlrig::Locator target_compat;
    orlrig::Locator pole_compat;
    if (target == nullptr) {
        if (const auto* controller = components.controller(constraint.target);
            controller != nullptr)
        {
            target_compat.xform =
                components.controller_world_xform(constraint.target);
            target = &target_compat;
        }
    }
    if (pole == nullptr) {
        if (const auto* controller = components.controller(constraint.pole);
            controller != nullptr)
        {
            pole_compat.xform =
                components.controller_world_xform(constraint.pole);
            pole = &pole_compat;
        }
    }
    if (target == nullptr || pole == nullptr) {
        constraint.bound = false;
        return false;
    }

    const auto status = runner.evaluate_two_bone(
        components.rigging(),
        constraint.root, constraint.mid, constraint.end,
        *target, *pole);
    if (!status) {
        print_runner_errors(status.errors);
        constraint.bound = false;
        return false;
    }
    return true;
}

} // namespace ORL
