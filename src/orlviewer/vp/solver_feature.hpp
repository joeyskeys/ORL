#pragma once

#include <optional>
#include <string>

#include "component_manager.hpp"
#include "orl_exec.hpp"
#include "vp/feature.hpp"

namespace ORL
{

// Evaluates stdlib solvers against packed joints, then writes rotations back
// so LBS sees the posed chain. Runs on CPU; two-bone IK is sequential.
class SolverFeature final : public vkkk::vp::ViewportFeature<vkkk::vp::ViewportPhase::Scene> {
public:
    explicit SolverFeature(ComponentManager& components);

    void on_update(vkkk::Context&, const vkkk::Context::Frame&);

private:
    bool ensure_program();
    bool evaluate_two_bone(ConstraintData& constraint);

    ComponentManager& components;
    std::optional<exec::OrlProgram> program;
    std::optional<exec::OrlExecution> execution;
    exec::OrlBuffer joints;
    exec::OrlBuffer target_xform;
    exec::OrlBuffer pole_xform;
};

} // namespace ORL
