#pragma once

#include "component_manager.hpp"
#include "../../orlexec/orlrig/runners.hpp"
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
    bool evaluate_two_bone(ConstraintData& constraint);

    ComponentManager& components;
    orlrig::SolverRunner runner;
};

} // namespace ORL
