#pragma once

#include "component_manager.hpp"
#include "../../orlexec/orlrig/runners.hpp"
#include "../graph_scene_runtime.hpp"
#include "../selection.hpp"
#include "vp/feature.hpp"

namespace ORL
{

// Evaluates the staged solver graph before deformation. The legacy component
// constraint runner remains as a compatibility fallback for older scenes.
class SolverFeature final : public vkkk::vp::ViewportFeature<vkkk::vp::ViewportPhase::Scene> {
public:
    SolverFeature(SceneGraphContext& graph_context,
        ComponentManager& components, const Selection& selection);

    void on_update(vkkk::Context&, const vkkk::Context::Frame&);

private:
    bool evaluate_two_bone(ConstraintData& constraint);

    ComponentManager& components;
    orlrig::SolverRunner runner;
    GraphSceneRuntime graph_runtime;
};

} // namespace ORL
