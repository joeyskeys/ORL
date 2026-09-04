#pragma once

#include <optional>
#include <string>
#include <string_view>

#include "asset_mgr/scene.h"
#include "component_manager.hpp"
#include "orl_exec.hpp"
#include "selection.hpp"
#include "vp/feature.hpp"
#include "vp/mesh_csr_feature.hpp"

namespace ORL
{

// Runs an ORL stdlib auto-weight entry against the scene mesh and joints,
// writing into a Weight component. Default algorithm is closest_joint.
class AutoWeightFeature final : public vkkk::vp::ViewportFeature<vkkk::vp::ViewportPhase::Scene> {
public:
    AutoWeightFeature(vkkk::Scene& scene, ComponentManager& components, ComponentId weight_id,
        const Selection& selection);

    void set_csr(MeshCsrFeature& feature);
    bool set_algorithm(std::string_view name);
    bool cycle_algorithm();
    std::string_view algorithm() const { return algorithm_name; }
    void set_dropoff(double value) { dropoff = value; }

    void request();
    void cancel_request() { pending = false; }
    void on_update(vkkk::Context& context, const vkkk::Context::Frame&);

private:
    bool ensure_program();
    bool run(vkkk::Context& context);

    vkkk::Scene& scene;
    ComponentManager& components;
    ComponentId weight_id;
    const Selection& selection;
    MeshCsrFeature* csr = nullptr;
    std::string algorithm_name{"closest_joint"};
    double dropoff = 4.0;
    std::string compiled;
    std::optional<exec::OrlProgram> program;
    std::optional<exec::OrlExecution> execution;
    bool pending = false;
};

} // namespace ORL
