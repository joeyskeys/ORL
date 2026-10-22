#pragma once

#include <string>
#include <string_view>

#include "asset_mgr/scene.h"
#include "component_manager.hpp"
#include "../../orlexec/orlrig/runners.hpp"
#include "selection.hpp"
#include "vp/mesh_csr_feature.hpp"
#include "vp/rhi_feature.hpp"

namespace ORL
{

// Runs an ORL stdlib auto-weight entry against the scene mesh and joints,
// writing into a Weight component. Default is Maya Closest Distance.
class AutoWeightFeature final : public RhiFeature {
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
    void on_update(ViewerAdapter& adapter, const Rhi::FrameState&);

private:
    bool run(ViewerAdapter& adapter);

    vkkk::Scene& scene;
    ComponentManager& components;
    ComponentId weight_id;
    const Selection& selection;
    MeshCsrFeature* csr = nullptr;
    std::string algorithm_name{"closest_distance"};
    double dropoff = 4.0;
    orlrig::AutoWeightRunner runner;
    bool pending = false;
};

} // namespace ORL
