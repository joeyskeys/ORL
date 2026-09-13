#pragma once

#include <string>
#include <string_view>

#include "component_manager.hpp"
#include "../graph_scene_runtime.hpp"
#include "selection.hpp"
#include "vp/feature.hpp"

namespace ORL
{

// Binds a stdlib deformer to a mesh + joints + weight buffer, then evaluates
// it each frame. Default type is lbs.
class DeformerFeature final : public vkkk::vp::ViewportFeature<vkkk::vp::ViewportPhase::Scene> {
public:
    DeformerFeature(SceneGraphContext& graph_context,
        ComponentId deformer_id, ComponentId weight_id, const Selection& selection);

    bool set_type(std::string_view name);
    std::string_view type() const { return type_name; }
    void set_mesh(std::string name);

    void request();
    void unbind();
    void on_update(vkkk::Context& context, const vkkk::Context::Frame&);

private:
    std::string type_name{"lbs"};
    GraphSceneRuntime runtime;
};

} // namespace ORL
