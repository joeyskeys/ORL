#pragma once

#include <optional>
#include <string>
#include <string_view>

#include "asset_mgr/scene.h"
#include "component_manager.hpp"
#include "orl_exec.hpp"
#include "selection.hpp"
#include "vp/feature.hpp"

namespace ORL
{

// Binds a stdlib deformer to a mesh + joints + weight buffer, then evaluates
// it each frame. Default type is lbs.
class DeformerFeature final : public vkkk::vp::ViewportFeature<vkkk::vp::ViewportPhase::Scene> {
public:
    DeformerFeature(vkkk::Scene& scene, ComponentManager& components,
        ComponentId deformer_id, ComponentId weight_id, const Selection& selection);

    bool set_type(std::string_view name);
    std::string_view type() const { return type_name; }
    void set_mesh(std::string name);

    void request();
    void unbind();
    void on_update(vkkk::Context& context, const vkkk::Context::Frame&);

private:
    bool ensure_programs();
    bool setup();
    bool evaluate(vkkk::Context& context);

    vkkk::Scene& scene;
    ComponentManager& components;
    ComponentId deformer_id;
    ComponentId weight_id;
    const Selection& selection;
    std::string type_name{"lbs"};
    std::string compiled;
    std::optional<exec::OrlProgram> capture_program;
    std::optional<exec::OrlExecution> capture_execution;
    std::optional<exec::OrlProgram> deform_program;
    std::optional<exec::OrlExecution> deform_execution;
    exec::OrlBuffer joints;
    exec::OrlBuffer output_positions;
    bool pending = false;
    bool logged_rest = false;
    bool logged_move = false;
};

} // namespace ORL
