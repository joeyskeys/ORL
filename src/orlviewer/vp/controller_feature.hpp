#pragma once

#include <iostream>

#include "component_manager.hpp"
#include "selection.hpp"
#include "vp/rhi_feature.hpp"
#include "vp/rig_shapes.hpp"

namespace ORL
{

class ControllerFeature final : public RhiFeature {
public:
    ControllerFeature(const ComponentManager& components, const Selection& selection)
        : components(components)
        , selection(selection)
    {
    }

    void on_attach(ViewerAdapter& adapter, Rhi::Extent2D) {
        report(rig_shapes::register_controller_shapes(adapter));
        report(adapter.prepare_controllers());
    }

    void on_submit(ViewerAdapter& adapter, const Rhi::FrameState&) {
        if (components.size(ComponentKind::Controller) == 0) {
            return;
        }

        components.for_each([&](const Component& meta) {
            if (meta.kind != ComponentKind::Controller) {
                return;
            }
            if (components.controller(meta.id) == nullptr) {
                return;
            }
            report(adapter.draw_controller({
                .shape = rig_shapes::controller_shape(components.controller_shape(meta.id)),
                .model = to_rhi(components.controller_world_xform(meta.id)),
                .color = is_selected(meta.id) ? selected_color : unselected_color,
            }));
        });
    }

private:
    static void report(const Rhi::Status& status) {
        if (!status) {
            std::cerr << "ControllerFeature: " << status.message << '\n';
        }
    }

    bool is_selected(ComponentId id) const {
        for (const auto& ref : selection.refs()) {
            if (ref.kind == SelectionRef::Kind::Controller && ref.component == id) {
                return true;
            }
        }
        return false;
    }

    const ComponentManager& components;
    const Selection& selection;
    Rhi::Vec4 selected_color{0.2f, 0.55f, 1.0f, 1.0f};
    Rhi::Vec4 unselected_color{0.95f, 0.55f, 0.18f, 1.0f};
};

} // namespace ORL
