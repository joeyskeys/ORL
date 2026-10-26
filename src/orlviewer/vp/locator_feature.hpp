#pragma once

#include <iostream>
#include <vector>

#include "component_manager.hpp"
#include "selection.hpp"
#include "vp/rhi_feature.hpp"
#include "vp/rig_shapes.hpp"

namespace ORL
{

class LocatorFeature final : public RhiFeature {
public:
    LocatorFeature(const ComponentManager& components, const Selection& selection)
        : components(components)
        , selection(selection)
    {
    }

    void on_attach(ViewerAdapter& adapter, Rhi::Extent2D) {
        report(rig_shapes::register_locator_shapes(adapter));
        report(adapter.prepare_locators());
    }

    void on_submit(ViewerAdapter& adapter, const Rhi::FrameState&) {
        // LocatorFeature intentionally renders the host-authored component
        // snapshot. Device-only solver locator state is published through
        // SceneGraphContext for GPU graph consumers; copying it back here
        // would reintroduce the readback that the fused solver avoids. A
        // future external-buffer locator draw can consume that publication
        // directly.
        locators.clear();
        components.for_each([&](const Component& meta) {
            if (meta.kind != ComponentKind::Locator) {
                return;
            }
            const auto* locator = components.locator(meta.id);
            if (locator == nullptr) {
                return;
            }
            locators.push_back({
                .model = to_rhi(locator->xform),
                .color = is_selected(meta.id) ? selected_color : unselected_color,
            });
        });
        if (locators.empty()) {
            return;
        }
        report(adapter.draw_locators(rig_shapes::kLocatorCross, locators));
    }

private:
    static void report(const Rhi::Status& status) {
        if (!status) {
            std::cerr << "LocatorFeature: " << status.message << '\n';
        }
    }

    bool is_selected(ComponentId id) const {
        for (const auto& ref : selection.refs()) {
            if (ref.kind == SelectionRef::Kind::Locator
                && ref.component == id)
            {
                return true;
            }
        }
        return false;
    }

    const ComponentManager& components;
    const Selection& selection;
    std::vector<Rhi::LocatorInstance> locators;
    Rhi::Vec4 selected_color{0.2f, 0.55f, 1.0f, 1.0f};
    Rhi::Vec4 unselected_color{0.95f, 0.78f, 0.28f, 1.0f};
};

} // namespace ORL
