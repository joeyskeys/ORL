#pragma once

#include <iostream>
#include <vector>

#include "component_manager.hpp"
#include "../scene_graph_context.hpp"
#include "selection.hpp"
#include "vp/rhi_feature.hpp"
#include "vp/rig_shapes.hpp"

namespace ORL
{

class LocatorFeature final : public RhiFeature {
public:
    LocatorFeature(SceneGraphContext& graph_context,
        const ComponentManager& components, const Selection& selection)
        : graph_context(graph_context)
        , components(components)
        , selection(selection)
    {
    }

    void on_attach(ViewerAdapter& adapter, Rhi::Extent2D) {
        report(rig_shapes::register_locator_shapes(adapter));
        report(adapter.prepare_locators());
    }

    void on_submit(ViewerAdapter& adapter, const Rhi::FrameState&) {
        locators.clear();
        for (const auto id : components.packed_locator_ids()) {
            const auto* meta = components.find(id);
            const auto* locator = components.locator(id);
            if (meta == nullptr || locator == nullptr) {
                continue;
            }
            locators.push_back({
                .model = to_rhi(locator->xform),
                .color = is_selected(meta->id) ? selected_color : unselected_color,
            });
        }
        if (locators.empty()) {
            return;
        }

        bool gpu_locators = false;
        const auto device_locators = graph_context.computed_locators_device();
        if (device_locators.has_value()
            && graph_context.computed_locators_device_count() != 0)
        {
            const auto count = static_cast<std::uint32_t>(
                graph_context.computed_locators_device_count());
            gpu_locators = static_cast<bool>(adapter.upload_device_locators(
                {device_locators->device_ptr, device_locators->bytes}, count));
        }

        report(adapter.draw_locators({
            .shape = rig_shapes::kLocatorCross,
            .instances = locators,
            .use_device_locators = gpu_locators,
        }));
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

    SceneGraphContext& graph_context;
    const ComponentManager& components;
    const Selection& selection;
    std::vector<Rhi::LocatorInstance> locators;
    Rhi::Vec4 selected_color{0.2f, 0.55f, 1.0f, 1.0f};
    Rhi::Vec4 unselected_color{0.95f, 0.78f, 0.28f, 1.0f};
};

} // namespace ORL
