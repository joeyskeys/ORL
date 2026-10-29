#pragma once

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <unordered_map>
#include <utility>
#include <vector>

#include "component_manager.hpp"
#include "../scene_graph_context.hpp"
#include "selection.hpp"
#include "vp/picking.hpp"
#include "vp/rhi_feature.hpp"
#include "vp/rig_shapes.hpp"

namespace ORL
{

// GPU picking for controller shapes and locators. The two domains share one
// A-buffer; the high bits of each token identify the domain so a token cannot
// collide between controllers and locators.
class RigPickingFeature final : public RhiFeature {
public:
    using HitCallback = PickRequest::HitCallback;

    RigPickingFeature(SceneGraphContext& graph_context,
        const ComponentManager& components,
        std::uint32_t nodes_per_pixel = 8)
        : graph_context(graph_context)
        , components(components)
        , nodes_per_pixel(nodes_per_pixel)
    {
    }

    void on_attach(ViewerAdapter& adapter, Rhi::Extent2D) {
        ready = register_shapes(adapter) && prepare(adapter);
        std::cout << (ready ? "Rig picking: GPU\n"
                            : "Rig picking: CPU fallback (GPU prepare failed)\n");
    }

    void on_resize(ViewerAdapter& adapter, Rhi::Extent2D) {
        pick.reset();
        ready = ready && prepare(adapter);
    }

    void on_update(ViewerAdapter& adapter, const Rhi::FrameState& frame) {
        if (ready) {
            pick.update(adapter, frame);
        }
    }

    void on_submit(ViewerAdapter& adapter, const Rhi::FrameState&) {
        if (!ready || !enabled || !pick.pending()) {
            return;
        }

        controllers.clear();
        locators.clear();
        components.for_each([&](const Component& meta) {
            if (meta.kind == ComponentKind::Controller
                && components.controller(meta.id) != nullptr)
            {
                controllers.push_back({
                    .shape = rig_shapes::controller_shape(components.controller_shape(meta.id)),
                    .model = to_rhi(components.controller_world_xform(meta.id)),
                    .id = controller_token(meta.id),
                });
            }
        });
        for (const auto id : components.packed_locator_ids()) {
            const auto* locator = components.locator(id);
            if (locator == nullptr) {
                continue;
            }
            locators.push_back({
                .model = to_rhi(locator->xform),
                .id = locator_token(id),
            });
        }

        bool device_locators = false;
        const auto current_locators = graph_context.computed_locators_device();
        if (current_locators.has_value()
            && graph_context.computed_locators_device_count() != 0)
        {
            const auto count = static_cast<std::uint32_t>(
                std::min(
                    graph_context.computed_locators_device_count(),
                    locators.size()));
            device_locators = count != 0
                && static_cast<bool>(adapter.upload_device_pick_locators(
                    {current_locators->device_ptr, current_locators->bytes},
                    count));
        }

        const auto status = adapter.draw_rig_pick({
            .controllers = controllers,
            .locator_shape = rig_shapes::kLocatorCross,
            .locators = locators,
            .use_device_locators = device_locators,
        });
        if (!status) {
            std::cerr << "RigPickingFeature: " << status.message << '\n';
        }
    }

    bool available() const { return ready && enabled; }

    std::uint64_t request(double x, double y) {
        return available() ? pick.request(x, y) : 0;
    }

    void set_hit_callback(HitCallback callback) {
        pick.set_hit_callback(std::move(callback));
    }

    bool resolve(std::uint32_t token, SelectionRef& ref) const {
        if ((token & kControllerDomain) == kControllerDomain) {
            const auto found = controller_ids_by_token.find(token);
            if (found == controller_ids_by_token.end()
                || components.controller(found->second) == nullptr)
            {
                return false;
            }
            ref = SelectionRef::controller(found->second);
            return true;
        }
        if ((token & kLocatorDomain) == kLocatorDomain) {
            const auto found = locator_ids_by_token.find(token);
            if (found == locator_ids_by_token.end()
                || components.locator(found->second) == nullptr)
            {
                return false;
            }
            ref = SelectionRef::locator(found->second);
            return true;
        }
        return false;
    }

    bool enabled = true;

private:
    static constexpr std::uint32_t kControllerDomain = 0x40000000u;
    static constexpr std::uint32_t kLocatorDomain = 0x80000000u;

    static void report(const Rhi::Status& status) {
        if (!status) {
            std::cerr << "RigPickingFeature: " << status.message << '\n';
        }
    }

    static bool register_shapes(ViewerAdapter& adapter) {
        const auto controllers = rig_shapes::register_controller_shapes(adapter);
        const auto locators = rig_shapes::register_locator_shapes(adapter);
        report(controllers);
        report(locators);
        return controllers && locators;
    }

    bool prepare(ViewerAdapter& adapter) {
        const auto status = adapter.prepare_pick_layer({
            .layer = Rhi::PickLayer::rig,
            .nodes_per_pixel = nodes_per_pixel,
        });
        report(status);
        return static_cast<bool>(status);
    }

    std::uint32_t controller_token(ComponentId id) {
        const auto found = controller_tokens.find(id.value);
        if (found != controller_tokens.end()) {
            return found->second;
        }
        const auto token = kControllerDomain | next_controller_token++;
        controller_tokens.emplace(id.value, token);
        controller_ids_by_token.emplace(token, id);
        return token;
    }

    std::uint32_t locator_token(ComponentId id) {
        const auto found = locator_tokens.find(id.value);
        if (found != locator_tokens.end()) {
            return found->second;
        }
        const auto token = kLocatorDomain | next_locator_token++;
        locator_tokens.emplace(id.value, token);
        locator_ids_by_token.emplace(token, id);
        return token;
    }

    SceneGraphContext& graph_context;
    const ComponentManager& components;
    PickRequest pick{Rhi::PickLayer::rig, "RigPickingFeature"};
    std::vector<Rhi::RigPickShape> controllers;
    std::vector<Rhi::RigPickLocator> locators;
    std::unordered_map<std::uint64_t, std::uint32_t> controller_tokens;
    std::unordered_map<std::uint32_t, ComponentId> controller_ids_by_token;
    std::unordered_map<std::uint64_t, std::uint32_t> locator_tokens;
    std::unordered_map<std::uint32_t, ComponentId> locator_ids_by_token;
    std::uint32_t next_controller_token = 1;
    std::uint32_t next_locator_token = 1;
    std::uint32_t nodes_per_pixel = 8;
    bool ready = false;
};

} // namespace ORL
