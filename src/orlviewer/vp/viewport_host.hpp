#pragma once

#include <algorithm>
#include <concepts>
#include <cstdint>
#include <tuple>
#include <utility>
#include <vector>

#include "orlviewer/rhi/vkkk_native.hpp"
#include "vp/viewport.hpp"

namespace ORL
{

// Viewer feature host driven by the RHI adapter. The adapter owns the window,
// frame lifecycle, and the cleared scene pass; features that still record
// vkkk commands are attached here and recorded through the adapter's pass
// hooks in the same Picking -> Scene -> ScreenOverlay order as vkkk's Viewport.
// Per frame call: begin_frame -> update -> end_frame.
template <vkkk::vp::ViewportFeatureType... FeatureTypes>
class ViewportHost {
public:
    explicit ViewportHost(Rhi::VkkkAdapter& adapter)
        : adapter(adapter)
        , ctx(Rhi::VkkkNative::context(adapter))
        , viewport_extent(ctx.extent())
    {
        Rhi::VkkkPassHooks hooks;
        hooks.before_scene = [this](vk::raii::CommandBuffer& cmd, std::uint32_t image_index) {
            record_phase<vkkk::vp::ViewportPhase::Picking>(cmd, image_index);
        };
        hooks.scene = [this](vk::raii::CommandBuffer& cmd, std::uint32_t image_index) {
            record_phase<vkkk::vp::ViewportPhase::Scene>(cmd, image_index);
        };
        if constexpr (has_screen_overlay) {
            hooks.overlay = [this](vk::raii::CommandBuffer& cmd, std::uint32_t image_index) {
                record_phase<vkkk::vp::ViewportPhase::ScreenOverlay>(cmd, image_index);
            };
        }
        Rhi::VkkkNative::set_pass_hooks(adapter, std::move(hooks));
    }

    ~ViewportHost() {
        Rhi::VkkkNative::set_pass_hooks(adapter, {});
    }

    ViewportHost(const ViewportHost&) = delete;
    ViewportHost& operator=(const ViewportHost&) = delete;

    Rhi::Status begin_frame(Rhi::FrameState& frame) {
        auto status = adapter.begin_frame(frame);
        if (!status) {
            return status;
        }
        const vk::Extent2D current{frame.extent.width, frame.extent.height};
        if (current != viewport_extent) {
            viewport_extent = current;
            resize_all();
        }
        return status;
    }

    void update() {
        const auto* frame = Rhi::VkkkNative::active_frame(adapter);
        if (frame == nullptr) {
            return;
        }
        ([&] {
            for (auto& entry : store<FeatureTypes>().entries) {
                entry.feature.on_update(ctx, *frame);
            }
        }(), ...);
    }

    Rhi::Status end_frame(Rhi::FrameState& frame) {
        return adapter.end_frame(frame, true);
    }

    template <vkkk::vp::ViewportFeatureType Feature, typename... Args>
        requires ((std::same_as<Feature, FeatureTypes> || ...))
    vkkk::vp::FeatureHandle<Feature> add_feature(Args&&... args) {
        const vkkk::vp::FeatureHandle<Feature> handle{next_feature_id++};
        auto& entries = store<Feature>().entries;
        entries.push_back(vkkk::vp::FeatureEntry<Feature>{
            .id = handle.id,
            .feature = Feature{std::forward<Args>(args)...},
        });

        auto& feature = entries.back().feature;
        feature.on_attach(ctx, viewport_extent);
        feature.on_resize(ctx, viewport_extent);
        return handle;
    }

    template <vkkk::vp::ViewportFeatureType Feature>
        requires ((std::same_as<Feature, FeatureTypes> || ...))
    Feature* find_feature(vkkk::vp::FeatureHandle<Feature> handle) {
        auto& entries = store<Feature>().entries;
        const auto it = std::find_if(entries.begin(), entries.end(),
            [handle](const auto& entry) { return entry.id == handle.id; });
        return it == entries.end() ? nullptr : &it->feature;
    }

    template <vkkk::vp::ViewportFeatureType Feature>
        requires ((std::same_as<Feature, FeatureTypes> || ...))
    bool remove_feature(vkkk::vp::FeatureHandle<Feature> handle) {
        auto& entries = store<Feature>().entries;
        const auto it = std::find_if(entries.begin(), entries.end(),
            [handle](const auto& entry) { return entry.id == handle.id; });
        if (it == entries.end()) {
            return false;
        }
        entries.erase(it);
        return true;
    }

    Rhi::Extent2D extent() const {
        return {viewport_extent.width, viewport_extent.height};
    }

private:
    static constexpr bool has_screen_overlay =
        ((FeatureTypes::phase == vkkk::vp::ViewportPhase::ScreenOverlay) || ...);

    template <typename Feature>
    vkkk::vp::FeatureStore<Feature>& store() {
        return std::get<vkkk::vp::FeatureStore<Feature>>(feature_stores);
    }

    template <vkkk::vp::ViewportPhase Phase>
    void record_phase(vk::raii::CommandBuffer& cmd, std::uint32_t image_index) {
        ([&] {
            if constexpr (FeatureTypes::phase == Phase) {
                for (auto& entry : store<FeatureTypes>().entries) {
                    entry.feature.on_record(ctx, cmd, image_index);
                }
            }
        }(), ...);
    }

    void resize_all() {
        ([&] {
            for (auto& entry : store<FeatureTypes>().entries) {
                entry.feature.on_resize(ctx, viewport_extent);
            }
        }(), ...);
    }

    Rhi::VkkkAdapter& adapter;
    vkkk::Context& ctx;
    vk::Extent2D viewport_extent{};
    std::uint64_t next_feature_id = 1;
    std::tuple<vkkk::vp::FeatureStore<FeatureTypes>...> feature_stores;
};

} // namespace ORL
