#pragma once

#include <algorithm>
#include <concepts>
#include <cstdint>
#include <tuple>
#include <utility>
#include <vector>

#include "orlviewer/rhi/vkkk_native.hpp"
#include "vp/rhi_feature.hpp"
#include "vp/viewport.hpp"

namespace ORL
{

// Features still recording native vkkk commands. They are recorded through the
// adapter's pass hooks until they move onto adapter display calls.
template <typename Feature>
concept NativeFeatureType = !RhiFeatureType<Feature>
    && vkkk::vp::ViewportFeatureType<Feature>;

template <typename Feature>
concept HostedFeatureType = RhiFeatureType<Feature> || NativeFeatureType<Feature>;

// Viewer feature host driven by the RHI adapter. The adapter owns the window,
// frame lifecycle, and passes. Per frame call: begin_frame -> update ->
// end_frame. end_frame submits RhiFeature draws in feature order, then the
// adapter records them before any native feature in the same pass.
template <HostedFeatureType... FeatureTypes>
class ViewportHost {
public:
    explicit ViewportHost(ViewerAdapter& adapter)
        : adapter(adapter)
        , ctx(Rhi::VkkkNative::context(adapter))
        , viewport_extent{ctx.extent().width, ctx.extent().height}
    {
        Rhi::VkkkPassHooks hooks;
        if constexpr (has_native_phase<vkkk::vp::ViewportPhase::Picking>) {
            hooks.before_scene = [this](vk::raii::CommandBuffer& cmd, std::uint32_t image_index) {
                record_phase<vkkk::vp::ViewportPhase::Picking>(cmd, image_index);
            };
        }
        if constexpr (has_native_phase<vkkk::vp::ViewportPhase::Scene>) {
            hooks.scene = [this](vk::raii::CommandBuffer& cmd, std::uint32_t image_index) {
                record_phase<vkkk::vp::ViewportPhase::Scene>(cmd, image_index);
            };
        }
        if constexpr (has_native_phase<vkkk::vp::ViewportPhase::ScreenOverlay>) {
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
        if (frame.extent != viewport_extent) {
            viewport_extent = frame.extent;
            resize_all();
        }
        return status;
    }

    void update(const Rhi::FrameState& frame) {
        const auto* native_frame = Rhi::VkkkNative::active_frame(adapter);
        if (native_frame == nullptr) {
            return;
        }
        ([&] {
            for (auto& entry : store<FeatureTypes>().entries) {
                if constexpr (RhiFeatureType<FeatureTypes>) {
                    entry.feature.on_update(adapter, frame);
                }
                else {
                    entry.feature.on_update(ctx, *native_frame);
                }
            }
        }(), ...);
    }

    Rhi::Status end_frame(Rhi::FrameState& frame) {
        ([&] {
            if constexpr (RhiFeatureType<FeatureTypes>) {
                for (auto& entry : store<FeatureTypes>().entries) {
                    entry.feature.on_submit(adapter, frame);
                }
            }
        }(), ...);
        return adapter.end_frame(frame, true);
    }

    template <HostedFeatureType Feature, typename... Args>
        requires ((std::same_as<Feature, FeatureTypes> || ...))
    vkkk::vp::FeatureHandle<Feature> add_feature(Args&&... args) {
        const vkkk::vp::FeatureHandle<Feature> handle{next_feature_id++};
        auto& entries = store<Feature>().entries;
        entries.push_back(vkkk::vp::FeatureEntry<Feature>{
            .id = handle.id,
            .feature = Feature{std::forward<Args>(args)...},
        });

        auto& feature = entries.back().feature;
        if constexpr (RhiFeatureType<Feature>) {
            feature.on_attach(adapter, viewport_extent);
            feature.on_resize(adapter, viewport_extent);
        }
        else {
            feature.on_attach(ctx, native_extent());
            feature.on_resize(ctx, native_extent());
        }
        return handle;
    }

    template <HostedFeatureType Feature>
        requires ((std::same_as<Feature, FeatureTypes> || ...))
    Feature* find_feature(vkkk::vp::FeatureHandle<Feature> handle) {
        auto& entries = store<Feature>().entries;
        const auto it = std::find_if(entries.begin(), entries.end(),
            [handle](const auto& entry) { return entry.id == handle.id; });
        return it == entries.end() ? nullptr : &it->feature;
    }

    template <HostedFeatureType Feature>
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
        return viewport_extent;
    }

private:
    template <typename Feature, vkkk::vp::ViewportPhase Phase>
    static constexpr bool native_in_phase() {
        if constexpr (NativeFeatureType<Feature>) {
            return Feature::phase == Phase;
        }
        else {
            return false;
        }
    }

    template <vkkk::vp::ViewportPhase Phase>
    static constexpr bool has_native_phase = (native_in_phase<FeatureTypes, Phase>() || ...);

    template <typename Feature>
    vkkk::vp::FeatureStore<Feature>& store() {
        return std::get<vkkk::vp::FeatureStore<Feature>>(feature_stores);
    }

    vk::Extent2D native_extent() const {
        return {viewport_extent.width, viewport_extent.height};
    }

    template <vkkk::vp::ViewportPhase Phase>
    void record_phase(vk::raii::CommandBuffer& cmd, std::uint32_t image_index) {
        ([&] {
            if constexpr (native_in_phase<FeatureTypes, Phase>()) {
                for (auto& entry : store<FeatureTypes>().entries) {
                    entry.feature.on_record(ctx, cmd, image_index);
                }
            }
        }(), ...);
    }

    void resize_all() {
        ([&] {
            for (auto& entry : store<FeatureTypes>().entries) {
                if constexpr (RhiFeatureType<FeatureTypes>) {
                    entry.feature.on_resize(adapter, viewport_extent);
                }
                else {
                    entry.feature.on_resize(ctx, native_extent());
                }
            }
        }(), ...);
    }

    ViewerAdapter& adapter;
    vkkk::Context& ctx;
    Rhi::Extent2D viewport_extent{};
    std::uint64_t next_feature_id = 1;
    std::tuple<vkkk::vp::FeatureStore<FeatureTypes>...> feature_stores;
};

} // namespace ORL
