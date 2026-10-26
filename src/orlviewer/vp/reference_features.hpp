#pragma once

#include <iostream>
#include <string_view>
#include <utility>

#include "vp/rhi_feature.hpp"

namespace ORL
{

namespace detail
{

inline void report_reference(std::string_view feature, const Rhi::Status& status) {
    if (!status) {
        std::cerr << feature << ": " << status.message << '\n';
    }
}

} // namespace detail

// World-space XZ grid; hidden by the viewer in orthographic views.
class WorldGridFeature final : public RhiFeature {
public:
    explicit WorldGridFeature(Rhi::WorldGridSetup setup = {})
        : setup(setup)
    {
    }

    void on_attach(ViewerAdapter& adapter, Rhi::Extent2D) {
        detail::report_reference("WorldGridFeature", adapter.prepare_world_grid(setup));
    }

    void on_submit(ViewerAdapter& adapter, const Rhi::FrameState&) {
        if (visible) {
            detail::report_reference("WorldGridFeature", adapter.draw_world_grid());
        }
    }

    bool visible = true;

private:
    Rhi::WorldGridSetup setup;
};

// Screen-corner orientation gizmo for the scene world frame.
class FrameAxisFeature final : public RhiFeature {
public:
    explicit FrameAxisFeature(Rhi::FrameAxisSetup setup)
        : setup(std::move(setup))
    {
    }

    void on_attach(ViewerAdapter& adapter, Rhi::Extent2D) {
        detail::report_reference("FrameAxisFeature", adapter.prepare_frame_axis(setup));
    }

    void on_submit(ViewerAdapter& adapter, const Rhi::FrameState&) {
        if (visible) {
            detail::report_reference("FrameAxisFeature", adapter.draw_frame_axis());
        }
    }

    bool visible = true;

private:
    Rhi::FrameAxisSetup setup;
};

} // namespace ORL
