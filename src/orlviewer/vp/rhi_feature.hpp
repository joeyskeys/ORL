#pragma once

#include <cstring>

#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

#include "orlviewer/rhi/rhi_display.hpp"
#include "orlviewer/rhi/selected_adapter.hpp"

namespace ORL
{

using ViewerAdapter = Rhi::SelectedAdapter;
static_assert(Rhi::ViewerDisplayAdapter<ViewerAdapter>);

// Static base for viewport features that only talk to the selected adapter.
// on_update runs for every feature first; on_submit then issues the frame's
// adapter draws in feature order.
class RhiFeature {
public:
    void on_attach(ViewerAdapter&, Rhi::Extent2D) {}
    void on_resize(ViewerAdapter&, Rhi::Extent2D) {}
    void on_update(ViewerAdapter&, const Rhi::FrameState&) {}
    void on_submit(ViewerAdapter&, const Rhi::FrameState&) {}
};

template <typename Feature>
concept RhiFeatureType = requires(Feature feature, ViewerAdapter& adapter,
    Rhi::Extent2D extent, const Rhi::FrameState& frame)
{
    feature.on_attach(adapter, extent);
    feature.on_resize(adapter, extent);
    feature.on_update(adapter, frame);
    feature.on_submit(adapter, frame);
};

inline Rhi::Mat4 to_rhi(const glm::mat4& value) {
    Rhi::Mat4 result;
    static_assert(sizeof(result) == sizeof(value));
    std::memcpy(result.data(), &value, sizeof(result));
    return result;
}

inline Rhi::Vec4 to_rhi(const glm::vec4& value) {
    return {value.x, value.y, value.z, value.w};
}

inline Rhi::Vec3 to_rhi(const glm::vec3& value) {
    return {value.x, value.y, value.z};
}

} // namespace ORL
