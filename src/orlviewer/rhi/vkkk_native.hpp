#pragma once

#include <cstdint>
#include <functional>

#include "vk_ins/context.hpp"
#if ORL_USE_QT6
#include "gui/qt_backend.hpp"
#else
#include "gui/glfw_backend.hpp"
#endif

#include "vkkk_adapter.hpp"

// vkkk-only bridge for viewer features that still record native vkkk commands.
// It is not part of the StaticAdapter contract and must shrink as features
// move onto adapter calls; the mt build never includes it.
namespace ORL::Rhi {

#if ORL_USE_QT6
using VkkkWindow = vkkk::QtBackend;
#else
using VkkkWindow = vkkk::GlfwBackend;
#endif

using VkkkRecordFn =
    std::function<void(vk::raii::CommandBuffer&, std::uint32_t image_index)>;

// Recorded by VkkkAdapter::end_frame around the adapter-owned scene pass:
// before_scene outside any pass, scene inside the cleared scene pass after the
// adapter's own draws, overlay in a color-load pass without depth.
struct VkkkPassHooks {
    VkkkRecordFn before_scene;
    VkkkRecordFn scene;
    VkkkRecordFn overlay;
};

struct VkkkNative {
    static VkkkWindow& window(VkkkAdapter& adapter);
    static vkkk::Context& context(VkkkAdapter& adapter);
    // Valid only between a successful begin_frame and the matching end_frame.
    static const vkkk::Context::Frame* active_frame(const VkkkAdapter& adapter);
    static void set_pass_hooks(VkkkAdapter& adapter, VkkkPassHooks hooks);
};

} // namespace ORL::Rhi
