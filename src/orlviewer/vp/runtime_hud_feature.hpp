#pragma once

#if ORL_USE_QT6
#include <string>

#include "gui/qt_backend.hpp"
#else
#include <imgui.h>

#include "gui/gui.h"
#endif

#include "runtime_config.hpp"
#include "vp/feature.hpp"

namespace ORL
{

class RuntimeHudFeature final : public vkkk::vp::ViewportFeature<vkkk::vp::ViewportPhase::ScreenOverlay> {
public:
#if !ORL_USE_QT6
    ~RuntimeHudFeature() {
        hud.shutdown();
    }
#endif

    void on_attach(vkkk::Context& context, vk::Extent2D) {
#if ORL_USE_QT6
        if (auto* qt = dynamic_cast<vkkk::QtBackend*>(context.window())) {
            qt->set_status(std::string("Device: ") + compute_device_label());
        }
#else
        hud.init(&context);
#endif
    }

    void on_update(vkkk::Context& context, const vkkk::Context::Frame&) {
#if ORL_USE_QT6
        if (auto* qt = dynamic_cast<vkkk::QtBackend*>(context.window())) {
            qt->set_status(std::string("Device: ") + compute_device_label());
        }
#else
        hud.begin_frame();
        ImGui::SetNextWindowPos(ImVec2(12.0f, 12.0f), ImGuiCond_Always);
        ImGui::SetNextWindowBgAlpha(0.7f);
        ImGui::Begin("Compute device", nullptr,
            ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize
                | ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoMove);
        ImGui::Text("Device: %s", compute_device_label());
        ImGui::End();
#endif
    }

    void on_record(vkkk::Context&, vk::raii::CommandBuffer& cmd, uint32_t) {
#if !ORL_USE_QT6
        hud.render(static_cast<VkCommandBuffer>(*cmd));
#else
        (void)cmd;
#endif
    }

private:
#if !ORL_USE_QT6
    vkkk::ImGuiHud hud;
#endif
};

} // namespace ORL
