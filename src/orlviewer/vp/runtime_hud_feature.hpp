#pragma once

#include <imgui.h>

#include "gui/gui.h"
#include "runtime_config.hpp"
#include "vp/feature.hpp"

namespace ORL
{

class RuntimeHudFeature final : public vkkk::vp::ViewportFeature<vkkk::vp::ViewportPhase::ScreenOverlay> {
public:
    ~RuntimeHudFeature() {
        hud.shutdown();
    }

    void on_attach(vkkk::Context& context, vk::Extent2D) {
        hud.init(&context);
    }

    void on_update(vkkk::Context&, const vkkk::Context::Frame&) {
        hud.begin_frame();
        ImGui::SetNextWindowPos(ImVec2(12.0f, 12.0f), ImGuiCond_Always);
        ImGui::SetNextWindowBgAlpha(0.7f);
        ImGui::Begin("Compute device", nullptr,
            ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize
                | ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoMove);
        ImGui::Text("Device: %s", compute_device_label());
        ImGui::End();
    }

    void on_record(vkkk::Context&, vk::raii::CommandBuffer& cmd, uint32_t) {
        hud.render(static_cast<VkCommandBuffer>(*cmd));
    }

private:
    vkkk::ImGuiHud hud;
};

} // namespace ORL
