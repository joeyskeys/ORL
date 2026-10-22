#pragma once

#include <algorithm>
#include <cmath>
#include <iostream>

#include <glm/mat4x4.hpp>
#include <glm/vec2.hpp>
#include <glm/vec4.hpp>

#include "camera_navigator.hpp"
#include "vp/rhi_feature.hpp"

namespace ORL
{

class OrthoGridFeature final : public RhiFeature {
public:
    explicit OrthoGridFeature(const CameraNavigator& navigator)
        : navigator(navigator)
    {
    }

    void on_attach(ViewerAdapter& adapter, Rhi::Extent2D extent) {
        viewport_extent = extent;
        const auto status = adapter.prepare_ortho_grid();
        if (!status) {
            std::cerr << "OrthoGridFeature: " << status.message << '\n';
        }
    }

    void on_resize(ViewerAdapter&, Rhi::Extent2D extent) {
        viewport_extent = extent;
    }

    void on_submit(ViewerAdapter& adapter, const Rhi::FrameState&) {
        if (!visible || !navigator.orthographic) {
            return;
        }

        const float width = static_cast<float>(std::max(viewport_extent.width, 1u));
        const float height = static_cast<float>(std::max(viewport_extent.height, 1u));
        const float half_h = navigator.ortho_half_height();
        const float half_w = half_h * (width / height);
        const float ppu_x = width / std::max(2.0f * half_w, 1.0e-4f);
        const float ppu_y = height / std::max(2.0f * half_h, 1.0e-4f);

        const glm::vec4 clip = navigator.camera.ubo_data.proj * navigator.camera.ubo_data.view
            * glm::vec4{0.0f, 0.0f, 0.0f, 1.0f};
        const float w = std::abs(clip.w) < 1.0e-6f ? 1.0f : clip.w;
        const glm::vec2 ndc{clip.x / w, clip.y / w};

        const auto status = adapter.draw_ortho_grid({
            .origin_px_ppu = {
                (ndc.x * 0.5f + 0.5f) * width,
                (ndc.y * 0.5f + 0.5f) * height,
                ppu_x,
                ppu_y,
            },
            .viewport_cell = {width, height, nice_step(32.0f / std::max(ppu_x, 1.0e-4f)), 1.0f},
            .line_color = {0.22f, 0.22f, 0.22f, 1.0f},
            .axis_color = {0.08f, 0.08f, 0.08f, 2.0f},
        });
        if (!status) {
            std::cerr << "OrthoGridFeature: " << status.message << '\n';
        }
    }

    bool visible = true;

private:
    static float nice_step(float world) {
        world = std::max(world, 1.0e-4f);
        const float exp = std::pow(10.0f, std::floor(std::log10(world)));
        const float mantissa = world / exp;
        if (mantissa < 2.0f) {
            return exp;
        }
        if (mantissa < 5.0f) {
            return 2.0f * exp;
        }
        return 5.0f * exp;
    }

    const CameraNavigator& navigator;
    Rhi::Extent2D viewport_extent{};
};

} // namespace ORL
