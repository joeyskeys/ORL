#pragma once

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>

#include <glm/mat4x4.hpp>
#include <glm/vec4.hpp>

#include "built_in_shader/common.h"
#include "camera_navigator.hpp"
#include "concepts/mesh.h"
#include "utils/sizeable.hpp"
#include "vk_ins/shader_module_pack.hpp"
#include "vp/feature.hpp"

namespace ORL
{

struct OrthoGridUBO : public vkkk::Sizeable<OrthoGridUBO> {
    glm::vec4 origin_px_ppu{0.0f};
    glm::vec4 viewport_cell{0.0f, 0.0f, 1.0f, 1.0f};
    glm::vec4 line_color{0.22f, 0.22f, 0.22f, 1.0f};
    glm::vec4 axis_color{0.08f, 0.08f, 0.08f, 2.0f};
};

class OrthoGridFeature final : public vkkk::vp::ViewportFeature<vkkk::vp::ViewportPhase::Scene> {
public:
    OrthoGridFeature(const CameraNavigator& navigator, std::filesystem::path shader_dir)
        : navigator(navigator)
        , shader_dir(std::move(shader_dir))
    {
    }

    void on_attach(vkkk::Context& context, vk::Extent2D extent) {
        viewport_extent = extent;
        ready = create_pipeline(context) && create_triangle(context);
        if (!ready) {
            std::cerr << "OrthoGridFeature: screen-space grid pipeline is unavailable\n";
        }
    }

    void on_resize(vkkk::Context&, vk::Extent2D extent) {
        viewport_extent = extent;
    }

    void on_record(vkkk::Context& context, vk::raii::CommandBuffer& cmd, uint32_t image_index) {
        if (!ready || !visible || !navigator.orthographic) {
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

        OrthoGridUBO ubo{};
        ubo.origin_px_ppu = {
            (ndc.x * 0.5f + 0.5f) * width,
            (ndc.y * 0.5f + 0.5f) * height,
            ppu_x,
            ppu_y,
        };
        ubo.viewport_cell = {width, height, nice_step(32.0f / std::max(ppu_x, 1.0e-4f)), 1.0f};
        ubo.line_color = {0.22f, 0.22f, 0.22f, 1.0f};
        ubo.axis_color = {0.08f, 0.08f, 0.08f, 2.0f};

        context.sync_ubo(kPipeline, kUboBlock, &ubo, image_index);
        if (context.bind(cmd, kPipeline, image_index)) {
            context.draw(cmd, kPipeline, kTriangle, 1);
        }
    }

    bool visible = true;

private:
    static constexpr const char* kPipeline = "orl_ortho_grid";
    static constexpr const char* kUboBlock = "OrthoGridUBO";
    static constexpr const char* kTriangle = "orl_ortho_grid_tri";

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

    bool create_pipeline(vkkk::Context& context) {
        if (context.pipelines.contains(kPipeline)) {
            return true;
        }

        vkkk::ShaderModule vert_module;
        vkkk::ShaderModule frag_module;
        if (!vert_module.load(shader_dir / "ortho_grid.vert", vk::ShaderStageFlagBits::eVertex)
            || !frag_module.load(shader_dir / "ortho_grid.frag", vk::ShaderStageFlagBits::eFragment))
        {
            return false;
        }

        vkkk::ShaderModulePack pack;
        if (!pack.add_shader_module(vert_module) || !pack.add_shader_module(frag_module)) {
            return false;
        }

        vkkk::PipelineOption option;
        option.setup_input_assembly(vk::PrimitiveTopology::eTriangleList, false);
        option.setup_multisampling(false, vk::SampleCountFlagBits::e1);
        option.setup_rasterizer(false, false, vk::PolygonMode::eFill, 1.0f,
            vk::CullModeFlagBits::eNone, vk::FrontFace::eCounterClockwise, false);
        option.setup_depth_stencil(false, false, vk::CompareOp::eLessOrEqual, false, false);
        return context.create_pipeline(kPipeline, pack, option, {vkkk::VERTEX});
    }

    bool create_triangle(vkkk::Context& context) {
        if (context.meshes.contains(kTriangle)) {
            return true;
        }
        constexpr float vertices[] = {
            0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f,
        };
        constexpr uint32_t indices[] = {0, 1, 2};
        vkkk::Mesh mesh({vkkk::VERTEX});
        mesh.load(3, reinterpret_cast<const char*>(vertices), sizeof(vertices),
            1, reinterpret_cast<const char*>(indices), sizeof(indices));
        return context.load_mesh(kTriangle, mesh);
    }

    const CameraNavigator& navigator;
    std::filesystem::path shader_dir;
    vk::Extent2D viewport_extent{};
    bool ready = false;
};

} // namespace ORL
