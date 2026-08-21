#pragma once

#include <filesystem>
#include <iostream>
#include <string>
#include <string_view>

#include "asset_mgr/scene.h"
#include "built_in_shader/phong.h"
#include "concepts/camera.h"
#include "utils/sizeable.hpp"
#include "vk_ins/shader_module_pack.hpp"
#include "vp/feature.hpp"

namespace ORL
{

struct XRayInstanceAttrs : public vkkk::Sizeable<XRayInstanceAttrs> {
    glm::mat4 model{1.0f};
    glm::vec4 color{0.45f, 0.78f, 0.95f, 0.22f};
};

class SceneMeshFeature final : public vkkk::vp::ViewportFeature<vkkk::vp::ViewportPhase::Scene> {
public:
    SceneMeshFeature(vkkk::Scene& scene, bool right_handed, std::filesystem::path shader_dir)
        : scene(scene)
        , right_handed(right_handed)
        , shader_dir(std::move(shader_dir))
    {
    }

    bool set_display_mode(std::string_view name) {
        if (name == "phong") {
            mode = Mode::Phong;
            return true;
        }
        if (name == "xray") {
            mode = Mode::XRay;
            return true;
        }
        return false;
    }

    std::string_view display_mode() const {
        return mode == Mode::XRay ? "xray" : "phong";
    }

    void on_attach(vkkk::Context& context, vk::Extent2D) {
        if (scene.camera == nullptr) {
            return;
        }
        phong_ready = create_phong_pipeline(context)
            && context.resize_pipeline_ssbo(kPhongPipeline, vkkk::buf::PhongInstanceAttrs, 1)
            && context.alloc_pipeline_ssbo(kPhongPipeline, vkkk::buf::PhongInstanceAttrs);
        xray_ready = create_xray_pipeline(context)
            && context.resize_pipeline_ssbo(kXRayPipeline, kXRayInstanceBlock, 1)
            && context.alloc_pipeline_ssbo(kXRayPipeline, kXRayInstanceBlock);
        if (!xray_ready) {
            std::cerr << "SceneMeshFeature: x-ray pipeline is unavailable\n";
        }
    }

    void on_record(vkkk::Context& context, vk::raii::CommandBuffer& cmd, uint32_t image_index) {
        if (scene.camera == nullptr || scene.object_count() == 0) {
            return;
        }
        if (mode == Mode::XRay) {
            record_xray(context, cmd, image_index);
            return;
        }
        record_phong(context, cmd, image_index);
    }

private:
    enum class Mode {
        Phong,
        XRay,
    };

    static constexpr const char* kPhongPipeline = "orl_scene_phong";
    static constexpr const char* kXRayPipeline = "orl_scene_xray";
    static constexpr const char* kXRayInstanceBlock = "XRayInstanceAttrs";

    vk::FrontFace front_face() const {
        return right_handed ? vk::FrontFace::eClockwise : vk::FrontFace::eCounterClockwise;
    }

    bool create_phong_pipeline(vkkk::Context& context) {
        if (context.pipelines.contains(kPhongPipeline)) {
            return true;
        }

        vkkk::ShaderModule vert_module;
        vkkk::ShaderModule frag_module;
        if (!vert_module.load(vkkk::phong_vert, vk::ShaderStageFlagBits::eVertex, "orl_scene_phong_vert")
            || !frag_module.load(vkkk::phong_frag, vk::ShaderStageFlagBits::eFragment,
                "orl_scene_phong_frag"))
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
            vk::CullModeFlagBits::eBack, front_face(), false);
        option.setup_depth_stencil(true, true, vk::CompareOp::eLessOrEqual, false, false);
        return context.create_pipeline(kPhongPipeline, pack, option, {vkkk::VERTEX, vkkk::NORMAL});
    }

    bool create_xray_pipeline(vkkk::Context& context) {
        if (context.pipelines.contains(kXRayPipeline)) {
            return true;
        }

        const auto vert_path = shader_dir / "xray.vert";
        const auto frag_path = shader_dir / "xray.frag";
        vkkk::ShaderModule vert_module;
        vkkk::ShaderModule frag_module;
        if (!vert_module.load(vert_path, vk::ShaderStageFlagBits::eVertex)
            || !frag_module.load(frag_path, vk::ShaderStageFlagBits::eFragment))
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
            vk::CullModeFlagBits::eNone, front_face(), false);
        option.setup_depth_stencil(true, false, vk::CompareOp::eLessOrEqual, false, false);
        option.blend_attachment_info.blendEnable = vk::True;
        option.blend_attachment_info.srcColorBlendFactor = vk::BlendFactor::eSrcAlpha;
        option.blend_attachment_info.dstColorBlendFactor = vk::BlendFactor::eOneMinusSrcAlpha;
        option.blend_attachment_info.colorBlendOp = vk::BlendOp::eAdd;
        option.blend_attachment_info.srcAlphaBlendFactor = vk::BlendFactor::eOne;
        option.blend_attachment_info.dstAlphaBlendFactor = vk::BlendFactor::eOneMinusSrcAlpha;
        option.blend_attachment_info.alphaBlendOp = vk::BlendOp::eAdd;
        return context.create_pipeline(kXRayPipeline, pack, option, {vkkk::VERTEX, vkkk::NORMAL});
    }

    void record_phong(vkkk::Context& context, vk::raii::CommandBuffer& cmd, uint32_t image_index) {
        if (!phong_ready) {
            return;
        }

        vkkk::PointLightUBO light{};
        light.vec = glm::vec4{2.0f, 3.0f, 2.0f, 1.0f};
        light.color = glm::vec4{1.0f};
        context.sync_ubo(kPhongPipeline, vkkk::buf::CameraUBO, &scene.camera->ubo_data, image_index);
        context.sync_ubo(kPhongPipeline, vkkk::buf::PointLightUBO, &light, image_index);

        scene.for_each_object([&](const std::string&, const vkkk::SceneObject& object) {
            auto attrs = phong_material;
            attrs.model = object.model;
            context.sync_ssbo(kPhongPipeline, vkkk::buf::PhongInstanceAttrs, &attrs, image_index);
            if (context.bind(cmd, kPhongPipeline, image_index)) {
                context.draw(cmd, kPhongPipeline, object.mesh_name, 1);
            }
        });
    }

    void record_xray(vkkk::Context& context, vk::raii::CommandBuffer& cmd, uint32_t image_index) {
        if (!xray_ready) {
            record_phong(context, cmd, image_index);
            return;
        }

        context.sync_ubo(kXRayPipeline, vkkk::buf::CameraUBO, &scene.camera->ubo_data, image_index);
        scene.for_each_object([&](const std::string&, const vkkk::SceneObject& object) {
            auto attrs = xray_material;
            attrs.model = object.model;
            context.sync_ssbo(kXRayPipeline, kXRayInstanceBlock, &attrs, image_index);
            if (context.bind(cmd, kXRayPipeline, image_index)) {
                context.draw(cmd, kXRayPipeline, object.mesh_name, 1);
            }
        });
    }

    vkkk::Scene& scene;
    bool right_handed = true;
    std::filesystem::path shader_dir;
    Mode mode = Mode::Phong;
    vkkk::PhongInstanceAttrs phong_material{
        .model = glm::mat4{1.0f},
        .ambient = glm::vec4{0.08f, 0.08f, 0.08f, 1.0f},
        .diffuse = glm::vec4{0.72f, 0.72f, 0.72f, 1.0f},
        .specular = glm::vec4{0.35f, 0.35f, 0.35f, 1.0f},
        .shininess = 32.0f,
    };
    XRayInstanceAttrs xray_material{};
    bool phong_ready = false;
    bool xray_ready = false;
};

} // namespace ORL
