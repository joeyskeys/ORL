#pragma once

#include <string>

#include "asset_mgr/scene.h"
#include "built_in_shader/phong.h"
#include "concepts/camera.h"
#include "vk_ins/shader_module_pack.hpp"
#include "vp/feature.hpp"

namespace ORL
{

class ScenePhongFeature final : public vkkk::vp::ViewportFeature<vkkk::vp::ViewportPhase::Scene> {
public:
    explicit ScenePhongFeature(vkkk::Scene& scene, bool right_handed)
        : scene(scene)
        , right_handed(right_handed)
    {
    }

    void on_attach(vkkk::Context& context, vk::Extent2D) {
        if (scene.camera == nullptr || !create_pipeline(context)) {
            return;
        }
        ready = context.resize_pipeline_ssbo(kPipelineName, vkkk::buf::PhongInstanceAttrs, 1)
            && context.alloc_pipeline_ssbo(kPipelineName, vkkk::buf::PhongInstanceAttrs);
    }

    void on_record(vkkk::Context& context, vk::raii::CommandBuffer& cmd, uint32_t image_index) {
        if (!ready || scene.camera == nullptr || scene.object_count() == 0) {
            return;
        }

        vkkk::PointLightUBO light{};
        light.vec = glm::vec4{2.0f, 3.0f, 2.0f, 1.0f};
        light.color = glm::vec4{1.0f};
        context.sync_ubo(kPipelineName, vkkk::buf::CameraUBO, &scene.camera->ubo_data, image_index);
        context.sync_ubo(kPipelineName, vkkk::buf::PointLightUBO, &light, image_index);

        scene.for_each_object([&](const std::string&, const vkkk::SceneObject& object) {
            auto attrs = material;
            attrs.model = object.model;
            context.sync_ssbo(kPipelineName, vkkk::buf::PhongInstanceAttrs, &attrs, image_index);
            if (context.bind(cmd, kPipelineName, image_index)) {
                context.draw(cmd, kPipelineName, object.mesh_name, 1);
            }
        });
    }

private:
    static constexpr const char* kPipelineName = "orl_scene_phong";

    bool create_pipeline(vkkk::Context& context) {
        if (context.pipelines.contains(kPipelineName)) {
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
        // World-CCW triangles: lookAtLH + Vulkan Y-flip keeps CCW in the
        // framebuffer; lookAtRH + Y-flip turns them CW.
        option.setup_rasterizer(false, false, vk::PolygonMode::eFill, 1.0f,
            vk::CullModeFlagBits::eBack,
            right_handed ? vk::FrontFace::eClockwise : vk::FrontFace::eCounterClockwise,
            false);
        option.setup_depth_stencil(true, true, vk::CompareOp::eLessOrEqual, false, false);
        return context.create_pipeline(kPipelineName, pack, option, {vkkk::VERTEX, vkkk::NORMAL});
    }

    vkkk::Scene& scene;
    bool right_handed = true;
    vkkk::PhongInstanceAttrs material{
        .model = glm::mat4{1.0f},
        .ambient = glm::vec4{0.08f, 0.08f, 0.08f, 1.0f},
        .diffuse = glm::vec4{0.72f, 0.72f, 0.72f, 1.0f},
        .specular = glm::vec4{0.35f, 0.35f, 0.35f, 1.0f},
        .shininess = 32.0f,
    };
    bool ready = false;
};

} // namespace ORL
