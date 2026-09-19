#pragma once

#include <algorithm>
#include <filesystem>
#include <iostream>
#include <utility>

#include <glm/geometric.hpp>
#include <glm/mat4x4.hpp>
#include <glm/vec4.hpp>

#include "concepts/line.h"
#include "ops/move_op.hpp"
#include "ops/rotate_op.hpp"
#include "ops/scale_op.hpp"
#include "utils/sizeable.hpp"
#include "vk_ins/shader_module_pack.hpp"
#include "vp/feature.hpp"

namespace ORL
{

struct TransformGuideModelUBO : public vkkk::Sizeable<TransformGuideModelUBO> {
    glm::mat4 model{1.0f};
};

struct TransformGuideColorUBO : public vkkk::Sizeable<TransformGuideColorUBO> {
    glm::vec4 value{1.0f};
};

class TransformGuideFeature final
    : public vkkk::vp::ViewportFeature<
          vkkk::vp::ViewportPhase::ScreenOverlay> {
public:
    TransformGuideFeature(const MoveOp& move, const RotateOp& rotate,
        const ScaleOp& scale, const vkkk::Camera& camera,
        std::filesystem::path shader_dir)
        : move(move)
        , rotate(rotate)
        , scale(scale)
        , camera(camera)
        , shader_dir(std::move(shader_dir))
    {
    }

    void on_attach(vkkk::Context& context, vk::Extent2D) {
        ready = create_pipeline(context) && create_line(context);
        if (!ready) {
            std::cerr << "TransformGuideFeature: guide pipeline is unavailable\n";
        }
    }

    void on_record(vkkk::Context& context, vk::raii::CommandBuffer& cmd,
        uint32_t image_index)
    {
        const auto active = active_guide();
        if (!ready || !active.visible) {
            return;
        }

        glm::vec3 axis = active.axis;
        const float axis_length = glm::length(axis);
        if (axis_length < 1.0e-6f) {
            return;
        }
        axis /= axis_length;

        TransformGuideModelUBO model{};
        model.model[0] = glm::vec4{axis * guide_half_length(), 0.0f};
        model.model[3] = glm::vec4{active.origin, 1.0f};
        TransformGuideColorUBO color{};
        color.value = axis_color(active.axis_kind);

        context.sync_ubo(
            kPipeline, vkkk::buf::CameraUBO, &camera.ubo_data, image_index);
        context.sync_ubo(kPipeline, kModelBlock, &model, image_index);
        context.sync_ubo(kPipeline, kColorBlock, &color, image_index);
        if (context.bind(cmd, kPipeline, image_index)) {
            if (line_width_dynamic) {
                cmd.setLineWidth(line_width);
            }
            context.draw_lines(cmd, kLine);
        }
    }

private:
    struct GuideState {
        bool visible = false;
        TransformAxis axis_kind = TransformAxis::None;
        glm::vec3 axis{0.0f};
        glm::vec3 origin{0.0f};
    };

    GuideState active_guide() const {
        if (move.guide_visible()) {
            return {
                true, move.guide_axis_kind(),
                move.guide_axis(), move.guide_origin(),
            };
        }
        if (rotate.guide_visible()) {
            return {
                true, rotate.guide_axis_kind(),
                rotate.guide_axis(), rotate.guide_origin(),
            };
        }
        if (scale.guide_visible()) {
            return {
                true, scale.guide_axis_kind(),
                scale.guide_axis(), scale.guide_origin(),
            };
        }
        return {};
    }

    static constexpr const char* kPipeline = "orl_transform_guide";
    static constexpr const char* kModelBlock = "ModelUBO";
    static constexpr const char* kColorBlock = "ColorUBO";
    static constexpr const char* kLine = "orl_transform_guide_line";

    static glm::vec4 axis_color(TransformAxis axis) {
        switch (axis) {
        case TransformAxis::X:
            return {0.95f, 0.2f, 0.25f, 1.0f};
        case TransformAxis::Y:
            return {0.25f, 0.9f, 0.35f, 1.0f};
        case TransformAxis::Z:
            return {0.25f, 0.45f, 1.0f, 1.0f};
        default:
            return {1.0f, 1.0f, 1.0f, 1.0f};
        }
    }

    float guide_half_length() const {
        return std::max(10.0f, camera.far);
    }

    bool create_pipeline(vkkk::Context& context) {
        if (context.pipelines.contains(kPipeline)) {
            return true;
        }

        vkkk::ShaderModule vert_module;
        vkkk::ShaderModule frag_module;
        if (!context.load_shader(
                vert_module, shader_dir / "controller.vert",
                vk::ShaderStageFlagBits::eVertex)
            || !context.load_shader(
                frag_module, shader_dir / "line.frag",
                vk::ShaderStageFlagBits::eFragment))
        {
            return false;
        }

        vkkk::ShaderModulePack pack;
        if (!pack.add_shader_module(vert_module)
            || !pack.add_shader_module(frag_module))
        {
            return false;
        }

        const float width = context.wide_lines_enabled
            ? std::clamp(2.5f, context.line_width_range[0],
                context.line_width_range[1])
            : 1.0f;
        vkkk::PipelineOption option;
        option.setup_input_assembly(vk::PrimitiveTopology::eLineList, false);
        option.setup_multisampling(false, vk::SampleCountFlagBits::e1);
        option.setup_rasterizer(false, false, vk::PolygonMode::eFill, width,
            vk::CullModeFlagBits::eNone, vk::FrontFace::eCounterClockwise,
            false);
        option.setup_depth_stencil(false, false, vk::CompareOp::eLessOrEqual,
            false, false);
        if (context.wide_lines_enabled) {
            option.dynamic_states.push_back(vk::DynamicState::eLineWidth);
            option.dynamic_info.dynamicStateCount =
                static_cast<uint32_t>(option.dynamic_states.size());
            option.dynamic_info.pDynamicStates = option.dynamic_states.data();
            line_width = width;
            line_width_dynamic = true;
        }
        return context.create_pipeline(
            kPipeline, pack, option, {vkkk::VERTEX});
    }

    bool create_line(vkkk::Context& context) {
        if (context.lines.contains(kLine)) {
            return true;
        }
        vkkk::Lines line({vkkk::VERTEX});
        line.load(glm::vec3{-1.0f, 0.0f, 0.0f},
            glm::vec3{1.0f, 0.0f, 0.0f});
        return context.load_lines(kLine, line);
    }

    const MoveOp& move;
    const RotateOp& rotate;
    const ScaleOp& scale;
    const vkkk::Camera& camera;
    std::filesystem::path shader_dir;
    float line_width = 1.0f;
    bool line_width_dynamic = false;
    bool ready = false;
};

} // namespace ORL
