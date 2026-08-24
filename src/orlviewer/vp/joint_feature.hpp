#pragma once

#include <algorithm>
#include <array>
#include <filesystem>
#include <iostream>
#include <string>

#include "built_in_shader/common.h"
#include "component_manager.hpp"
#include "concepts/camera.h"
#include "concepts/line.h"
#include "concepts/point.h"
#include "utils/sizeable.hpp"
#include "vk_ins/shader_module_pack.hpp"
#include "vp/feature.hpp"

namespace ORL
{

struct PointSizeUBO : public vkkk::Sizeable<PointSizeUBO> {
    glm::vec4 value{10.0f, 0.0f, 0.0f, 0.0f};
};

struct PointColorUBO : public vkkk::Sizeable<PointColorUBO> {
    glm::vec4 value{0.95f, 0.78f, 0.28f, 1.0f};
};

struct LineWidthUBO : public vkkk::Sizeable<LineWidthUBO> {
    glm::vec4 value{2.5f, 0.0f, 0.0f, 0.0f};
};

struct LineColorUBO : public vkkk::Sizeable<LineColorUBO> {
    glm::vec4 value{0.62f, 0.44f, 0.14f, 1.0f};
};

class JointFeature final : public vkkk::vp::ViewportFeature<vkkk::vp::ViewportPhase::Scene> {
public:
    JointFeature(const ComponentManager& components, const vkkk::Camera& camera,
        std::filesystem::path shader_dir)
        : components(components)
        , camera(camera)
        , shader_dir(std::move(shader_dir))
    {
    }

    void on_attach(vkkk::Context& context, vk::Extent2D) {
        points_ready = create_point_pipeline(context) && create_dummy_points(context)
            && context.resize_pipeline_ssbo(kPointPipeline, kJointsBlock, 1)
            && context.alloc_pipeline_ssbo(kPointPipeline, kJointsBlock);
        if (!points_ready) {
            std::cerr << "JointFeature: point-list pipeline is unavailable\n";
        }

        lines_ready = create_line_pipeline(context) && create_dummy_lines(context)
            && context.resize_pipeline_ssbo(kLinePipeline, kJointsBlock, 1)
            && context.alloc_pipeline_ssbo(kLinePipeline, kJointsBlock);
        if (!lines_ready) {
            std::cerr << "JointFeature: line-list pipeline is unavailable\n";
        }
    }

    void on_record(vkkk::Context& context, vk::raii::CommandBuffer& cmd, uint32_t image_index) {
        const auto joints = components.packed_joints();
        if (joints.empty()) {
            return;
        }

        record_lines(context, cmd, image_index, joints);
        record_points(context, cmd, image_index, joints);
    }

private:
    static constexpr const char* kPointPipeline = "orl_joints";
    static constexpr const char* kLinePipeline = "orl_joint_lines";
    static constexpr const char* kJointsBlock = "Joints";
    static constexpr const char* kPointSizeBlock = "PointSizeUBO";
    static constexpr const char* kPointColorBlock = "PointColorUBO";
    static constexpr const char* kLineWidthBlock = "LineWidthUBO";
    static constexpr const char* kLineColorBlock = "LineColorUBO";
    static constexpr const char* kDummyPoints = "orl_joint_draw_points";
    static constexpr const char* kDummyLines = "orl_joint_draw_lines";

    bool create_point_pipeline(vkkk::Context& context) {
        if (context.pipelines.contains(kPointPipeline)) {
            return true;
        }

        const auto vert_path = shader_dir / "point.vert";
        const auto frag_path = shader_dir / "point.frag";
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
        option.setup_input_assembly(vk::PrimitiveTopology::ePointList, false);
        option.setup_multisampling(false, vk::SampleCountFlagBits::e1);
        option.setup_rasterizer(false, false, vk::PolygonMode::eFill, 1.0f,
            vk::CullModeFlagBits::eNone, vk::FrontFace::eCounterClockwise, false);
        option.setup_depth_stencil(true, true, vk::CompareOp::eLessOrEqual, false, false);
        return context.create_pipeline(kPointPipeline, pack, option, {vkkk::VERTEX});
    }

    bool create_line_pipeline(vkkk::Context& context) {
        if (context.pipelines.contains(kLinePipeline)) {
            return true;
        }

        const auto vert_path = shader_dir / "line.vert";
        const auto frag_path = shader_dir / "line.frag";
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

        const float baked_width = context.wide_lines_enabled
            ? std::clamp(line_width.value.x, context.line_width_range[0],
                context.line_width_range[1])
            : 1.0f;
        vkkk::PipelineOption option;
        option.setup_input_assembly(vk::PrimitiveTopology::eLineList, false);
        option.setup_multisampling(false, vk::SampleCountFlagBits::e1);
        option.setup_rasterizer(false, false, vk::PolygonMode::eFill, baked_width,
            vk::CullModeFlagBits::eNone, vk::FrontFace::eCounterClockwise, false);
        option.setup_depth_stencil(true, false, vk::CompareOp::eLessOrEqual, false, false);
        if (context.wide_lines_enabled) {
            option.dynamic_states.push_back(vk::DynamicState::eLineWidth);
            option.dynamic_info.dynamicStateCount =
                static_cast<uint32_t>(option.dynamic_states.size());
            option.dynamic_info.pDynamicStates = option.dynamic_states.data();
            line_width_dynamic = true;
        }
        return context.create_pipeline(kLinePipeline, pack, option, {vkkk::VERTEX});
    }

    bool create_dummy_points(vkkk::Context& context) {
        if (context.points.contains(kDummyPoints)) {
            return true;
        }
        const std::array<float, 3> vertex{0.0f, 0.0f, 0.0f};
        vkkk::Points points({vkkk::VERTEX});
        points.load(1, reinterpret_cast<const char*>(vertex.data()),
            static_cast<uint32_t>(vertex.size() * sizeof(float)));
        return context.load_points(kDummyPoints, points);
    }

    bool create_dummy_lines(vkkk::Context& context) {
        if (context.lines.contains(kDummyLines)) {
            return true;
        }
        constexpr float vertices[] = {
            0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f,
        };
        constexpr uint32_t indices[] = {0, 1};
        vkkk::Lines lines({vkkk::VERTEX});
        lines.load(2, reinterpret_cast<const char*>(vertices), sizeof(vertices),
            2, reinterpret_cast<const char*>(indices), sizeof(indices));
        return context.load_lines(kDummyLines, lines);
    }

    void record_lines(vkkk::Context& context, vk::raii::CommandBuffer& cmd, uint32_t image_index,
        const std::vector<orlviewer::Joint>& joints)
    {
        if (!lines_ready) {
            return;
        }
        if (!context.resize_pipeline_ssbo(kLinePipeline, kJointsBlock, joints.size())) {
            return;
        }

        LineWidthUBO width = line_width;
        if (context.wide_lines_enabled) {
            width.value.x = std::clamp(width.value.x, context.line_width_range[0],
                context.line_width_range[1]);
        }
        else {
            width.value.x = 1.0f;
        }
        context.sync_ubo(kLinePipeline, vkkk::buf::CameraUBO, &camera.ubo_data, image_index);
        context.sync_ubo(kLinePipeline, kLineWidthBlock, &width, image_index);
        context.sync_ubo(kLinePipeline, kLineColorBlock, &line_color, image_index);
        context.sync_ssbo(kLinePipeline, kJointsBlock, joints.data(), image_index,
            static_cast<uint32_t>(joints.size() * orlviewer::kJointStride));
        if (context.bind(cmd, kLinePipeline, image_index)) {
            if (line_width_dynamic) {
                cmd.setLineWidth(width.value.x);
            }
            context.draw_lines(cmd, kDummyLines, 0, static_cast<uint32_t>(joints.size()));
        }
    }

    void record_points(vkkk::Context& context, vk::raii::CommandBuffer& cmd, uint32_t image_index,
        const std::vector<orlviewer::Joint>& joints)
    {
        if (!points_ready) {
            return;
        }
        if (!context.resize_pipeline_ssbo(kPointPipeline, kJointsBlock, joints.size())) {
            return;
        }

        PointSizeUBO size = point_size;
        size.value.x = std::clamp(size.value.x, context.point_size_range[0],
            context.point_size_range[1]);
        context.sync_ubo(kPointPipeline, vkkk::buf::CameraUBO, &camera.ubo_data, image_index);
        context.sync_ubo(kPointPipeline, kPointSizeBlock, &size, image_index);
        context.sync_ubo(kPointPipeline, kPointColorBlock, &point_color, image_index);
        context.sync_ssbo(kPointPipeline, kJointsBlock, joints.data(), image_index,
            static_cast<uint32_t>(joints.size() * orlviewer::kJointStride));
        if (context.bind(cmd, kPointPipeline, image_index)) {
            context.draw_points(cmd, kDummyPoints, 0,
                static_cast<uint32_t>(joints.size()));
        }
    }

    const ComponentManager& components;
    const vkkk::Camera& camera;
    std::filesystem::path shader_dir;
    PointSizeUBO point_size{};
    PointColorUBO point_color{};
    LineWidthUBO line_width{};
    LineColorUBO line_color{};
    bool points_ready = false;
    bool lines_ready = false;
    bool line_width_dynamic = false;
};

} // namespace ORL
