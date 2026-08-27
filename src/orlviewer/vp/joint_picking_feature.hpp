#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <functional>
#include <iostream>
#include <string>
#include <vector>

#include <GLFW/glfw3.h>

#include "built_in_shader/common.h"
#include "component_manager.hpp"
#include "concepts/camera.h"
#include "concepts/point.h"
#include "vk_ins/shader_module_pack.hpp"
#include "vp/feature.hpp"
#include "vp/joint_feature.hpp"

namespace ORL
{

// GPU joint pick: same point size and radial discard as JointFeature, written
// into a vkkk A-buffer so overlapping hits can be resolved on readback.
class JointPickingFeature final : public vkkk::vp::ViewportFeature<vkkk::vp::ViewportPhase::Picking> {
public:
    JointPickingFeature(const ComponentManager& components, const vkkk::Camera& camera,
        std::filesystem::path shader_dir, uint32_t nodes_per_pixel = 4)
        : components(components)
        , camera(camera)
        , shader_dir(std::move(shader_dir))
        , nodes_per_pixel(nodes_per_pixel)
    {
    }

    void on_attach(vkkk::Context& context, vk::Extent2D extent) {
        ready = nodes_per_pixel != 0 && create_pipeline(context) && create_dummy_points(context)
            && context.resize_pipeline_ssbo(kPipeline, kJointsBlock, 1)
            && context.alloc_pipeline_ssbo(kPipeline, kJointsBlock)
            && resize_buffers(context, extent);
        if (ready) {
            std::cout << "Joint picking: GPU\n";
        }
        else {
            std::cout << "Joint picking: CPU (GPU prepare failed)\n";
        }
    }

    void on_resize(vkkk::Context& context, vk::Extent2D extent) {
        pick_pending = false;
        last_render_serial = 0;
        ready = ready && resize_buffers(context, extent);
    }

    void on_update(vkkk::Context& context, const vkkk::Context::Frame& frame) {
        current_serial = frame.serial;
        if (!ready) {
            return;
        }

        if (pick_pending && last_render_serial >= pending_serial) {
            std::vector<uint32_t> ids;
            bool overflow = false;
            if (context.read_abuffer_pixel(kABufferName, last_image_index, pending_x, pending_y,
                    ids, overflow)
                && hit_callback)
            {
                hit_callback(ids, overflow);
            }
            pick_pending = false;
        }

        if (!want_pick || context.get_window() == nullptr) {
            return;
        }

        int window_width = 0;
        int window_height = 0;
        glfwGetWindowSize(context.get_window(), &window_width, &window_height);
        const auto extent = context.extent();
        if (window_width > 0 && window_height > 0 && extent.width > 0 && extent.height > 0) {
            pending_x = std::min(static_cast<uint32_t>(std::floor(
                cursor_x * static_cast<double>(extent.width) / window_width)), extent.width - 1);
            pending_y = std::min(static_cast<uint32_t>(std::floor(
                cursor_y * static_cast<double>(extent.height) / window_height)), extent.height - 1);
            pending_serial = frame.serial;
            pick_pending = true;
        }
        want_pick = false;
    }

    void on_record(vkkk::Context& context, vk::raii::CommandBuffer& cmd, uint32_t image_index) {
        if (!ready || !enabled || !pick_pending || !context.clear_abuffer(kABufferName, image_index)) {
            return;
        }

        vkkk::PassDesc pass{};
        pass.colors.clear();
        pass.present = false;
        context.begin_pass(cmd, image_index, pass);

        const auto joints = components.packed_joints();
        if (!joints.empty()
            && context.resize_pipeline_ssbo(kPipeline, kJointsBlock, joints.size()))
        {
            PointSizeUBO size = point_size;
            size.value.x = std::clamp(size.value.x, context.point_size_range[0],
                context.point_size_range[1]);
            context.sync_ubo(kPipeline, vkkk::buf::CameraUBO, &camera.ubo_data, image_index);
            context.sync_ubo(kPipeline, kPointSizeBlock, &size, image_index);
            context.sync_ssbo(kPipeline, kJointsBlock, joints.data(), image_index,
                static_cast<uint32_t>(joints.size() * orlviewer::kJointStride));
            if (context.bind(cmd, kPipeline, image_index)) {
                context.draw_points(cmd, kDummyPoints, 0, static_cast<uint32_t>(joints.size()));
            }
        }

        context.end_pass(cmd, image_index, pass);
        context.barrier_abuffer_for_host(cmd);
        last_image_index = image_index;
        last_render_serial = current_serial;
    }

    bool available() const { return ready && enabled; }

    bool request(double x, double y) {
        if (!available()) {
            return false;
        }
        cursor_x = x;
        cursor_y = y;
        want_pick = true;
        return true;
    }

    void set_hit_callback(std::function<void(const std::vector<uint32_t>&, bool)> callback) {
        hit_callback = std::move(callback);
    }

    PointSizeUBO point_size{};
    bool enabled = true;

private:
    static constexpr const char* kPipeline = "orl_joint_picking";
    static constexpr const char* kABufferName = "orl_joint_picking_abuffer";
    static constexpr const char* kJointsBlock = "Joints";
    static constexpr const char* kPointSizeBlock = "PointSizeUBO";
    static constexpr const char* kDummyPoints = "orl_joint_pick_points";

    bool create_pipeline(vkkk::Context& context) {
        if (context.pipelines.contains(kPipeline)) {
            return true;
        }

        const auto vert_path = shader_dir / "joint_picking.vert";
        const auto frag_path = shader_dir / "joint_picking.frag";
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
        option.setup_depth_stencil(false, false, vk::CompareOp::eAlways, false, false);
        return context.create_pipeline(kPipeline, pack, option, {vkkk::VERTEX}, true, true, {},
            static_cast<vk::Format>(context.get_depth_format()));
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

    bool resize_buffers(vkkk::Context& context, vk::Extent2D extent) {
        return context.resize_abuffer(kABufferName, extent, nodes_per_pixel)
            && context.bind_pipeline_abuffer(kPipeline, kABufferName, 3, 4);
    }

    const ComponentManager& components;
    const vkkk::Camera& camera;
    std::filesystem::path shader_dir;
    std::function<void(const std::vector<uint32_t>&, bool)> hit_callback;
    uint32_t nodes_per_pixel = 4;
    uint32_t pending_x = 0;
    uint32_t pending_y = 0;
    uint32_t last_image_index = 0;
    uint64_t pending_serial = 0;
    uint64_t last_render_serial = 0;
    uint64_t current_serial = 0;
    double cursor_x = 0.0;
    double cursor_y = 0.0;
    bool want_pick = false;
    bool pick_pending = false;
    bool ready = false;
};

} // namespace ORL
