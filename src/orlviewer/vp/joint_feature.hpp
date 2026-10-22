#pragma once

#include <cstddef>
#include <cstdint>
#include <iostream>
#include <span>
#include <vector>

#include <glm/vec3.hpp>

#include "component_manager.hpp"
#include "../scene_graph_context.hpp"
#include "ops/create_joint_op.hpp"
#include "vp/rhi_feature.hpp"

namespace ORL
{

class JointFeature final : public RhiFeature {
public:
    JointFeature(SceneGraphContext& graph_context, const ComponentManager& components)
        : graph_context(graph_context)
        , components(components)
    {
    }

    void set_preview_source(const CreateJointOp& source) {
        preview_source = &source;
    }

    void on_attach(ViewerAdapter& adapter, Rhi::Extent2D) {
        const auto status = adapter.prepare_joints({
            .joint_stride = orlviewer::kJointStride,
            .line_width = line_width,
        });
        if (!status) {
            std::cerr << "JointFeature: " << status.message << '\n';
        }
    }

    void on_submit(ViewerAdapter& adapter, const Rhi::FrameState&) {
        bool gpu_joints = false;
        std::uint32_t joint_count = 0;
        const auto device_joints = graph_context.computed_joints_device();
        if (device_joints.has_value()
            && graph_context.computed_joints_device_count() != 0)
        {
            joint_count = static_cast<std::uint32_t>(
                graph_context.computed_joints_device_count());
            gpu_joints = static_cast<bool>(adapter.upload_device_joints(
                {device_joints->device_ptr, device_joints->bytes}, joint_count));
        }

        const auto* preview = preview_source == nullptr
            ? nullptr : &preview_source->preview();
        std::vector<orlviewer::Joint> joints;
        if (!gpu_joints) {
            joints = components.packed_joints();
            if (preview != nullptr && preview->visible) {
                append_preview(joints, *preview);
            }
            if (joints.empty()) {
                return;
            }
            joint_count = static_cast<std::uint32_t>(joints.size());
        }

        auto draw = make_draw();
        draw.use_device_joints = gpu_joints;
        draw.joint_count = joint_count;
        draw.joints = std::as_bytes(std::span{joints});
        report(adapter.draw_joints(draw));
        if (gpu_joints && preview != nullptr && preview->visible) {
            draw_gpu_preview(adapter, *preview);
        }
    }

private:
    Rhi::JointDraw make_draw() const {
        Rhi::JointDraw draw;
        draw.point_size = point_size;
        draw.point_color = point_color;
        draw.line_width = line_width;
        draw.line_color = line_color;
        return draw;
    }

    static void report(const Rhi::Status& status) {
        if (!status) {
            std::cerr << "JointFeature: " << status.message << '\n';
        }
    }

    void append_preview(std::vector<orlviewer::Joint>& joints,
        const CreateJointOp::Preview& preview) const
    {
        orlviewer::Joint pending = orlviewer::make_identity_joint();
        const auto parent_index = preview.parent
            ? components.joint_index(preview.parent) : -1;
        pending.parent = parent_index >= 0
            && static_cast<std::size_t>(parent_index) < joints.size()
            ? parent_index : -1;
        const auto local = orlviewer::world_to_local(
            joints, pending.parent, preview.world);
        pending.translation[0] = local.x;
        pending.translation[1] = local.y;
        pending.translation[2] = local.z;
        pending.selected = 1;
        joints.push_back(pending);
    }

    // The device joints stay in the joint buffers, so the pending joint is
    // drawn separately from host data: a parent-to-pending bone plus its point.
    void draw_gpu_preview(ViewerAdapter& adapter, const CreateJointOp::Preview& preview) {
        const auto packed = components.packed_joints();
        const auto parent_index = preview.parent
            ? components.joint_index(preview.parent) : -1;
        if (parent_index >= 0
            && static_cast<std::size_t>(parent_index) < packed.size())
        {
            const auto parent_world = glm::vec3{
                orlviewer::joint_world_matrix(packed, parent_index)[3]};
            auto parent = orlviewer::make_identity_joint();
            parent.selected = 1;
            parent.translation[0] = parent_world.x;
            parent.translation[1] = parent_world.y;
            parent.translation[2] = parent_world.z;

            auto pending = orlviewer::make_identity_joint();
            pending.parent = 0;
            pending.selected = 1;
            pending.translation[0] = preview.world.x - parent_world.x;
            pending.translation[1] = preview.world.y - parent_world.y;
            pending.translation[2] = preview.world.z - parent_world.z;

            const std::vector<orlviewer::Joint> line_joints{parent, pending};
            auto draw = make_draw();
            draw.points = false;
            draw.joint_count = static_cast<std::uint32_t>(line_joints.size());
            draw.joints = std::as_bytes(std::span{line_joints});
            report(adapter.draw_joints(draw));
        }

        auto point = orlviewer::make_identity_joint();
        point.selected = 1;
        point.translation[0] = preview.world.x;
        point.translation[1] = preview.world.y;
        point.translation[2] = preview.world.z;
        const std::vector<orlviewer::Joint> point_joints{point};
        auto draw = make_draw();
        draw.lines = false;
        draw.joint_count = 1;
        draw.joints = std::as_bytes(std::span{point_joints});
        report(adapter.draw_joints(draw));
    }

    SceneGraphContext& graph_context;
    const ComponentManager& components;
    const CreateJointOp* preview_source = nullptr;
    float point_size = 10.0f;
    Rhi::Vec4 point_color{0.95f, 0.78f, 0.28f, 1.0f};
    float line_width = 2.5f;
    Rhi::Vec4 line_color{0.62f, 0.44f, 0.14f, 1.0f};
};

} // namespace ORL
