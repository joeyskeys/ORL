#pragma once

#include <cstdint>
#include <iostream>
#include <span>
#include <vector>

#include "component_manager.hpp"
#include "../scene_graph_context.hpp"
#include "vp/picking.hpp"
#include "vp/rhi_feature.hpp"

namespace ORL
{

// GPU joint pick: same point size and radial discard as JointFeature, written
// into an A-buffer so overlapping hits can be resolved on readback.
class JointPickingFeature final : public RhiFeature {
public:
    JointPickingFeature(SceneGraphContext& graph_context,
        const ComponentManager& components, std::uint32_t nodes_per_pixel = 4)
        : graph_context(graph_context)
        , components(components)
        , nodes_per_pixel(nodes_per_pixel)
    {
    }

    void on_attach(ViewerAdapter& adapter, Rhi::Extent2D) {
        ready = prepare(adapter);
        std::cout << (ready ? "Joint picking: GPU\n"
                            : "Joint picking: CPU (GPU prepare failed)\n");
    }

    void on_resize(ViewerAdapter& adapter, Rhi::Extent2D) {
        pick.reset();
        ready = ready && prepare(adapter);
    }

    void on_update(ViewerAdapter& adapter, const Rhi::FrameState& frame) {
        if (ready) {
            pick.update(adapter, frame);
        }
    }

    void on_submit(ViewerAdapter& adapter, const Rhi::FrameState&) {
        if (!ready || !enabled || !pick.pending()) {
            return;
        }

        bool gpu_joints = false;
        std::uint32_t joint_count = 0;
        const auto device_joints = graph_context.computed_joints_device();
        if (device_joints.has_value()
            && graph_context.computed_joints_device_count() != 0)
        {
            joint_count = static_cast<std::uint32_t>(
                graph_context.computed_joints_device_count());
            gpu_joints = static_cast<bool>(adapter.upload_device_pick_joints(
                {device_joints->device_ptr, device_joints->bytes}, joint_count));
        }

        std::vector<orlviewer::Joint> joints;
        if (!gpu_joints) {
            joints = components.packed_joints();
            joint_count = static_cast<std::uint32_t>(joints.size());
        }
        rendered_joint_ids = components.packed_joint_ids();
        if (rendered_joint_ids.size() > joint_count) {
            rendered_joint_ids.resize(joint_count);
        }

        const auto status = adapter.draw_joint_pick({
            .joints = std::as_bytes(std::span{joints}),
            .joint_count = joint_count,
            .use_device_joints = gpu_joints,
            .point_size = point_size,
        });
        if (!status) {
            std::cerr << "JointPickingFeature: " << status.message << '\n';
        }
    }

    bool available() const { return ready && enabled; }

    std::uint64_t request(double x, double y) {
        return available() ? pick.request(x, y) : 0;
    }

    void set_hit_callback(PickRequest::HitCallback callback) {
        pick.set_hit_callback(std::move(callback));
    }

    ComponentId resolve_joint_index(std::uint32_t index) const {
        return index < rendered_joint_ids.size()
            ? rendered_joint_ids[index] : ComponentId{};
    }

    float point_size = 10.0f;
    bool enabled = true;

private:
    bool prepare(ViewerAdapter& adapter) {
        const auto status = adapter.prepare_pick_layer({
            .layer = Rhi::PickLayer::joint,
            .nodes_per_pixel = nodes_per_pixel,
            .joint_stride = orlviewer::kJointStride,
        });
        if (!status) {
            std::cerr << "JointPickingFeature: " << status.message << '\n';
        }
        return static_cast<bool>(status);
    }

    SceneGraphContext& graph_context;
    const ComponentManager& components;
    PickRequest pick{Rhi::PickLayer::joint, "JointPickingFeature"};
    std::vector<ComponentId> rendered_joint_ids;
    std::uint32_t nodes_per_pixel = 4;
    bool ready = false;
};

} // namespace ORL
