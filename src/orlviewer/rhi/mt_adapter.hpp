#pragma once

#include <memory>

#include "adapter.hpp"
#include "rhi_display.hpp"

namespace ORL::Rhi {

class MtAdapter final {
public:
    MtAdapter();
    ~MtAdapter();

    MtAdapter(const MtAdapter&) = delete;
    MtAdapter& operator=(const MtAdapter&) = delete;
    MtAdapter(MtAdapter&&) noexcept;
    MtAdapter& operator=(MtAdapter&&) noexcept;

    static constexpr std::string_view backend_name() noexcept {
        return "mt";
    }

    Status initialize(const AdapterConfig& config);
    Capabilities capabilities() const noexcept;
    void poll_events();
    bool should_close() const;
    Status set_camera(const CameraState& camera);
    Status begin_frame(FrameState& frame);
    Status resize(Extent2D extent);
    Status upload_indexed_mesh(
        MeshToken& token,
        const IndexedMeshUpload& upload);
    Status draw_indexed(MeshToken mesh, std::uint32_t index_count);
    Status end_frame(FrameState& frame, bool present);
    void wait_idle() noexcept;
    Status shutdown();

    // Display families report unsupported until their MSL pipelines exist.
    Status register_shape(std::string_view name, ShapeKind kind,
        const ShapeGeometry& geometry);
    Status prepare_scene_meshes(const SceneMeshSetup& setup);
    Status draw_scene_meshes(const SceneMeshDraw& draw);
    Status prepare_joints(const JointDisplaySetup& setup);
    Status upload_device_joints(DeviceBuffer device, std::uint32_t count);
    Status draw_joints(const JointDraw& draw);
    Status prepare_controllers();
    Status draw_controller(const ShapeInstanceDraw& draw);
    Status prepare_locators();
    Status upload_device_locators(DeviceBuffer device, std::uint32_t count);
    Status draw_locators(const LocatorDraw& draw);
    Status prepare_ortho_grid();
    Status draw_ortho_grid(const OrthoGridDraw& draw);
    Status prepare_overlay_lines();
    Status draw_overlay_line(const ShapeInstanceDraw& draw);
    Status prepare_status_text();
    Status draw_status_text(std::string_view text);
    Status prepare_world_grid(const WorldGridSetup& setup);
    Status draw_world_grid();
    Status prepare_frame_axis(const FrameAxisSetup& setup);
    Status draw_frame_axis();
    Status prepare_pick_layer(const PickLayerSetup& setup);
    bool map_cursor(double x, double y, std::uint32_t& px, std::uint32_t& py) const;
    Status draw_mesh_pick(std::span<const MeshPickInstance> instances);
    Status upload_device_pick_joints(DeviceBuffer device, std::uint32_t count);
    Status draw_joint_pick(const JointPickDraw& draw);
    Status upload_device_pick_locators(DeviceBuffer device, std::uint32_t count);
    Status draw_rig_pick(const RigPickDraw& draw);
    Status read_pick(PickLayer layer, std::uint64_t min_serial,
        std::uint32_t x, std::uint32_t y, PickReadback& readback);
    Status prepare_mesh_adjacency();
    Status build_mesh_adjacency(const MeshAdjacencyInput& input, MeshAdjacency& output);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

static_assert(ViewerDisplayAdapter<MtAdapter>);

} // namespace ORL::Rhi
