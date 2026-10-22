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

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

static_assert(ViewerDisplayAdapter<MtAdapter>);

} // namespace ORL::Rhi
