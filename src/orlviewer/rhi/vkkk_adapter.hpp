#pragma once

#include <memory>

#include "adapter.hpp"
#include "rhi_display.hpp"

namespace ORL::Rhi {

class VkkkAdapter final {
public:
    VkkkAdapter();
    ~VkkkAdapter();

    VkkkAdapter(const VkkkAdapter&) = delete;
    VkkkAdapter& operator=(const VkkkAdapter&) = delete;
    VkkkAdapter(VkkkAdapter&&) noexcept;
    VkkkAdapter& operator=(VkkkAdapter&&) noexcept;

    static constexpr std::string_view backend_name() noexcept {
        return "vkkk";
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

    Status register_shape(std::string_view name, ShapeKind kind,
        const ShapeGeometry& geometry);
    Status prepare_scene_meshes(const SceneMeshSetup& setup);
    Status draw_scene_meshes(const SceneMeshDraw& draw);

private:
    friend struct VkkkNative;

    struct Impl;
    std::unique_ptr<Impl> impl_;
};

static_assert(ViewerDisplayAdapter<VkkkAdapter>);

} // namespace ORL::Rhi
