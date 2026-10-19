#pragma once

#include <memory>

#include "adapter.hpp"

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
    Status begin_frame(FrameState& frame);
    Status resize(Extent2D extent);
    Status upload_indexed_mesh(
        MeshToken& token,
        const IndexedMeshUpload& upload);
    Status draw_indexed(MeshToken mesh, std::uint32_t index_count);
    Status end_frame(FrameState& frame, bool present);
    void wait_idle() noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

static_assert(StaticAdapter<VkkkAdapter>);

} // namespace ORL::Rhi
