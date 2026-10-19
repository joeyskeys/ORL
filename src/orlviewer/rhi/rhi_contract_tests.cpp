#include "rhi_contracts.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace {

class MockAdapter final {
public:
    static constexpr std::string_view backend_name() noexcept {
        return "mock";
    }

    ORL::Rhi::Status initialize(const ORL::Rhi::AdapterConfig&) {
        initialized = true;
        return ORL::Rhi::Status::success();
    }

    ORL::Rhi::Capabilities capabilities() const noexcept {
        return {
            .windowed = false,
            .offscreen = true,
            .indexed_mesh = true,
            .depth_test = true,
            .blending = true,
            .picking = false,
            .host_upload = true,
        };
    }

    ORL::Rhi::Status begin_frame(ORL::Rhi::FrameState& frame) {
        if (!initialized || active) {
            return ORL::Rhi::Status::failure(
                ORL::Rhi::StatusCode::invalid_argument,
                "invalid mock frame transition");
        }
        active = true;
        frame.serial = ++serial;
        frame.extent = {4, 4};
        frame.drawable = true;
        return ORL::Rhi::Status::success();
    }

    ORL::Rhi::Status resize(ORL::Rhi::Extent2D extent) {
        return extent.valid()
            ? ORL::Rhi::Status::success()
            : ORL::Rhi::Status::failure(
                ORL::Rhi::StatusCode::invalid_argument,
                "invalid mock extent");
    }

    ORL::Rhi::Status upload_indexed_mesh(
        ORL::Rhi::MeshToken& token,
        const ORL::Rhi::IndexedMeshUpload& upload)
    {
        if (upload.vertex_count == 0 || upload.indices.empty()) {
            return ORL::Rhi::Status::failure(
                ORL::Rhi::StatusCode::invalid_argument,
                "invalid mock mesh");
        }
        token.value = 1;
        return ORL::Rhi::Status::success();
    }

    ORL::Rhi::Status draw_indexed(
        ORL::Rhi::MeshToken mesh,
        std::uint32_t index_count)
    {
        if (!active || !mesh || index_count == 0) {
            return ORL::Rhi::Status::failure(
                ORL::Rhi::StatusCode::invalid_argument,
                "invalid mock draw");
        }
        drew = true;
        return ORL::Rhi::Status::success();
    }

    ORL::Rhi::Status end_frame(
        ORL::Rhi::FrameState& frame,
        bool)
    {
        if (!active || frame.serial != serial) {
            return ORL::Rhi::Status::failure(
                ORL::Rhi::StatusCode::invalid_argument,
                "invalid mock frame end");
        }
        active = false;
        frame.drawable = false;
        return drew
            ? ORL::Rhi::Status::success()
            : ORL::Rhi::Status::failure(
                ORL::Rhi::StatusCode::invalid_argument,
                "mock frame did not draw");
    }

    void wait_idle() noexcept {
        active = false;
    }

private:
    bool initialized = false;
    bool active = false;
    bool drew = false;
    std::uint64_t serial = 0;
};

static_assert(ORL::Rhi::StaticAdapter<MockAdapter>);

} // namespace

int main() {
    MockAdapter adapter;
    if (!adapter.initialize({})) {
        return 1;
    }

    constexpr std::array<float, 9> vertices{
        0.0f, 0.5f, 0.0f,
        -0.5f, -0.5f, 0.0f,
        0.5f, -0.5f, 0.0f,
    };
    constexpr std::array<std::uint32_t, 3> indices{0, 1, 2};
    const ORL::Rhi::IndexedMeshUpload upload{
        .vertex_bytes = std::as_bytes(std::span{vertices}),
        .vertex_count = 3,
        .vertex_stride = sizeof(float) * 3,
        .indices = indices,
    };

    ORL::Rhi::MeshToken mesh;
    ORL::Rhi::FrameState frame;
    if (!adapter.upload_indexed_mesh(mesh, upload)
        || !adapter.begin_frame(frame)
        || !adapter.draw_indexed(mesh, 3)
        || !adapter.end_frame(frame, false))
    {
        return 2;
    }
    return 0;
}
