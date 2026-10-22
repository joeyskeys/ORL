#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <concepts>
#include <span>
#include <string>
#include <string_view>
#include <utility>

namespace ORL::Rhi {

struct Extent2D {
    std::uint32_t width = 0;
    std::uint32_t height = 0;

    constexpr bool valid() const noexcept {
        return width != 0 && height != 0;
    }

    friend constexpr bool operator==(Extent2D, Extent2D) = default;
};

enum class StatusCode : std::uint8_t {
    ok,
    invalid_argument,
    not_initialized,
    unsupported,
    drawable_unavailable,
    resource_failure,
    device_failure,
};

struct Status {
    StatusCode code = StatusCode::ok;
    std::string message;

    constexpr explicit operator bool() const noexcept {
        return code == StatusCode::ok;
    }

    static Status success() {
        return {};
    }

    static Status failure(StatusCode status_code, std::string status_message) {
        return Status{status_code, std::move(status_message)};
    }
};

struct Capabilities {
    bool windowed = false;
    bool offscreen = false;
    bool indexed_mesh = false;
    bool depth_test = false;
    bool blending = false;
    bool picking = false;
    bool host_upload = false;
};

struct AdapterConfig {
    Extent2D initial_extent{1200, 800};
    bool offscreen = false;
    bool validation = true;
    bool force_shader_recompile = false;
    std::string application_name = "ORL";
    std::string window_title = "ORL";
    std::string shader_cache_directory;
    // Loaded at initialize and written back by shutdown when non-empty.
    std::string pipeline_cache_path;
    std::array<float, 4> clear_color{0.035f, 0.045f, 0.065f, 1.0f};
};

struct FrameState {
    std::uint64_t serial = 0;
    Extent2D extent{};
    bool drawable = false;
};

struct CameraState {
    // Column-major matrices match GLM and the existing vkkk camera UBO.
    std::array<float, 16> view{};
    std::array<float, 16> projection{};
};

struct IndexedMeshUpload {
    std::span<const std::byte> vertex_bytes{};
    std::uint32_t vertex_count = 0;
    std::uint32_t vertex_stride = 0;
    std::span<const std::uint32_t> indices{};
};

struct MeshToken {
    std::uint64_t value = 0;

    constexpr explicit operator bool() const noexcept {
        return value != 0;
    }

    friend constexpr bool operator==(MeshToken, MeshToken) = default;
};

struct ShaderId {
    std::uint32_t value = 0;

    friend constexpr bool operator==(ShaderId, ShaderId) = default;
};

// A selected adapter owns the native object behind this token. The common
// contract intentionally does not expose a generic ORL resource hierarchy.
template <typename Adapter>
concept StaticAdapter = requires(
    Adapter& adapter,
    const Adapter& const_adapter,
    const AdapterConfig& config,
    const CameraState& camera,
    FrameState& frame,
    const IndexedMeshUpload& upload,
    MeshToken& token,
    MeshToken mesh,
    Extent2D extent)
{
    { Adapter::backend_name() } -> std::same_as<std::string_view>;
    { adapter.initialize(config) } -> std::same_as<Status>;
    { const_adapter.capabilities() } -> std::same_as<Capabilities>;
    // Host window lifecycle. Offscreen adapters never request a close.
    { adapter.poll_events() } -> std::same_as<void>;
    { const_adapter.should_close() } -> std::same_as<bool>;
    { adapter.set_camera(camera) } -> std::same_as<Status>;
    { adapter.begin_frame(frame) } -> std::same_as<Status>;
    { adapter.resize(extent) } -> std::same_as<Status>;
    { adapter.upload_indexed_mesh(token, upload) } -> std::same_as<Status>;
    { adapter.draw_indexed(mesh, std::uint32_t{}) } -> std::same_as<Status>;
    { adapter.end_frame(frame, bool{}) } -> std::same_as<Status>;
    { adapter.wait_idle() } noexcept;
    // Drains the device and persists native caches; call once before exit.
    { adapter.shutdown() } -> std::same_as<Status>;
};

} // namespace ORL::Rhi
