#include "mt_adapter.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <mt/mt.hpp>

namespace ORL::Rhi {
namespace {

Status mt_failure(StatusCode code, std::string message) {
    return Status::failure(code, std::move(message));
}

std::uint32_t buffer_usage(mt::BufferUsage usage) {
    return static_cast<std::uint32_t>(usage);
}

} // namespace

struct MtAdapter::Impl {
    using DeviceHandle = decltype(mt::Device::systemDefault());
    using QueueHandle = decltype(
        std::declval<mt::Device&>().createCommandQueue());
    using SurfaceHandle = decltype(mt::Surface::offscreen(
        std::declval<mt::Device&>(),
        std::uint32_t{},
        std::uint32_t{},
        mt::PixelFormat::rgba8Unorm));
    using CommandBufferHandle = decltype(
        std::declval<mt::CommandQueue&>().makeCommandBuffer());
    using DrawableHandle = decltype(
        std::declval<mt::Surface&>().nextDrawable());
    using RenderPassHandle = decltype(
        std::declval<mt::Surface&>().makeRenderPassDescriptor(
            std::declval<mt::Drawable&>()));
    using BufferHandle = decltype(
        std::declval<mt::Device&>().createStagingBuffer(
            std::uint64_t{},
            std::uint32_t{},
            std::string{}));

    struct MeshRecord {
        BufferHandle vertex_buffer;
        BufferHandle index_buffer;
        std::uint32_t index_count = 0;
    };

    DeviceHandle device;
    QueueHandle queue;
    SurfaceHandle surface;
    CommandBufferHandle active_command_buffer;
    DrawableHandle active_drawable;
    RenderPassHandle active_render_pass;
    std::unordered_map<std::uint64_t, MeshRecord> meshes;
    std::uint64_t next_mesh_token = 1;
    std::uint64_t next_frame_serial = 1;
    Capabilities caps{
        .windowed = false,
        .offscreen = true,
        .indexed_mesh = false,
        .depth_test = false,
        .blending = false,
        .picking = false,
        .host_upload = true,
    };
    bool initialized = false;
    AdapterConfig config{};
    CameraState camera{};
};

MtAdapter::MtAdapter()
    : impl_(std::make_unique<Impl>()) {}

MtAdapter::~MtAdapter() {
    wait_idle();
}

MtAdapter::MtAdapter(MtAdapter&&) noexcept = default;

MtAdapter& MtAdapter::operator=(MtAdapter&&) noexcept = default;

Status MtAdapter::initialize(const AdapterConfig& config) {
    if (!config.initial_extent.valid()) {
        return mt_failure(
            StatusCode::invalid_argument,
            "mt requires a non-zero initial extent");
    }
    if (!config.offscreen) {
        return mt_failure(
            StatusCode::unsupported,
            "the initial mt adapter slice exposes the offscreen surface path");
    }

    impl_->config = config;
    impl_->device = mt::Device::systemDefault();
    if (!impl_->device) {
        return mt_failure(
            StatusCode::device_failure,
            "Metal did not provide a system default device");
    }
    impl_->queue = impl_->device->createCommandQueue("ORL RHI");
    if (!impl_->queue) {
        return mt_failure(
            StatusCode::device_failure,
            "mt could not create a command queue");
    }
    impl_->surface = mt::Surface::offscreen(
        *impl_->device,
        config.initial_extent.width,
        config.initial_extent.height,
        mt::PixelFormat::rgba8Unorm);
    if (!impl_->surface) {
        return mt_failure(
            StatusCode::resource_failure,
            "mt could not create the offscreen color surface");
    }
    impl_->initialized = true;
    return Status::success();
}

Capabilities MtAdapter::capabilities() const noexcept {
    return impl_->caps;
}

void MtAdapter::poll_events() {}

bool MtAdapter::should_close() const {
    return false;
}

Status MtAdapter::set_camera(const CameraState& camera) {
    impl_->camera = camera;
    return Status::success();
}

Status MtAdapter::begin_frame(FrameState& frame) {
    if (!impl_->initialized || !impl_->surface || !impl_->queue) {
        return mt_failure(
            StatusCode::not_initialized,
            "mt RHI adapter has not been initialized");
    }
    if (impl_->active_command_buffer) {
        return mt_failure(
            StatusCode::invalid_argument,
            "mt RHI frame is already active");
    }

    impl_->active_drawable = impl_->surface->nextDrawable();
    if (!impl_->active_drawable) {
        return mt_failure(
            StatusCode::drawable_unavailable,
            "mt surface did not provide a drawable");
    }
    impl_->active_render_pass =
        impl_->surface->makeRenderPassDescriptor(*impl_->active_drawable);
    impl_->active_command_buffer = impl_->queue->makeCommandBuffer();
    if (!impl_->active_render_pass || !impl_->active_command_buffer) {
        impl_->active_drawable.reset();
        impl_->active_render_pass.reset();
        impl_->active_command_buffer.reset();
        return mt_failure(
            StatusCode::resource_failure,
            "mt could not create a frame command buffer or render pass");
    }

    const auto extent = impl_->surface->getDrawableSize();
    frame.serial = impl_->next_frame_serial++;
    frame.extent = {extent.getWidth(), extent.getHeight()};
    frame.drawable = true;
    auto clear_color = impl_->active_render_pass->getClearColor();
    const auto& clear = impl_->config.clear_color;
    clear_color.setRed(clear[0]);
    clear_color.setGreen(clear[1]);
    clear_color.setBlue(clear[2]);
    clear_color.setAlpha(clear[3]);
    impl_->active_render_pass->setClearColor(clear_color);
    return Status::success();
}

Status MtAdapter::resize(Extent2D extent) {
    if (!impl_->initialized || !impl_->surface) {
        return mt_failure(
            StatusCode::not_initialized,
            "mt RHI adapter has not been initialized");
    }
    if (!extent.valid()) {
        return mt_failure(
            StatusCode::invalid_argument,
            "mt resize requires a non-zero extent");
    }
    if (impl_->active_command_buffer) {
        return mt_failure(
            StatusCode::invalid_argument,
            "mt resize cannot run during an active frame");
    }
    if (!impl_->surface->resize(extent.width, extent.height)) {
        return mt_failure(
            StatusCode::resource_failure,
            "mt could not resize the offscreen surface");
    }
    return Status::success();
}

Status MtAdapter::upload_indexed_mesh(
    MeshToken& token,
    const IndexedMeshUpload& upload)
{
    if (!impl_->initialized || !impl_->device) {
        return mt_failure(
            StatusCode::not_initialized,
            "mt RHI adapter has not been initialized");
    }
    if (upload.vertex_count == 0
        || upload.vertex_stride != sizeof(float) * 3
        || upload.vertex_bytes.size()
            != static_cast<std::size_t>(upload.vertex_count)
                * upload.vertex_stride
        || upload.indices.empty()
        || upload.indices.size() % 3 != 0)
    {
        return mt_failure(
            StatusCode::invalid_argument,
            "mt RHI indexed mesh expects tightly packed float3 vertices "
            "and triangle indices");
    }

    const auto record_id = token ? token.value : impl_->next_mesh_token++;
    const auto vertex_buffer = impl_->device->createStagingBuffer(
        static_cast<std::uint64_t>(upload.vertex_bytes.size()),
        buffer_usage(mt::BufferUsage::vertex),
        "orl.rhi.vertices");
    const auto index_buffer = impl_->device->createStagingBuffer(
        static_cast<std::uint64_t>(
            upload.indices.size() * sizeof(std::uint32_t)),
        buffer_usage(mt::BufferUsage::index),
        "orl.rhi.indices");
    if (!vertex_buffer || !index_buffer
        || !vertex_buffer->upload(
            upload.vertex_bytes.data(),
            static_cast<std::uint64_t>(upload.vertex_bytes.size()),
            0)
        || !index_buffer->upload(
            upload.indices.data(),
            static_cast<std::uint64_t>(
                upload.indices.size() * sizeof(std::uint32_t)),
            0))
    {
        return mt_failure(
            StatusCode::resource_failure,
            "mt could not allocate or upload the indexed mesh buffers");
    }

    token.value = record_id;
    impl_->meshes[record_id] = {
        vertex_buffer,
        index_buffer,
        static_cast<std::uint32_t>(upload.indices.size())};
    return Status::success();
}

Status MtAdapter::draw_indexed(MeshToken mesh, std::uint32_t index_count) {
    if (!impl_->initialized) {
        return mt_failure(
            StatusCode::not_initialized,
            "mt RHI adapter has not been initialized");
    }
    if (!impl_->active_command_buffer) {
        return mt_failure(
            StatusCode::invalid_argument,
            "mt draw must be submitted between begin_frame and end_frame");
    }
    if (impl_->meshes.find(mesh.value) == impl_->meshes.end()) {
        return mt_failure(
            StatusCode::invalid_argument,
            "mt draw referenced an unknown RHI mesh token");
    }
    (void)index_count;
    return mt_failure(
        StatusCode::unsupported,
        "mt indexed drawing needs a logical shader/pipeline registration; "
        "the adapter currently validates and owns the uploaded buffers");
}

Status MtAdapter::end_frame(FrameState& frame, bool present) {
    if (!impl_->active_command_buffer
        || !impl_->active_render_pass
        || !frame.drawable)
    {
        return mt_failure(
            StatusCode::invalid_argument,
            "mt end_frame does not match an active frame");
    }

    const auto encoder =
        impl_->active_command_buffer->makeRenderEncoder(
            *impl_->active_render_pass);
    if (!encoder || !encoder->endEncoding()) {
        impl_->active_command_buffer.reset();
        impl_->active_drawable.reset();
        impl_->active_render_pass.reset();
        frame.drawable = false;
        return mt_failure(
            StatusCode::resource_failure,
            "mt could not encode the RHI clear pass");
    }
    if (present
        && impl_->surface->isLayerBacked()
        && impl_->active_drawable
        && !impl_->active_command_buffer->present(
            *impl_->active_drawable))
    {
        return mt_failure(
            StatusCode::resource_failure,
            "mt could not present the offscreen drawable");
    }
    if (!impl_->active_command_buffer->commit()) {
        return mt_failure(
            StatusCode::device_failure,
            "mt could not commit the RHI command buffer");
    }

    impl_->active_command_buffer.reset();
    impl_->active_drawable.reset();
    impl_->active_render_pass.reset();
    frame.drawable = false;
    return Status::success();
}

void MtAdapter::wait_idle() noexcept {
    if (impl_ != nullptr && impl_->active_command_buffer != nullptr) {
        if (impl_->active_command_buffer->isCommitted()) {
            impl_->active_command_buffer->waitUntilCompleted();
        }
    }
}

Status MtAdapter::shutdown() {
    if (!impl_->initialized) {
        return mt_failure(
            StatusCode::not_initialized,
            "mt RHI adapter has not been initialized");
    }
    if (impl_->active_command_buffer) {
        return mt_failure(
            StatusCode::invalid_argument,
            "mt shutdown cannot run during an active frame");
    }
    wait_idle();
    return Status::success();
}

} // namespace ORL::Rhi
