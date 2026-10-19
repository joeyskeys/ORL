#include "vkkk_adapter.hpp"

#include <filesystem>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "built_in_shader/fixed_color.h"
#include "concepts/camera.h"
#include "concepts/mesh.h"
#include "vk_ins/context.hpp"
#include "vk_ins/shader_module_pack.hpp"

#if ORL_USE_QT6
#include "gui/qt_backend.hpp"
#else
#include "gui/glfw_backend.hpp"
#endif

namespace ORL::Rhi {
namespace {

constexpr const char* kPipelineName = "orl_rhi_fixed_color";
constexpr const char* kInstanceBlockName = "FixedColorInstanceAttrs";
constexpr const char* kCameraBlockName = "CameraUBO";

Status exception_status(StatusCode code, const std::exception& error) {
    return Status::failure(code, error.what());
}

} // namespace

struct VkkkAdapter::Impl {
    struct MeshRecord {
        std::string name;
        std::uint32_t index_count = 0;
    };

    std::unique_ptr<vkkk::WindowBackend> window;
    std::unique_ptr<vkkk::Context> context;
    std::optional<vkkk::Context::Frame> active_frame;
    std::unordered_map<std::uint64_t, MeshRecord> meshes;
    std::vector<std::pair<MeshToken, std::uint32_t>> pending_draws;
    std::uint64_t next_mesh_token = 1;
    Capabilities caps{
        .windowed = true,
        .offscreen = false,
        .indexed_mesh = true,
        .depth_test = true,
        .blending = true,
        .picking = false,
        .host_upload = true,
    };
    bool initialized = false;
    bool pipeline_ready = false;
    AdapterConfig config{};
    vkkk::CameraUBO camera_ubo{};

    ~Impl() {
        if (context != nullptr) {
            try {
                context->wait_idle();
            }
            catch (...) {
                // Destruction must not throw after a partially initialized
                // native context has already reported its failure.
            }
        }
        context.reset();
        window.reset();
    }

    Status create_pipeline() {
        vkkk::ShaderModule vertex_module;
        vkkk::ShaderModule fragment_module;
        if (!vertex_module.load(
                vkkk::fixed_color_vert,
                vk::ShaderStageFlagBits::eVertex,
                "orl_rhi_fixed_color_vert")
            || !fragment_module.load(
                vkkk::fixed_color_frag,
                vk::ShaderStageFlagBits::eFragment,
                "orl_rhi_fixed_color_frag"))
        {
            return Status::failure(
                StatusCode::resource_failure,
                "vkkk could not compile the RHI fixed-color shaders");
        }

        vkkk::ShaderModulePack shader_pack;
        if (!shader_pack.add_shader_module(vertex_module)
            || !shader_pack.add_shader_module(fragment_module))
        {
            return Status::failure(
                StatusCode::resource_failure,
                "vkkk could not assemble the RHI shader pack");
        }

        vkkk::PipelineOption option;
        option.setup_input_assembly(vk::PrimitiveTopology::eTriangleList, false);
        option.setup_multisampling(false, vk::SampleCountFlagBits::e1);
        option.setup_rasterizer(
            false,
            false,
            vk::PolygonMode::eFill,
            1.0f,
            vk::CullModeFlagBits::eNone,
            vk::FrontFace::eCounterClockwise,
            false);
        option.setup_depth_stencil(
            true,
            true,
            vk::CompareOp::eLessOrEqual,
            false,
            false);

        if (!context->create_pipeline(
                kPipelineName,
                shader_pack,
                option,
                {vkkk::VERTEX}))
        {
            return Status::failure(
                StatusCode::resource_failure,
                "vkkk could not create the RHI fixed-color pipeline");
        }
        if (!context->resize_pipeline_ssbo(
                kPipelineName,
                kInstanceBlockName,
                1)
            || !context->alloc_pipeline_ssbo(
                kPipelineName,
                kInstanceBlockName))
        {
            return Status::failure(
                StatusCode::resource_failure,
                "vkkk could not allocate the RHI instance buffer");
        }

        camera_ubo.view = glm::mat4{1.0f};
        camera_ubo.proj = glm::mat4{1.0f};
        pipeline_ready = true;
        return Status::success();
    }
};

VkkkAdapter::VkkkAdapter()
    : impl_(std::make_unique<Impl>()) {}

VkkkAdapter::~VkkkAdapter() = default;

VkkkAdapter::VkkkAdapter(VkkkAdapter&&) noexcept = default;

VkkkAdapter& VkkkAdapter::operator=(VkkkAdapter&&) noexcept = default;

Status VkkkAdapter::initialize(const AdapterConfig& config) {
    if (!config.initial_extent.valid()) {
        return Status::failure(
            StatusCode::invalid_argument,
            "vkkk requires a non-zero initial extent");
    }
    if (config.offscreen) {
        return Status::failure(
            StatusCode::unsupported,
            "vkkk RHI adapter currently requires a window surface");
    }

    try {
        impl_->config = config;
#if ORL_USE_QT6
        impl_->window = std::make_unique<vkkk::QtBackend>(
            static_cast<int>(config.initial_extent.width),
            static_cast<int>(config.initial_extent.height),
            config.application_name.c_str());
#else
        impl_->window = std::make_unique<vkkk::GlfwBackend>(
            static_cast<int>(config.initial_extent.width),
            static_cast<int>(config.initial_extent.height),
            config.application_name.c_str(),
            true);
#endif
        impl_->context = std::make_unique<vkkk::Context>();
        if (!config.shader_cache_directory.empty()) {
            impl_->context->set_shader_cache(
                std::filesystem::path{config.shader_cache_directory},
                config.force_shader_recompile);
        }
        if (!config.pipeline_cache_path.empty()) {
            impl_->context->set_pipeline_cache_path(
                std::filesystem::path{config.pipeline_cache_path},
                config.force_shader_recompile);
        }
        impl_->context->init(
            *impl_->window,
            config.application_name.c_str(),
            VK_MAKE_VERSION(0, 1, 0),
            "ORL RHI",
            vk::ApiVersion13,
            config.validation,
            {},
            {});

        const auto pipeline_status = impl_->create_pipeline();
        if (!pipeline_status) {
            return pipeline_status;
        }
        impl_->initialized = true;
        return Status::success();
    }
    catch (const std::exception& error) {
        return exception_status(StatusCode::device_failure, error);
    }
}

Capabilities VkkkAdapter::capabilities() const noexcept {
    return impl_->caps;
}

Status VkkkAdapter::begin_frame(FrameState& frame) {
    if (!impl_->initialized || impl_->context == nullptr) {
        return Status::failure(
            StatusCode::not_initialized,
            "vkkk RHI adapter has not been initialized");
    }
    if (impl_->active_frame.has_value()) {
        return Status::failure(
            StatusCode::invalid_argument,
            "vkkk RHI frame is already active");
    }

    try {
        impl_->window->poll_events();
        impl_->active_frame.emplace();
        if (!impl_->context->begin_frame(*impl_->active_frame)) {
            impl_->active_frame.reset();
            frame.drawable = false;
            frame.extent = {
                impl_->context->extent().width,
                impl_->context->extent().height};
            return Status::failure(
                StatusCode::drawable_unavailable,
                "vkkk could not acquire a swapchain image");
        }
        frame.serial = impl_->active_frame->serial;
        const auto extent = impl_->context->extent();
        frame.extent = {extent.width, extent.height};
        frame.drawable = true;
        return Status::success();
    }
    catch (const std::exception& error) {
        impl_->active_frame.reset();
        return exception_status(StatusCode::device_failure, error);
    }
}

Status VkkkAdapter::resize(Extent2D extent) {
    if (!impl_->initialized || impl_->context == nullptr) {
        return Status::failure(
            StatusCode::not_initialized,
            "vkkk RHI adapter has not been initialized");
    }
    if (!extent.valid()) {
        return Status::failure(
            StatusCode::invalid_argument,
            "vkkk resize requires a non-zero extent");
    }
    if (impl_->active_frame.has_value()) {
        return Status::failure(
            StatusCode::invalid_argument,
            "vkkk resize cannot run during an active frame");
    }

    try {
        impl_->context->recreate_swapchain();
        return Status::success();
    }
    catch (const std::exception& error) {
        return exception_status(StatusCode::device_failure, error);
    }
}

Status VkkkAdapter::upload_indexed_mesh(
    MeshToken& token,
    const IndexedMeshUpload& upload)
{
    if (!impl_->initialized || impl_->context == nullptr) {
        return Status::failure(
            StatusCode::not_initialized,
            "vkkk RHI adapter has not been initialized");
    }
    if (upload.vertex_count == 0
        || upload.vertex_stride != sizeof(float) * 3
        || upload.vertex_bytes.size()
            != static_cast<std::size_t>(upload.vertex_count)
                * upload.vertex_stride
        || upload.indices.empty()
        || upload.indices.size() % 3 != 0)
    {
        return Status::failure(
            StatusCode::invalid_argument,
            "vkkk RHI indexed mesh expects tightly packed float3 vertices "
            "and triangle indices");
    }

    const auto record_id = token ? token.value : impl_->next_mesh_token++;
    const std::string name = "orl_rhi_mesh_" + std::to_string(record_id);
    try {
        vkkk::Mesh mesh({vkkk::VERTEX});
        mesh.load(
            upload.vertex_count,
            reinterpret_cast<const char*>(upload.vertex_bytes.data()),
            static_cast<std::uint32_t>(upload.vertex_bytes.size()),
            static_cast<std::uint32_t>(upload.indices.size() / 3),
            reinterpret_cast<const char*>(upload.indices.data()),
            static_cast<std::uint32_t>(
                upload.indices.size() * sizeof(std::uint32_t)));
        if (!impl_->context->load_mesh(name, mesh)) {
            return Status::failure(
                StatusCode::resource_failure,
                "vkkk rejected the RHI mesh upload");
        }
        token.value = record_id;
        impl_->meshes[record_id] = {
            name,
            static_cast<std::uint32_t>(upload.indices.size())};
        return Status::success();
    }
    catch (const std::exception& error) {
        return exception_status(StatusCode::resource_failure, error);
    }
}

Status VkkkAdapter::draw_indexed(MeshToken mesh, std::uint32_t index_count) {
    if (!impl_->initialized || impl_->context == nullptr) {
        return Status::failure(
            StatusCode::not_initialized,
            "vkkk RHI adapter has not been initialized");
    }
    if (!impl_->active_frame.has_value()) {
        return Status::failure(
            StatusCode::invalid_argument,
            "vkkk draw must be submitted between begin_frame and end_frame");
    }
    const auto found = impl_->meshes.find(mesh.value);
    if (found == impl_->meshes.end()) {
        return Status::failure(
            StatusCode::invalid_argument,
            "vkkk draw referenced an unknown RHI mesh token");
    }
    if (index_count == 0) {
        index_count = found->second.index_count;
    }
    if (index_count == 0 || index_count > found->second.index_count) {
        return Status::failure(
            StatusCode::invalid_argument,
            "vkkk draw index count is outside the uploaded mesh");
    }
    impl_->pending_draws.emplace_back(mesh, index_count);
    return Status::success();
}

Status VkkkAdapter::end_frame(FrameState& frame, bool present) {
    if (!impl_->initialized || impl_->context == nullptr) {
        return Status::failure(
            StatusCode::not_initialized,
            "vkkk RHI adapter has not been initialized");
    }
    if (!impl_->active_frame.has_value()
        || frame.serial != impl_->active_frame->serial)
    {
        return Status::failure(
            StatusCode::invalid_argument,
            "vkkk end_frame does not match the active frame");
    }

    try {
        const auto native_frame = *impl_->active_frame;
        impl_->context->record_frame(
            native_frame,
            [this, present](vk::raii::CommandBuffer& command_buffer,
                            std::uint32_t image_index) {
                vkkk::PassDesc pass;
                pass.present = present;
                pass.colors[0].clear = {0.035f, 0.045f, 0.065f, 1.0f};
                impl_->context->begin_pass(
                    command_buffer,
                    image_index,
                    pass);
                impl_->context->sync_ubo(
                    kPipelineName,
                    kCameraBlockName,
                    &impl_->camera_ubo,
                    image_index);
                vkkk::FixedColorInstanceAttrs instance;
                for (const auto& [token, index_count] : impl_->pending_draws) {
                    const auto found = impl_->meshes.find(token.value);
                    if (found == impl_->meshes.end()) {
                        continue;
                    }
                    impl_->context->sync_ssbo(
                        kPipelineName,
                        kInstanceBlockName,
                        &instance,
                        image_index,
                        sizeof(instance));
                    if (impl_->context->bind(
                            command_buffer,
                            kPipelineName,
                            image_index))
                    {
                        impl_->context->draw(
                            command_buffer,
                            kPipelineName,
                            found->second.name,
                            1,
                            0,
                            index_count);
                    }
                }
                impl_->context->end_pass(
                    command_buffer,
                    image_index,
                    pass);
            });
        impl_->context->end_frame(native_frame);
        impl_->pending_draws.clear();
        impl_->active_frame.reset();
        frame.drawable = false;
        return Status::success();
    }
    catch (const std::exception& error) {
        impl_->pending_draws.clear();
        impl_->active_frame.reset();
        return exception_status(StatusCode::device_failure, error);
    }
}

void VkkkAdapter::wait_idle() noexcept {
    if (impl_ != nullptr && impl_->context != nullptr) {
        try {
            impl_->context->wait_idle();
        }
        catch (...) {
        }
    }
}

} // namespace ORL::Rhi
