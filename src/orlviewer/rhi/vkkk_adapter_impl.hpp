#pragma once

// Private to orl_rhi_vkkk. Shared by the adapter core and its display
// translation units; never included by viewer code.

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

#include <glm/mat4x4.hpp>
#include <glm/vec4.hpp>

#include "built_in_shader/common.h"
#include "concepts/camera.h"
#include "concepts/point.h"
#include "vp/frame_axis.hpp"
#include "vp/grid.hpp"
#if !ORL_USE_QT6
#include "gui/gui.h"
#endif
#include "vk_ins/shader_module_pack.hpp"
#include "rhi_display.hpp"
#include "vkkk_native.hpp"

namespace ORL::Rhi {

inline Status exception_status(StatusCode code, const std::exception& error) {
    return Status::failure(code, error.what());
}

namespace vkkk_detail {

enum class Pass : std::uint8_t {
    picking,
    scene,
    overlay,
};

struct ShaderSource {
    // Either a vkkk built-in GLSL string or a viewer shader file.
    const char* builtin = nullptr;
    const char* builtin_name = "";
    std::filesystem::path path;
};

struct PipelineDesc {
    std::string name;
    ShaderSource vertex;
    ShaderSource fragment;
    vk::PrimitiveTopology topology = vk::PrimitiveTopology::eTriangleList;
    vk::PolygonMode polygon_mode = vk::PolygonMode::eFill;
    vk::CullModeFlags cull_mode = vk::CullModeFlagBits::eNone;
    vk::FrontFace front_face = vk::FrontFace::eCounterClockwise;
    float line_width = 1.0f;
    bool dynamic_line_width = false;
    bool depth_test = true;
    bool depth_write = true;
    vk::CompareOp depth_compare = vk::CompareOp::eLessOrEqual;
    bool alpha_blend = false;
    std::vector<vkkk::VERT_COMP> components{vkkk::VERTEX};
    // Depth-format-only target used by the A-buffer picking passes.
    bool pick_target = false;
};

inline bool load_stage(vkkk::Context& context, vkkk::ShaderModule& module,
    const ShaderSource& source, vk::ShaderStageFlagBits stage)
{
    if (source.builtin != nullptr) {
        return context.load_shader(module, source.builtin, stage, source.builtin_name);
    }
    return context.load_shader(module, source.path, stage);
}

inline bool create_pipeline(vkkk::Context& context, const PipelineDesc& desc) {
    if (context.pipelines.contains(desc.name)) {
        return true;
    }

    vkkk::ShaderModule vertex_module;
    vkkk::ShaderModule fragment_module;
    if (!load_stage(context, vertex_module, desc.vertex, vk::ShaderStageFlagBits::eVertex)
        || !load_stage(context, fragment_module, desc.fragment,
            vk::ShaderStageFlagBits::eFragment))
    {
        return false;
    }
    vkkk::ShaderModulePack pack;
    if (!pack.add_shader_module(vertex_module) || !pack.add_shader_module(fragment_module)) {
        return false;
    }

    vkkk::PipelineOption option;
    option.setup_input_assembly(desc.topology, false);
    option.setup_multisampling(false, vk::SampleCountFlagBits::e1);
    option.setup_rasterizer(false, false, desc.polygon_mode, desc.line_width,
        desc.cull_mode, desc.front_face, false);
    option.setup_depth_stencil(desc.depth_test, desc.depth_write, desc.depth_compare,
        false, false);
    if (desc.alpha_blend) {
        option.blend_attachment_info.blendEnable = vk::True;
        option.blend_attachment_info.srcColorBlendFactor = vk::BlendFactor::eSrcAlpha;
        option.blend_attachment_info.dstColorBlendFactor = vk::BlendFactor::eOneMinusSrcAlpha;
        option.blend_attachment_info.colorBlendOp = vk::BlendOp::eAdd;
        option.blend_attachment_info.srcAlphaBlendFactor = vk::BlendFactor::eOne;
        option.blend_attachment_info.dstAlphaBlendFactor = vk::BlendFactor::eOneMinusSrcAlpha;
        option.blend_attachment_info.alphaBlendOp = vk::BlendOp::eAdd;
    }
    if (desc.dynamic_line_width) {
        option.dynamic_states.push_back(vk::DynamicState::eLineWidth);
        option.dynamic_info.dynamicStateCount =
            static_cast<uint32_t>(option.dynamic_states.size());
        option.dynamic_info.pDynamicStates = option.dynamic_states.data();
    }
    if (desc.pick_target) {
        return context.create_pipeline(desc.name, pack, option, desc.components,
            true, true, {}, static_cast<vk::Format>(context.get_depth_format()));
    }
    return context.create_pipeline(desc.name, pack, option, desc.components);
}

// Wide lines are optional in Vulkan; without them every line is 1px.
inline float line_width_for(const vkkk::Context& context, float requested) {
    return context.wide_lines_enabled
        ? std::clamp(requested, context.line_width_range[0], context.line_width_range[1])
        : 1.0f;
}

inline float point_size_for(const vkkk::Context& context, float requested) {
    return std::clamp(requested, context.point_size_range[0], context.point_size_range[1]);
}

// A single-vertex point set; shaders that pull records from an SSBO draw it
// once per record.
inline bool load_dummy_points(vkkk::Context& context, const std::string& name) {
    if (context.points.contains(name)) {
        return true;
    }
    const std::array<float, 3> vertex{0.0f, 0.0f, 0.0f};
    vkkk::Points points({vkkk::VERTEX});
    points.load(1, reinterpret_cast<const char*>(vertex.data()),
        static_cast<std::uint32_t>(vertex.size() * sizeof(float)));
    return context.load_points(name, points);
}

inline glm::mat4 to_glm(const Mat4& value) {
    glm::mat4 result;
    static_assert(sizeof(result) == sizeof(value));
    std::memcpy(&result, value.data(), sizeof(result));
    return result;
}

inline glm::vec4 to_glm(const Vec4& value) {
    return {value[0], value[1], value[2], value[3]};
}

struct SceneMeshState {
    bool prepared = false;
    bool phong_ready = false;
    bool wire_ready = false;
    bool xray_ready = false;
};

struct ControllerState {
    bool prepared = false;
    bool lines_ready = false;
    bool poly_ready = false;
};

struct PickLayerState {
    bool prepared = false;
    std::uint32_t nodes_per_pixel = 0;
    // Written by the replayed picking command once its pass is recorded.
    std::optional<std::uint64_t> rendered_serial;
    std::uint32_t rendered_image_index = 0;
    // Mesh layer: instance capacity of the pick SSBO.
    std::size_t instance_capacity = 0;
    // Joint layer.
    std::size_t joint_stride = 0;
    std::optional<std::uint64_t> device_upload_serial;
    std::uint32_t device_joint_count = 0;
};

struct JointState {
    bool prepared = false;
    bool points_ready = false;
    bool lines_ready = false;
    bool line_width_dynamic = false;
    std::size_t joint_stride = 0;
    // Serial of the frame whose joint buffers hold the last device upload.
    std::optional<std::uint64_t> device_upload_serial;
    std::uint32_t device_joint_count = 0;
};

} // namespace vkkk_detail

struct VkkkAdapter::Impl {
    struct MeshRecord {
        std::string name;
        std::uint32_t index_count = 0;
    };

    std::unique_ptr<VkkkWindow> window;
    std::unique_ptr<vkkk::Context> context;
    std::optional<vkkk::Context::Frame> active_frame;
    std::unordered_map<std::uint64_t, MeshRecord> meshes;
    std::vector<std::pair<MeshToken, std::uint32_t>> pending_draws;
    VkkkPassHooks pass_hooks;
    // Display-family draws queued during the frame, replayed by end_frame in
    // submission order inside the matching pass.
    std::vector<VkkkRecordFn> picking_commands;
    std::vector<VkkkRecordFn> scene_commands;
    std::vector<VkkkRecordFn> overlay_commands;
    std::unordered_map<std::string, ShapeKind> shapes;
    vkkk_detail::SceneMeshState scene_meshes;
    vkkk_detail::JointState joints;
    vkkk_detail::ControllerState controllers;
    std::array<vkkk_detail::PickLayerState, 3> pick_layers{};
    bool locators_prepared = false;
    bool locators_ready = false;
    bool ortho_grid_prepared = false;
    bool ortho_grid_ready = false;
    bool overlay_lines_prepared = false;
    bool overlay_lines_ready = false;
    bool overlay_line_width_dynamic = false;
    bool status_text_prepared = false;
    bool mesh_adjacency_ready = false;
#if !ORL_USE_QT6
    std::unique_ptr<vkkk::ImGuiHud> hud;
#endif
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
    bool shut_down = false;
    AdapterConfig config{};
    vkkk::CameraUBO camera_ubo{};
    // vkkk's reference features read a Camera; only ubo_data is kept current.
    vkkk::Camera camera_mirror{};
    std::unique_ptr<vkkk::vp::GridFeature> world_grid;
    std::unique_ptr<vkkk::vp::FrameAxisFeature> frame_axis;

    ~Impl();

    Status create_pipeline();

    // The HUD holds Vulkan and window-system state, so it goes before the context.
    void release_hud() noexcept {
#if !ORL_USE_QT6
        if (hud != nullptr) {
            try {
                hud->shutdown();
            }
            catch (...) {
            }
            hud.reset();
        }
#endif
    }

    std::filesystem::path shader_path(std::string_view file) const {
        return std::filesystem::path{config.shader_directory} / std::string{file};
    }

    Status require_context(std::string_view call) const {
        if (!initialized || context == nullptr) {
            return Status::failure(StatusCode::not_initialized,
                std::string{call} + ": vkkk RHI adapter has not been initialized");
        }
        return Status::success();
    }

    Status require_frame(std::string_view call) const {
        if (auto status = require_context(call); !status) {
            return status;
        }
        if (!active_frame.has_value()) {
            return Status::failure(StatusCode::invalid_argument,
                std::string{call} + " must run between begin_frame and end_frame");
        }
        return Status::success();
    }

    void queue(vkkk_detail::Pass pass, VkkkRecordFn command) {
        switch (pass) {
        case vkkk_detail::Pass::picking:
            picking_commands.push_back(std::move(command));
            break;
        case vkkk_detail::Pass::scene:
            scene_commands.push_back(std::move(command));
            break;
        case vkkk_detail::Pass::overlay:
            overlay_commands.push_back(std::move(command));
            break;
        }
    }

    void clear_frame_commands() {
        pending_draws.clear();
        picking_commands.clear();
        scene_commands.clear();
        overlay_commands.clear();
    }
};

} // namespace ORL::Rhi
