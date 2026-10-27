#include "vkkk_adapter.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

#include "built_in_shader/phong.h"
#include "concepts/line.h"
#include "concepts/mesh.h"
#include "concepts/point.h"
#include "vkkk_adapter_impl.hpp"

#if !ORL_USE_QT6
#include <imgui.h>
#endif

namespace ORL::Rhi {
namespace {

using vkkk_detail::load_dummy_points;
using vkkk_detail::Pass;
using vkkk_detail::PipelineDesc;
using vkkk_detail::to_glm;

static_assert(sizeof(Vec3) == sizeof(float) * 3);

Status geometry_status(std::string_view name, ShapeKind kind,
    const ShapeGeometry& geometry)
{
    const std::size_t arity = kind == ShapeKind::lines ? 2 : 3;
    if (geometry.positions.empty() || geometry.indices.empty()
        || geometry.indices.size() % arity != 0)
    {
        return Status::failure(StatusCode::invalid_argument,
            "shape '" + std::string{name} + "' needs positions and complete "
                + (kind == ShapeKind::lines ? "line pairs" : "triangles"));
    }
    for (const auto index : geometry.indices) {
        if (index >= geometry.positions.size()) {
            return Status::failure(StatusCode::invalid_argument,
                "shape '" + std::string{name} + "' indexes past its positions");
        }
    }
    return Status::success();
}

// --- Scene meshes -----------------------------------------------------------

constexpr const char* kPhongPipeline = "orl_scene_phong";
constexpr const char* kWirePipeline = "orl_scene_wire";
constexpr const char* kXRayPipeline = "orl_scene_xray";
constexpr const char* kXRayInstanceBlock = "XRayInstanceAttrs";

struct XRayInstanceAttrs {
    glm::mat4 model{1.0f};
    glm::vec4 color{0.45f, 0.78f, 0.95f, 0.22f};
};

struct SceneMeshRecord {
    std::string mesh;
    glm::mat4 model{1.0f};
};

vkkk::PhongInstanceAttrs phong_attrs(const SurfaceMaterial& material) {
    vkkk::PhongInstanceAttrs attrs;
    attrs.ambient = to_glm(material.ambient);
    attrs.diffuse = to_glm(material.diffuse);
    attrs.specular = to_glm(material.specular);
    attrs.shininess = material.shininess;
    return attrs;
}

std::vector<SceneMeshRecord> scene_records(std::span<const SceneMeshInstance> instances) {
    std::vector<SceneMeshRecord> records;
    records.reserve(instances.size());
    for (const auto& instance : instances) {
        records.push_back({instance.mesh, to_glm(instance.model)});
    }
    return records;
}

// --- Joints -----------------------------------------------------------------

constexpr const char* kJointPointPipeline = "orl_joints";
constexpr const char* kJointLinePipeline = "orl_joint_lines";
constexpr const char* kJointsBlock = "Joints";
constexpr const char* kPointSizeBlock = "PointSizeUBO";
constexpr const char* kPointColorBlock = "PointColorUBO";
constexpr const char* kLineWidthBlock = "LineWidthUBO";
constexpr const char* kLineColorBlock = "LineColorUBO";
constexpr const char* kJointDummyPoints = "orl_joint_draw_points";
constexpr const char* kJointDummyLines = "orl_joint_draw_lines";

// --- Controllers ------------------------------------------------------------

constexpr const char* kControllerLinePipeline = "orl_controller_lines";
constexpr const char* kControllerPolyPipeline = "orl_controller_poly";
constexpr const char* kModelBlock = "ModelUBO";
constexpr const char* kColorBlock = "ColorUBO";
constexpr float kControllerLineWidth = 2.5f;
constexpr const char* kOverlayLinePipeline = "orl_transform_guide";

// --- Locators ---------------------------------------------------------------

constexpr const char* kLocatorPipeline = "orl_locators";
constexpr const char* kLocatorDevicePipeline = "orl_locators_device";
constexpr const char* kLocatorsBlock = "Locators";
constexpr const char* kLocatorTransformsBlock = "LocatorTransforms";
constexpr std::size_t kDeviceLocatorStride = sizeof(double) * 16;

// Matches the std430 Locators record in locator.vert.
static_assert(sizeof(LocatorInstance) == sizeof(float) * 20);

// --- Ortho grid -------------------------------------------------------------

constexpr const char* kOrthoGridPipeline = "orl_ortho_grid";
constexpr const char* kOrthoGridBlock = "OrthoGridUBO";
constexpr const char* kOrthoGridTriangle = "orl_ortho_grid_tri";

struct OrthoGridUBO {
    glm::vec4 origin_px_ppu{0.0f};
    glm::vec4 viewport_cell{0.0f, 0.0f, 1.0f, 1.0f};
    glm::vec4 line_color{0.22f, 0.22f, 0.22f, 1.0f};
    glm::vec4 axis_color{0.08f, 0.08f, 0.08f, 2.0f};
};

bool load_dummy_triangle(vkkk::Context& context, const std::string& name) {
    if (context.meshes.contains(name)) {
        return true;
    }
    constexpr float vertices[] = {
        0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f,
    };
    constexpr std::uint32_t indices[] = {0, 1, 2};
    vkkk::Mesh mesh({vkkk::VERTEX});
    mesh.load(3, reinterpret_cast<const char*>(vertices), sizeof(vertices),
        1, reinterpret_cast<const char*>(indices), sizeof(indices));
    return context.load_mesh(name, mesh);
}

bool load_dummy_lines(vkkk::Context& context, const std::string& name) {
    if (context.lines.contains(name)) {
        return true;
    }
    constexpr float vertices[] = {
        0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f,
    };
    constexpr std::uint32_t indices[] = {0, 1};
    vkkk::Lines lines({vkkk::VERTEX});
    lines.load(2, reinterpret_cast<const char*>(vertices), sizeof(vertices),
        2, reinterpret_cast<const char*>(indices), sizeof(indices));
    return context.load_lines(name, lines);
}

} // namespace

Status VkkkAdapter::register_shape(std::string_view name, ShapeKind kind,
    const ShapeGeometry& geometry)
{
    if (auto status = impl_->require_context("register_shape"); !status) {
        return status;
    }
    const std::string key{name};
    if (const auto found = impl_->shapes.find(key); found != impl_->shapes.end()) {
        return found->second == kind
            ? Status::success()
            : Status::failure(StatusCode::invalid_argument,
                "shape '" + key + "' is already registered with another kind");
    }
    if (auto status = geometry_status(name, kind, geometry); !status) {
        return status;
    }

    auto& context = *impl_->context;
    const auto* vertex_data = reinterpret_cast<const char*>(geometry.positions.data());
    const auto vertex_count = static_cast<std::uint32_t>(geometry.positions.size());
    const auto vertex_bytes = static_cast<std::uint32_t>(
        geometry.positions.size() * sizeof(Vec3));
    const auto* index_data = reinterpret_cast<const char*>(geometry.indices.data());
    const auto index_bytes = static_cast<std::uint32_t>(
        geometry.indices.size() * sizeof(std::uint32_t));
    try {
        bool loaded = false;
        if (kind == ShapeKind::lines) {
            loaded = context.lines.contains(key);
            if (!loaded) {
                vkkk::Lines lines({vkkk::VERTEX});
                lines.load(vertex_count, vertex_data, vertex_bytes,
                    static_cast<std::uint32_t>(geometry.indices.size()),
                    index_data, index_bytes);
                loaded = context.load_lines(key, lines);
            }
        }
        else {
            loaded = context.meshes.contains(key);
            if (!loaded) {
                vkkk::Mesh mesh({vkkk::VERTEX});
                mesh.load(vertex_count, vertex_data, vertex_bytes,
                    static_cast<std::uint32_t>(geometry.indices.size() / 3),
                    index_data, index_bytes);
                loaded = context.load_mesh(key, mesh);
            }
        }
        if (!loaded) {
            return Status::failure(StatusCode::resource_failure,
                "vkkk rejected shape '" + key + "'");
        }
    }
    catch (const std::exception& error) {
        return exception_status(StatusCode::resource_failure, error);
    }
    impl_->shapes.emplace(key, kind);
    return Status::success();
}

Status VkkkAdapter::prepare_scene_meshes(const SceneMeshSetup& setup) {
    if (auto status = impl_->require_context("prepare_scene_meshes"); !status) {
        return status;
    }
    auto& context = *impl_->context;
    auto& state = impl_->scene_meshes;
    const auto front_face = setup.right_handed
        ? vk::FrontFace::eClockwise : vk::FrontFace::eCounterClockwise;

    PipelineDesc phong;
    phong.name = kPhongPipeline;
    phong.vertex = {vkkk::phong_vert, "orl_scene_phong_vert"};
    phong.fragment = {vkkk::phong_frag, "orl_scene_phong_frag"};
    phong.cull_mode = vk::CullModeFlagBits::eBack;
    phong.front_face = front_face;
    phong.components = {vkkk::VERTEX, vkkk::NORMAL};
    state.phong_ready = vkkk_detail::create_pipeline(context, phong)
        && context.resize_pipeline_ssbo(kPhongPipeline, vkkk::buf::PhongInstanceAttrs, 1)
        && context.alloc_pipeline_ssbo(kPhongPipeline, vkkk::buf::PhongInstanceAttrs);

    PipelineDesc wire = phong;
    wire.name = kWirePipeline;
    wire.vertex = {vkkk::phong_vert, "orl_scene_wire_vert"};
    wire.fragment = {vkkk::phong_frag, "orl_scene_wire_frag"};
    wire.polygon_mode = vk::PolygonMode::eLine;
    wire.depth_write = false;
    state.wire_ready = vkkk_detail::create_pipeline(context, wire)
        && context.resize_pipeline_ssbo(kWirePipeline, vkkk::buf::PhongInstanceAttrs, 1)
        && context.alloc_pipeline_ssbo(kWirePipeline, vkkk::buf::PhongInstanceAttrs);

    PipelineDesc xray;
    xray.name = kXRayPipeline;
    xray.vertex.path = impl_->shader_path("xray.vert");
    xray.fragment.path = impl_->shader_path("xray.frag");
    xray.front_face = front_face;
    xray.depth_write = false;
    xray.alpha_blend = true;
    xray.components = {vkkk::VERTEX, vkkk::NORMAL};
    state.xray_ready = vkkk_detail::create_pipeline(context, xray)
        && context.resize_pipeline_ssbo(kXRayPipeline, kXRayInstanceBlock, 1)
        && context.alloc_pipeline_ssbo(kXRayPipeline, kXRayInstanceBlock);

    state.prepared = true;
    if (!state.phong_ready) {
        return Status::failure(StatusCode::resource_failure,
            "scene Phong pipeline is unavailable");
    }
    if (!state.xray_ready) {
        return Status::failure(StatusCode::unsupported,
            "scene x-ray pipeline is unavailable; x-ray falls back to Phong");
    }
    if (!state.wire_ready) {
        return Status::failure(StatusCode::unsupported,
            "scene wireframe pipeline is unavailable");
    }
    return Status::success();
}

Status VkkkAdapter::draw_scene_meshes(const SceneMeshDraw& draw) {
    if (auto status = impl_->require_frame("draw_scene_meshes"); !status) {
        return status;
    }
    if (!impl_->scene_meshes.prepared) {
        return Status::failure(StatusCode::not_initialized,
            "draw_scene_meshes requires prepare_scene_meshes");
    }

    const auto state = impl_->scene_meshes;
    const bool xray = draw.shading == SceneShading::xray && state.xray_ready;
    auto instances = scene_records(draw.instances);
    auto highlighted = scene_records(draw.highlighted);
    auto surface = phong_attrs(draw.surface);
    auto highlight = phong_attrs(draw.highlight);
    vkkk::PointLightUBO light{};
    light.vec = to_glm(draw.light.position);
    light.color = to_glm(draw.light.color);
    const auto xray_color = to_glm(draw.xray_color);

    impl_->queue(Pass::scene,
        [impl = impl_.get(), state, xray, instances = std::move(instances),
            highlighted = std::move(highlighted), surface, highlight, light, xray_color](
            vk::raii::CommandBuffer& cmd, std::uint32_t image_index) {
            auto& context = *impl->context;
            if (xray) {
                context.sync_ubo(kXRayPipeline, vkkk::buf::CameraUBO,
                    &impl->camera_ubo, image_index);
                for (const auto& instance : instances) {
                    XRayInstanceAttrs attrs{instance.model, xray_color};
                    context.sync_ssbo(kXRayPipeline, kXRayInstanceBlock, &attrs, image_index);
                    if (context.bind(*cmd, kXRayPipeline, image_index)) {
                        context.draw(*cmd, kXRayPipeline, instance.mesh, 1);
                    }
                }
            }
            else if (state.phong_ready) {
                context.sync_ubo(kPhongPipeline, vkkk::buf::CameraUBO,
                    &impl->camera_ubo, image_index);
                context.sync_ubo(kPhongPipeline, vkkk::buf::PointLightUBO, &light, image_index);
                for (const auto& instance : instances) {
                    auto attrs = surface;
                    attrs.model = instance.model;
                    context.sync_ssbo(kPhongPipeline, vkkk::buf::PhongInstanceAttrs,
                        &attrs, image_index);
                    if (context.bind(*cmd, kPhongPipeline, image_index)) {
                        context.draw(*cmd, kPhongPipeline, instance.mesh, 1);
                    }
                }
            }

            if (!state.wire_ready || highlighted.empty()) {
                return;
            }
            context.sync_ubo(kWirePipeline, vkkk::buf::CameraUBO,
                &impl->camera_ubo, image_index);
            context.sync_ubo(kWirePipeline, vkkk::buf::PointLightUBO, &light, image_index);
            for (const auto& instance : highlighted) {
                auto attrs = highlight;
                attrs.model = instance.model;
                context.sync_ssbo(kWirePipeline, vkkk::buf::PhongInstanceAttrs,
                    &attrs, image_index);
                if (context.bind(*cmd, kWirePipeline, image_index)) {
                    context.draw(*cmd, kWirePipeline, instance.mesh, 1);
                }
            }
        });
    return Status::success();
}

Status VkkkAdapter::prepare_joints(const JointDisplaySetup& setup) {
    if (auto status = impl_->require_context("prepare_joints"); !status) {
        return status;
    }
    if (setup.joint_stride == 0) {
        return Status::failure(StatusCode::invalid_argument,
            "prepare_joints needs a joint stride");
    }
    auto& context = *impl_->context;
    auto& state = impl_->joints;
    state.joint_stride = setup.joint_stride;

    PipelineDesc points;
    points.name = kJointPointPipeline;
    points.vertex.path = impl_->shader_path("point.vert");
    points.fragment.path = impl_->shader_path("point.frag");
    points.topology = vk::PrimitiveTopology::ePointList;
    state.points_ready = vkkk_detail::create_pipeline(context, points)
        && load_dummy_points(context, kJointDummyPoints)
        && context.resize_pipeline_ssbo(kJointPointPipeline, kJointsBlock, 1)
        && context.alloc_pipeline_ssbo(kJointPointPipeline, kJointsBlock);

    PipelineDesc lines;
    lines.name = kJointLinePipeline;
    lines.vertex.path = impl_->shader_path("line.vert");
    lines.fragment.path = impl_->shader_path("line.frag");
    lines.topology = vk::PrimitiveTopology::eLineList;
    lines.line_width = vkkk_detail::line_width_for(context, setup.line_width);
    lines.dynamic_line_width = context.wide_lines_enabled;
    lines.depth_write = false;
    state.lines_ready = vkkk_detail::create_pipeline(context, lines)
        && load_dummy_lines(context, kJointDummyLines)
        && context.resize_pipeline_ssbo(kJointLinePipeline, kJointsBlock, 1)
        && context.alloc_pipeline_ssbo(kJointLinePipeline, kJointsBlock);
    state.line_width_dynamic = state.lines_ready && context.wide_lines_enabled;

    state.prepared = true;
    if (!state.points_ready) {
        return Status::failure(StatusCode::resource_failure,
            "joint point-list pipeline is unavailable");
    }
    if (!state.lines_ready) {
        return Status::failure(StatusCode::resource_failure,
            "joint line-list pipeline is unavailable");
    }
    return Status::success();
}

Status VkkkAdapter::upload_device_joints(DeviceBuffer device, std::uint32_t count) {
    if (auto status = impl_->require_frame("upload_device_joints"); !status) {
        return status;
    }
    auto& state = impl_->joints;
    if (!state.prepared) {
        return Status::failure(StatusCode::not_initialized,
            "upload_device_joints requires prepare_joints");
    }
    state.device_upload_serial.reset();
    state.device_joint_count = 0;
    const auto bytes = static_cast<vk::DeviceSize>(count) * state.joint_stride;
    if (count == 0 || device.device_ptr == 0 || device.bytes < bytes) {
        return Status::failure(StatusCode::invalid_argument,
            "device joints are empty or smaller than the requested count");
    }

    auto& context = *impl_->context;
    const auto image_index = impl_->active_frame->image_index;
    try {
        const bool uploaded = context.resize_pipeline_ssbo(
                kJointPointPipeline, kJointsBlock, count)
            && context.resize_pipeline_ssbo(kJointLinePipeline, kJointsBlock, count)
            && context.write_pipeline_ssbo_from_cuda(kJointPointPipeline, kJointsBlock,
                image_index, device.device_ptr, bytes)
            && context.write_pipeline_ssbo_from_cuda(kJointLinePipeline, kJointsBlock,
                image_index, device.device_ptr, bytes);
        if (!uploaded) {
            return Status::failure(StatusCode::resource_failure,
                "vkkk could not copy device joints into the joint buffers");
        }
    }
    catch (const std::exception& error) {
        return exception_status(StatusCode::resource_failure, error);
    }
    state.device_upload_serial = impl_->active_frame->serial;
    state.device_joint_count = count;
    return Status::success();
}

Status VkkkAdapter::draw_joints(const JointDraw& draw) {
    if (auto status = impl_->require_frame("draw_joints"); !status) {
        return status;
    }
    const auto state = impl_->joints;
    if (!state.prepared) {
        return Status::failure(StatusCode::not_initialized,
            "draw_joints requires prepare_joints");
    }
    std::uint32_t joint_count = draw.joint_count;
    std::vector<std::byte> joints;
    if (draw.use_device_joints) {
        if (state.device_upload_serial != impl_->active_frame->serial) {
            return Status::failure(StatusCode::invalid_argument,
                "draw_joints uses device joints that were not uploaded this frame");
        }
        joint_count = std::min(joint_count, state.device_joint_count);
    }
    else {
        if (draw.joints.size() < static_cast<std::size_t>(joint_count) * state.joint_stride) {
            return Status::failure(StatusCode::invalid_argument,
                "draw_joints host data is smaller than joint_count records");
        }
        joints.assign(draw.joints.begin(),
            draw.joints.begin() + static_cast<std::ptrdiff_t>(joint_count * state.joint_stride));
    }
    if (joint_count == 0) {
        return Status::success();
    }

    auto& context = *impl_->context;
    const glm::vec4 point_size{
        vkkk_detail::point_size_for(context, draw.point_size), 0.0f, 0.0f, 0.0f};
    const glm::vec4 line_width{
        vkkk_detail::line_width_for(context, draw.line_width), 0.0f, 0.0f, 0.0f};
    const auto point_color = to_glm(draw.point_color);
    const auto line_color = to_glm(draw.line_color);
    const bool host = !draw.use_device_joints;
    const bool lines = draw.lines && state.lines_ready;
    const bool points = draw.points && state.points_ready;

    impl_->queue(Pass::scene,
        [impl = impl_.get(), state, host, lines, points, joint_count,
            joints = std::move(joints), point_size, line_width, point_color, line_color](
            vk::raii::CommandBuffer& cmd, std::uint32_t image_index) {
            auto& context = *impl->context;
            const auto bytes = static_cast<std::uint32_t>(joints.size());
            if (lines
                && (!host || context.resize_pipeline_ssbo(
                    kJointLinePipeline, kJointsBlock, joint_count)))
            {
                if (host) {
                    context.sync_ssbo(kJointLinePipeline, kJointsBlock, joints.data(),
                        image_index, bytes);
                }
                context.sync_ubo(kJointLinePipeline, vkkk::buf::CameraUBO,
                    &impl->camera_ubo, image_index);
                context.sync_ubo(kJointLinePipeline, kLineWidthBlock, &line_width, image_index);
                context.sync_ubo(kJointLinePipeline, kLineColorBlock, &line_color, image_index);
                if (context.bind(*cmd, kJointLinePipeline, image_index)) {
                    if (state.line_width_dynamic) {
                        cmd.setLineWidth(line_width.x);
                    }
                    context.draw_lines(*cmd, kJointDummyLines, 0, joint_count);
                }
            }

            if (points
                && (!host || context.resize_pipeline_ssbo(
                    kJointPointPipeline, kJointsBlock, joint_count)))
            {
                if (host) {
                    context.sync_ssbo(kJointPointPipeline, kJointsBlock, joints.data(),
                        image_index, bytes);
                }
                context.sync_ubo(kJointPointPipeline, vkkk::buf::CameraUBO,
                    &impl->camera_ubo, image_index);
                context.sync_ubo(kJointPointPipeline, kPointSizeBlock, &point_size, image_index);
                context.sync_ubo(kJointPointPipeline, kPointColorBlock, &point_color, image_index);
                if (context.bind(*cmd, kJointPointPipeline, image_index)) {
                    context.draw_points(*cmd, kJointDummyPoints, 0, joint_count);
                }
            }
        });
    return Status::success();
}

Status VkkkAdapter::prepare_controllers() {
    if (auto status = impl_->require_context("prepare_controllers"); !status) {
        return status;
    }
    auto& context = *impl_->context;
    auto& state = impl_->controllers;

    PipelineDesc lines;
    lines.name = kControllerLinePipeline;
    lines.vertex.path = impl_->shader_path("controller.vert");
    lines.fragment.path = impl_->shader_path("line.frag");
    lines.topology = vk::PrimitiveTopology::eLineList;
    lines.line_width = vkkk_detail::line_width_for(context, kControllerLineWidth);
    lines.depth_write = false;
    state.lines_ready = vkkk_detail::create_pipeline(context, lines);

    PipelineDesc poly = lines;
    poly.name = kControllerPolyPipeline;
    poly.topology = vk::PrimitiveTopology::eTriangleList;
    poly.line_width = 1.0f;
    poly.depth_write = true;
    state.poly_ready = vkkk_detail::create_pipeline(context, poly);

    state.prepared = true;
    if (!state.lines_ready) {
        return Status::failure(StatusCode::resource_failure,
            "controller curve pipeline is unavailable");
    }
    if (!state.poly_ready) {
        return Status::failure(StatusCode::resource_failure,
            "controller polygon pipeline is unavailable");
    }
    return Status::success();
}

Status VkkkAdapter::draw_controller(const ShapeInstanceDraw& draw) {
    if (auto status = impl_->require_frame("draw_controller"); !status) {
        return status;
    }
    const auto state = impl_->controllers;
    if (!state.prepared) {
        return Status::failure(StatusCode::not_initialized,
            "draw_controller requires prepare_controllers");
    }
    const auto found = impl_->shapes.find(draw.shape);
    if (found == impl_->shapes.end()) {
        return Status::failure(StatusCode::invalid_argument,
            "draw_controller: shape '" + draw.shape + "' is not registered");
    }
    const bool lines = found->second == ShapeKind::lines;
    if (lines ? !state.lines_ready : !state.poly_ready) {
        return Status::success();
    }

    const char* pipeline = lines ? kControllerLinePipeline : kControllerPolyPipeline;
    const auto model = to_glm(draw.model);
    const auto color = to_glm(draw.color);
    impl_->queue(Pass::scene,
        [impl = impl_.get(), lines, pipeline, shape = draw.shape, model, color](
            vk::raii::CommandBuffer& cmd, std::uint32_t image_index) {
            auto& context = *impl->context;
            context.sync_ubo(pipeline, vkkk::buf::CameraUBO, &impl->camera_ubo, image_index);
            context.sync_ubo(pipeline, kModelBlock, &model, image_index);
            context.sync_ubo(pipeline, kColorBlock, &color, image_index);
            if (!context.bind(*cmd, pipeline, image_index)) {
                return;
            }
            if (lines) {
                context.draw_lines(*cmd, shape);
            }
            else {
                context.draw(*cmd, pipeline, shape, 1);
            }
        });
    return Status::success();
}

Status VkkkAdapter::prepare_locators() {
    if (auto status = impl_->require_context("prepare_locators"); !status) {
        return status;
    }
    auto& context = *impl_->context;
    PipelineDesc host_desc;
    host_desc.name = kLocatorPipeline;
    host_desc.vertex.path = impl_->shader_path("locator.vert");
    host_desc.fragment.path = impl_->shader_path("line.frag");
    host_desc.topology = vk::PrimitiveTopology::eLineList;
    host_desc.depth_write = false;
    impl_->locators.ready = vkkk_detail::create_pipeline(context, host_desc)
        && context.resize_pipeline_ssbo(kLocatorPipeline, kLocatorsBlock, 1)
        && context.alloc_pipeline_ssbo(kLocatorPipeline, kLocatorsBlock);
    PipelineDesc device_desc;
    device_desc.name = kLocatorDevicePipeline;
    device_desc.vertex.path = impl_->shader_path("locator_device.vert");
    device_desc.fragment.path = impl_->shader_path("line.frag");
    device_desc.topology = vk::PrimitiveTopology::eLineList;
    device_desc.depth_write = false;
    impl_->locators.device_ready =
        vkkk_detail::create_pipeline(context, device_desc)
        && context.resize_pipeline_ssbo(
            kLocatorDevicePipeline, kLocatorTransformsBlock, 1)
        && context.alloc_pipeline_ssbo(
            kLocatorDevicePipeline, kLocatorTransformsBlock)
        && context.resize_pipeline_ssbo(
            kLocatorDevicePipeline, kLocatorsBlock, 1)
        && context.alloc_pipeline_ssbo(
            kLocatorDevicePipeline, kLocatorsBlock);
    impl_->locators.prepared = true;
    if (!impl_->locators.ready || !impl_->locators.device_ready) {
        return Status::failure(StatusCode::resource_failure,
            "locator pipeline is unavailable");
    }
    return Status::success();
}

Status VkkkAdapter::upload_device_locators(
    DeviceBuffer device, std::uint32_t count)
{
    if (auto status = impl_->require_frame("upload_device_locators"); !status) {
        return status;
    }
    auto& state = impl_->locators;
    if (!state.prepared || !state.device_ready) {
        return Status::failure(StatusCode::not_initialized,
            "upload_device_locators requires prepare_locators");
    }
    state.device_upload_serial.reset();
    state.device_locator_count = 0;
    const auto bytes = static_cast<vk::DeviceSize>(count)
        * static_cast<vk::DeviceSize>(kDeviceLocatorStride);
    if (count == 0 || device.device_ptr == 0
        || device.bytes < static_cast<std::size_t>(bytes))
    {
        return Status::failure(StatusCode::invalid_argument,
            "device locators are empty or smaller than the requested count");
    }
    auto& context = *impl_->context;
    const auto image_index = impl_->active_frame->image_index;
    if (!context.resize_pipeline_ssbo(
            kLocatorDevicePipeline, kLocatorTransformsBlock, count)
        || !context.write_pipeline_ssbo_from_cuda(
            kLocatorDevicePipeline, kLocatorTransformsBlock,
            image_index, device.device_ptr, bytes))
    {
        return Status::failure(StatusCode::resource_failure,
            "vkkk could not copy device locators into the locator buffers");
    }
    state.device_upload_serial = impl_->active_frame->serial;
    state.device_locator_count = count;
    return Status::success();
}

Status VkkkAdapter::draw_locators(const LocatorDraw& draw)
{
    if (auto status = impl_->require_frame("draw_locators"); !status) {
        return status;
    }
    if (!impl_->locators.prepared) {
        return Status::failure(StatusCode::not_initialized,
            "draw_locators requires prepare_locators");
    }
    const std::string key{draw.shape};
    const auto found = impl_->shapes.find(key);
    if (found == impl_->shapes.end() || found->second != ShapeKind::lines) {
        return Status::failure(StatusCode::invalid_argument,
            "draw_locators: '" + key + "' is not a registered line shape");
    }
    const bool device = draw.use_device_locators;
    if (device && (!impl_->locators.device_ready
            || impl_->locators.device_upload_serial
                != impl_->active_frame->serial))
    {
        return Status::failure(StatusCode::invalid_argument,
            "draw_locators uses device locators that were not uploaded "
            "this frame");
    }
    if ((!device && !impl_->locators.ready) || draw.instances.empty()) {
        return Status::success();
    }
    std::uint32_t count = static_cast<std::uint32_t>(draw.instances.size());
    if (device) {
        count = std::min(count, impl_->locators.device_locator_count);
    }
    if (count == 0) {
        return Status::success();
    }
    const auto pipeline = device
        ? kLocatorDevicePipeline : kLocatorPipeline;
    auto records = std::vector<LocatorInstance>(
        draw.instances.begin(),
        draw.instances.begin() + static_cast<std::ptrdiff_t>(count));

    impl_->queue(Pass::scene,
        [impl = impl_.get(), pipeline, shape = key,
            count, records = std::move(records)](
            vk::raii::CommandBuffer& cmd, std::uint32_t image_index) {
            auto& context = *impl->context;
            if (!context.resize_pipeline_ssbo(
                    pipeline, kLocatorsBlock, records.size())) {
                return;
            }
            context.sync_ubo(pipeline, vkkk::buf::CameraUBO,
                &impl->camera_ubo, image_index);
            context.sync_ssbo(pipeline, kLocatorsBlock, records.data(), image_index,
                static_cast<std::uint32_t>(records.size() * sizeof(LocatorInstance)));
            if (context.bind(*cmd, pipeline, image_index)) {
                context.draw_lines(*cmd, shape, 0, count);
            }
        });
    return Status::success();
}

Status VkkkAdapter::prepare_ortho_grid() {
    if (auto status = impl_->require_context("prepare_ortho_grid"); !status) {
        return status;
    }
    auto& context = *impl_->context;
    PipelineDesc desc;
    desc.name = kOrthoGridPipeline;
    desc.vertex.path = impl_->shader_path("ortho_grid.vert");
    desc.fragment.path = impl_->shader_path("ortho_grid.frag");
    desc.depth_test = false;
    desc.depth_write = false;
    impl_->ortho_grid_ready = vkkk_detail::create_pipeline(context, desc)
        && load_dummy_triangle(context, kOrthoGridTriangle);
    impl_->ortho_grid_prepared = true;
    if (!impl_->ortho_grid_ready) {
        return Status::failure(StatusCode::resource_failure,
            "screen-space grid pipeline is unavailable");
    }
    return Status::success();
}

Status VkkkAdapter::draw_ortho_grid(const OrthoGridDraw& draw) {
    if (auto status = impl_->require_frame("draw_ortho_grid"); !status) {
        return status;
    }
    if (!impl_->ortho_grid_prepared) {
        return Status::failure(StatusCode::not_initialized,
            "draw_ortho_grid requires prepare_ortho_grid");
    }
    if (!impl_->ortho_grid_ready) {
        return Status::success();
    }
    const OrthoGridUBO ubo{
        to_glm(draw.origin_px_ppu),
        to_glm(draw.viewport_cell),
        to_glm(draw.line_color),
        to_glm(draw.axis_color),
    };
    impl_->queue(Pass::scene,
        [impl = impl_.get(), ubo](vk::raii::CommandBuffer& cmd, std::uint32_t image_index) {
            auto& context = *impl->context;
            context.sync_ubo(kOrthoGridPipeline, kOrthoGridBlock, &ubo, image_index);
            if (context.bind(*cmd, kOrthoGridPipeline, image_index)) {
                context.draw(*cmd, kOrthoGridPipeline, kOrthoGridTriangle, 1);
            }
        });
    return Status::success();
}

Status VkkkAdapter::prepare_overlay_lines() {
    if (auto status = impl_->require_context("prepare_overlay_lines"); !status) {
        return status;
    }
    auto& context = *impl_->context;
    PipelineDesc desc;
    desc.name = kOverlayLinePipeline;
    desc.vertex.path = impl_->shader_path("controller.vert");
    desc.fragment.path = impl_->shader_path("line.frag");
    desc.topology = vk::PrimitiveTopology::eLineList;
    desc.line_width = vkkk_detail::line_width_for(context, kControllerLineWidth);
    desc.dynamic_line_width = context.wide_lines_enabled;
    desc.depth_test = false;
    desc.depth_write = false;
    impl_->overlay_lines_ready = vkkk_detail::create_pipeline(context, desc);
    impl_->overlay_line_width_dynamic =
        impl_->overlay_lines_ready && context.wide_lines_enabled;
    impl_->overlay_lines_prepared = true;
    if (!impl_->overlay_lines_ready) {
        return Status::failure(StatusCode::resource_failure,
            "overlay line pipeline is unavailable");
    }
    return Status::success();
}

Status VkkkAdapter::draw_overlay_line(const ShapeInstanceDraw& draw) {
    if (auto status = impl_->require_frame("draw_overlay_line"); !status) {
        return status;
    }
    if (!impl_->overlay_lines_prepared) {
        return Status::failure(StatusCode::not_initialized,
            "draw_overlay_line requires prepare_overlay_lines");
    }
    const auto found = impl_->shapes.find(draw.shape);
    if (found == impl_->shapes.end() || found->second != ShapeKind::lines) {
        return Status::failure(StatusCode::invalid_argument,
            "draw_overlay_line: '" + draw.shape + "' is not a registered line shape");
    }
    if (!impl_->overlay_lines_ready) {
        return Status::success();
    }

    const auto model = to_glm(draw.model);
    const auto color = to_glm(draw.color);
    const float width = impl_->overlay_line_width_dynamic
        ? vkkk_detail::line_width_for(*impl_->context, kControllerLineWidth) : 0.0f;
    impl_->queue(Pass::overlay,
        [impl = impl_.get(), shape = draw.shape, model, color, width](
            vk::raii::CommandBuffer& cmd, std::uint32_t image_index) {
            auto& context = *impl->context;
            context.sync_ubo(kOverlayLinePipeline, vkkk::buf::CameraUBO,
                &impl->camera_ubo, image_index);
            context.sync_ubo(kOverlayLinePipeline, kModelBlock, &model, image_index);
            context.sync_ubo(kOverlayLinePipeline, kColorBlock, &color, image_index);
            if (context.bind(*cmd, kOverlayLinePipeline, image_index)) {
                if (impl->overlay_line_width_dynamic) {
                    cmd.setLineWidth(width);
                }
                context.draw_lines(*cmd, shape);
            }
        });
    return Status::success();
}

Status VkkkAdapter::prepare_status_text() {
    if (auto status = impl_->require_context("prepare_status_text"); !status) {
        return status;
    }
    impl_->status_text_prepared = true;
#if !ORL_USE_QT6
    if (impl_->hud == nullptr) {
        auto hud = std::make_unique<vkkk::ImGuiHud>();
        if (!hud->init(impl_->context.get())) {
            return Status::failure(StatusCode::unsupported,
                "the ImGui status HUD could not be initialized");
        }
        impl_->hud = std::move(hud);
    }
#endif
    return Status::success();
}

Status VkkkAdapter::draw_status_text(std::string_view text) {
    if (auto status = impl_->require_frame("draw_status_text"); !status) {
        return status;
    }
    if (!impl_->status_text_prepared) {
        return Status::failure(StatusCode::not_initialized,
            "draw_status_text requires prepare_status_text");
    }
#if ORL_USE_QT6
    impl_->window->set_status(std::string{text});
#else
    if (impl_->hud == nullptr) {
        return Status::success();
    }
    impl_->queue(Pass::overlay,
        [hud = impl_->hud.get(), text = std::string{text}](
            vk::raii::CommandBuffer& cmd, std::uint32_t) {
            hud->begin_frame();
            ImGui::SetNextWindowPos(ImVec2(12.0f, 12.0f), ImGuiCond_Always);
            ImGui::SetNextWindowBgAlpha(0.7f);
            ImGui::Begin("Compute device", nullptr,
                ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize
                    | ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoMove);
            ImGui::TextUnformatted(text.c_str());
            ImGui::End();
            hud->render(static_cast<VkCommandBuffer>(*cmd));
        });
#endif
    return Status::success();
}

Status VkkkAdapter::prepare_world_grid(const WorldGridSetup& setup) {
    if (auto status = impl_->require_context("prepare_world_grid"); !status) {
        return status;
    }
    if (impl_->world_grid == nullptr) {
        try {
            auto grid = std::make_unique<vkkk::vp::GridFeature>(
                impl_->camera_mirror, setup.cell_count, setup.cell_size);
            grid->on_attach(*impl_->context, vk::Extent2D{
                impl_->config.initial_extent.width, impl_->config.initial_extent.height});
            impl_->world_grid = std::move(grid);
        }
        catch (const std::exception& error) {
            return exception_status(StatusCode::resource_failure, error);
        }
    }
    return Status::success();
}

Status VkkkAdapter::draw_world_grid() {
    if (auto status = impl_->require_frame("draw_world_grid"); !status) {
        return status;
    }
    if (impl_->world_grid == nullptr) {
        return Status::failure(StatusCode::not_initialized,
            "draw_world_grid requires prepare_world_grid");
    }
    impl_->world_grid->on_update(*impl_->context, *impl_->active_frame);
    impl_->queue(Pass::scene,
        [impl = impl_.get()](vk::raii::CommandBuffer& cmd, std::uint32_t image_index) {
            impl->world_grid->on_record(*impl->context, cmd, image_index);
        });
    return Status::success();
}

Status VkkkAdapter::prepare_frame_axis(const FrameAxisSetup& setup) {
    if (auto status = impl_->require_context("prepare_frame_axis"); !status) {
        return status;
    }
    if (impl_->frame_axis == nullptr) {
        vkkk::vp::CoordinateSystem frame;
        for (std::size_t axis = 0; axis < frame.axes.size(); ++axis) {
            frame.axes[axis] = {setup.axes[axis][0], setup.axes[axis][1], setup.axes[axis][2]};
            frame.labels[axis] = setup.labels[axis];
        }
        try {
            auto gizmo = std::make_unique<vkkk::vp::FrameAxisFeature>(
                impl_->camera_mirror, std::filesystem::path{setup.font_path}, frame);
            gizmo->on_attach(*impl_->context, vk::Extent2D{
                impl_->config.initial_extent.width, impl_->config.initial_extent.height});
            impl_->frame_axis = std::move(gizmo);
        }
        catch (const std::exception& error) {
            return exception_status(StatusCode::resource_failure, error);
        }
    }
    return Status::success();
}

Status VkkkAdapter::draw_frame_axis() {
    if (auto status = impl_->require_frame("draw_frame_axis"); !status) {
        return status;
    }
    if (impl_->frame_axis == nullptr) {
        return Status::failure(StatusCode::not_initialized,
            "draw_frame_axis requires prepare_frame_axis");
    }
    impl_->frame_axis->on_update(*impl_->context, *impl_->active_frame);
    impl_->queue(Pass::overlay,
        [impl = impl_.get()](vk::raii::CommandBuffer& cmd, std::uint32_t image_index) {
            impl->frame_axis->on_record(*impl->context, cmd, image_index);
        });
    return Status::success();
}

} // namespace ORL::Rhi
