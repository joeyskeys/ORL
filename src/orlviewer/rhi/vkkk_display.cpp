#include "vkkk_adapter.hpp"

#include <string>
#include <utility>
#include <vector>

#include "built_in_shader/phong.h"
#include "concepts/line.h"
#include "concepts/mesh.h"
#include "vkkk_adapter_impl.hpp"

namespace ORL::Rhi {
namespace {

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

} // namespace ORL::Rhi
