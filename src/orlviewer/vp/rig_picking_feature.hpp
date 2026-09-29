#pragma once

#include <cstdint>
#include <filesystem>
#include <functional>
#include <iostream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <glm/mat4x4.hpp>
#include <vulkan/vulkan.hpp>

#include "comps/controller_curves.hpp"
#include "component_manager.hpp"
#include "concepts/camera.h"
#include "gui/input.hpp"
#include "selection.hpp"
#include "vk_ins/shader_module_pack.hpp"
#include "vp/feature.hpp"
#include "vp/picking.hpp"

namespace ORL
{

// GPU picking for controller shapes and locators. The two domains share one
// A-buffer; the high bits of each token identify the domain so a token cannot
// collide between controllers and locators.
class RigPickingFeature final
    : public vkkk::vp::ViewportFeature<vkkk::vp::ViewportPhase::Picking> {
public:
    using HitCallback = std::function<void(
        std::uint64_t, const std::vector<GpuPickHit>&, bool)>;

    RigPickingFeature(const ComponentManager& components,
        const vkkk::Camera& camera, std::filesystem::path shader_dir,
        std::uint32_t nodes_per_pixel = 8)
        : components(components)
        , camera(camera)
        , shader_dir(std::move(shader_dir))
        , nodes_per_pixel(nodes_per_pixel)
    {
    }

    void on_attach(vkkk::Context& context, vk::Extent2D extent) {
        ready = nodes_per_pixel != 0
            && create_controller_pipelines(context)
            && create_locator_pipeline(context)
            && create_controller_curves(context)
            && create_quad(context)
            && context.resize_pipeline_ssbo(
                kLocatorPipeline, kLocatorsBlock, 1)
            && context.alloc_pipeline_ssbo(
                kLocatorPipeline, kLocatorsBlock)
            && resize_buffers(context, extent);
        if (ready) {
            std::cout << "Rig picking: GPU\n";
        }
        else {
            std::cout << "Rig picking: CPU fallback (GPU prepare failed)\n";
        }
    }

    void on_resize(vkkk::Context& context, vk::Extent2D extent) {
        want_pick = false;
        pick_pending = false;
        last_render_serial = 0;
        ready = ready && resize_buffers(context, extent);
    }

    void on_update(vkkk::Context& context, const vkkk::Context::Frame& frame) {
        current_serial = frame.serial;
        if (!ready) {
            return;
        }

        if (pick_pending && last_render_serial >= pending_serial) {
            std::vector<vkkk::ABufferHit> raw_hits;
            std::vector<GpuPickHit> hits;
            bool overflow = false;
            context.read_abuffer_pixel(kABufferName, last_image_index,
                pending_x, pending_y, raw_hits, overflow);
            hits.reserve(raw_hits.size());
            for (const auto& hit : raw_hits) {
                hits.push_back(GpuPickHit{hit.vertex_id, hit.depth});
            }
            if (hit_callback) {
                hit_callback(pending_request_id, hits, overflow);
            }
            pick_pending = false;
        }

        if (!want_pick || context.window() == nullptr) {
            return;
        }
        if (context.window()->map_cursor(pick_event.x, pick_event.y,
                context.extent(), pending_x, pending_y))
        {
            pending_serial = frame.serial;
            pending_request_id = requested_request_id;
            pick_pending = true;
        }
        else if (hit_callback) {
            hit_callback(requested_request_id, {}, false);
        }
        want_pick = false;
    }

    void on_record(vkkk::Context& context, vk::raii::CommandBuffer& cmd,
        std::uint32_t image_index)
    {
        if (!ready || !enabled || !pick_pending
            || !context.clear_abuffer(kABufferName, image_index))
        {
            return;
        }

        sync_controller_ids();
        std::vector<LocatorPickData> locators;
        components.for_each([&](const Component& meta) {
            if (meta.kind != ComponentKind::Locator
                || components.locator(meta.id) == nullptr)
            {
                return;
            }
            locators.push_back(LocatorPickData{
                .model = components.locator(meta.id)->xform,
                .token = locator_token(meta.id),
            });
        });

        const bool locator_buffer_ready = !locators.empty()
            && context.resize_pipeline_ssbo(
                kLocatorPipeline, kLocatorsBlock, locators.size())
            && context.alloc_pipeline_ssbo(
                kLocatorPipeline, kLocatorsBlock);

        vkkk::PassDesc pass{};
        pass.colors.clear();
        pass.present = false;
        context.begin_pass(cmd, image_index, pass);

        context.sync_ubo(kControllerLinePipeline, vkkk::buf::CameraUBO,
            &camera.ubo_data, image_index);
        context.sync_ubo(kControllerPolyPipeline, vkkk::buf::CameraUBO,
            &camera.ubo_data, image_index);
        context.sync_ubo(kLocatorPipeline, vkkk::buf::CameraUBO,
            &camera.ubo_data, image_index);

        for (std::size_t index = 0; index < controller_ids.size(); ++index) {
            const auto id = controller_ids[index];
            const auto shape = controller_shapes[index];
            const auto model = components.controller_world_xform(id);
            const std::uint32_t token = controller_token(id);
            PickId pick_id{token};
            if (shape == orlviewer::ControllerShape::Polygon) {
                ControllerModelUBO model_ubo{};
                model_ubo.model = model;
                context.sync_ubo(kControllerPolyPipeline, kModelBlock,
                    &model_ubo, image_index);
                if (context.bind(cmd, kControllerPolyPipeline, image_index)) {
                    context.push_constants(cmd, kControllerPolyPipeline,
                        kPickIdBlock, &pick_id, sizeof(pick_id));
                    context.draw(cmd, kControllerPolyPipeline, kQuad, 1);
                }
                continue;
            }

            ControllerModelUBO model_ubo{};
            model_ubo.model = model;
            context.sync_ubo(kControllerLinePipeline, kModelBlock,
                &model_ubo, image_index);
            if (context.bind(cmd, kControllerLinePipeline, image_index)) {
                context.push_constants(cmd, kControllerLinePipeline,
                    kPickIdBlock, &pick_id, sizeof(pick_id));
                context.draw_lines(cmd, std::string{
                    orlviewer::controller_curves::name(shape)});
            }
        }

        if (locator_buffer_ready) {
            context.sync_ssbo(kLocatorPipeline, kLocatorsBlock,
                locators.data(), image_index,
                static_cast<std::uint32_t>(
                    locators.size() * sizeof(LocatorPickData)));
            if (context.bind(cmd, kLocatorPipeline, image_index)) {
                const PickId no_controller_id{};
                context.push_constants(cmd, kLocatorPipeline, kPickIdBlock,
                    &no_controller_id, sizeof(no_controller_id));
                context.draw_lines(cmd, kCross, 0,
                    static_cast<std::uint32_t>(locators.size()));
            }
        }

        context.end_pass(cmd, image_index, pass);
        context.barrier_abuffer_for_host(cmd);
        last_image_index = image_index;
        last_render_serial = current_serial;
    }

    bool available() const { return ready && enabled; }

    std::uint64_t request(const vkkk::InputEvent& event) {
        if (!available()) {
            return 0;
        }
        pick_event = event;
        requested_request_id = ++next_request_id;
        want_pick = true;
        return requested_request_id;
    }

    void set_hit_callback(HitCallback callback) {
        hit_callback = std::move(callback);
    }

    bool resolve(std::uint32_t token, SelectionRef& ref) const {
        if ((token & kControllerDomain) == kControllerDomain) {
            const auto found = controller_ids_by_token.find(token);
            if (found == controller_ids_by_token.end()
                || components.controller(found->second) == nullptr)
            {
                return false;
            }
            ref = SelectionRef::controller(found->second);
            return true;
        }
        if ((token & kLocatorDomain) == kLocatorDomain) {
            const auto found = locator_ids_by_token.find(token);
            if (found == locator_ids_by_token.end()
                || components.locator(found->second) == nullptr)
            {
                return false;
            }
            ref = SelectionRef::locator(found->second);
            return true;
        }
        return false;
    }

    bool enabled = true;

private:
    struct ControllerModelUBO {
        glm::mat4 model{1.0f};
    };

    struct PickId {
        std::uint32_t value = 0;
    };

    struct LocatorPickData {
        glm::mat4 model{1.0f};
        std::uint32_t token = 0;
        std::uint32_t pad0 = 0;
        std::uint32_t pad1 = 0;
        std::uint32_t pad2 = 0;
    };

    static constexpr const char* kControllerLinePipeline =
        "orl_controller_picking_lines";
    static constexpr const char* kControllerPolyPipeline =
        "orl_controller_picking_poly";
    static constexpr const char* kLocatorPipeline = "orl_locator_picking";
    static constexpr const char* kModelBlock = "ModelUBO";
    static constexpr const char* kLocatorsBlock = "Locators";
    static constexpr const char* kPickIdBlock = "PickId";
    static constexpr const char* kQuad = "orl_controller_quad";
    static constexpr const char* kCross = "orl_locator_cross";
    static constexpr const char* kABufferName = "orl_rig_picking_abuffer";
    static constexpr std::uint32_t kControllerDomain = 0x40000000u;
    static constexpr std::uint32_t kLocatorDomain = 0x80000000u;

    bool create_controller_pipelines(vkkk::Context& context) {
        return create_controller_pipeline(context, kControllerLinePipeline,
                   vk::PrimitiveTopology::eLineList)
            && create_controller_pipeline(context, kControllerPolyPipeline,
                   vk::PrimitiveTopology::eTriangleList);
    }

    bool create_controller_pipeline(vkkk::Context& context,
        const char* name, vk::PrimitiveTopology topology)
    {
        if (context.pipelines.contains(name)) {
            return true;
        }
        vkkk::ShaderModule vert;
        vkkk::ShaderModule frag;
        if (!context.load_shader(vert, shader_dir / "controller_picking.vert",
                vk::ShaderStageFlagBits::eVertex)
            || !context.load_shader(frag, shader_dir / "selection_abuffer.frag",
                vk::ShaderStageFlagBits::eFragment))
        {
            return false;
        }
        vkkk::ShaderModulePack pack;
        if (!pack.add_shader_module(vert) || !pack.add_shader_module(frag)) {
            return false;
        }
        vkkk::PipelineOption option;
        option.setup_input_assembly(topology, false);
        option.setup_multisampling(false, vk::SampleCountFlagBits::e1);
        option.setup_rasterizer(false, false, vk::PolygonMode::eFill, 1.0f,
            vk::CullModeFlagBits::eNone, vk::FrontFace::eCounterClockwise,
            false);
        option.setup_depth_stencil(false, false, vk::CompareOp::eAlways,
            false, false);
        return context.create_pipeline(name, pack, option, {vkkk::VERTEX},
            true, true, {}, static_cast<vk::Format>(
                context.get_depth_format()));
    }

    bool create_locator_pipeline(vkkk::Context& context) {
        if (context.pipelines.contains(kLocatorPipeline)) {
            return true;
        }
        vkkk::ShaderModule vert;
        vkkk::ShaderModule frag;
        if (!context.load_shader(vert, shader_dir / "locator_picking.vert",
                vk::ShaderStageFlagBits::eVertex)
            || !context.load_shader(frag, shader_dir / "selection_abuffer.frag",
                vk::ShaderStageFlagBits::eFragment))
        {
            return false;
        }
        vkkk::ShaderModulePack pack;
        if (!pack.add_shader_module(vert) || !pack.add_shader_module(frag)) {
            return false;
        }
        vkkk::PipelineOption option;
        option.setup_input_assembly(vk::PrimitiveTopology::eLineList, false);
        option.setup_multisampling(false, vk::SampleCountFlagBits::e1);
        option.setup_rasterizer(false, false, vk::PolygonMode::eFill, 1.0f,
            vk::CullModeFlagBits::eNone, vk::FrontFace::eCounterClockwise,
            false);
        option.setup_depth_stencil(false, false, vk::CompareOp::eAlways,
            false, false);
        return context.create_pipeline(kLocatorPipeline, pack, option,
            {vkkk::VERTEX}, true, true, {}, static_cast<vk::Format>(
                context.get_depth_format()));
    }

    bool create_controller_curves(vkkk::Context& context) {
        for (const auto& definition : orlviewer::controller_curves::definitions) {
            const std::string name{definition.name};
            if (context.lines.contains(name)) {
                continue;
            }
            const auto lines = definition.curve().generate_lines(definition.segments);
            if (!context.load_lines(name, lines)) {
                return false;
            }
        }
        return true;
    }

    bool create_quad(vkkk::Context& context) {
        if (context.meshes.contains(kQuad)) {
            return true;
        }
        const auto points = orlviewer::controller_polygon_points();
        std::vector<float> vertices;
        vertices.reserve(points.size() * 3);
        for (const auto& point : points) {
            vertices.insert(vertices.end(), {point.x, point.y, point.z});
        }
        constexpr std::uint32_t indices[] = {0, 1, 2, 0, 2, 3};
        vkkk::Mesh mesh({vkkk::VERTEX});
        mesh.load(static_cast<std::uint32_t>(points.size()),
            reinterpret_cast<const char*>(vertices.data()),
            static_cast<std::uint32_t>(
                vertices.size() * sizeof(float)),
            2, reinterpret_cast<const char*>(indices), sizeof(indices));
        return context.load_mesh(kQuad, mesh);
    }

    bool resize_buffers(vkkk::Context& context, vk::Extent2D extent) {
        return context.resize_abuffer(kABufferName, extent, nodes_per_pixel)
            && context.bind_pipeline_abuffer(kControllerLinePipeline,
                kABufferName, 3, 4)
            && context.bind_pipeline_abuffer(kControllerPolyPipeline,
                kABufferName, 3, 4)
            && context.bind_pipeline_abuffer(kLocatorPipeline,
                kABufferName, 3, 4);
    }

    std::uint32_t controller_token(ComponentId id) {
        const auto found = controller_tokens.find(id.value);
        if (found != controller_tokens.end()) {
            return found->second;
        }
        const auto token = kControllerDomain | next_controller_token++;
        controller_tokens.emplace(id.value, token);
        controller_ids_by_token.emplace(token, id);
        return token;
    }

    std::uint32_t locator_token(ComponentId id) {
        const auto found = locator_tokens.find(id.value);
        if (found != locator_tokens.end()) {
            return found->second;
        }
        const auto token = kLocatorDomain | next_locator_token++;
        locator_tokens.emplace(id.value, token);
        locator_ids_by_token.emplace(token, id);
        return token;
    }

    void sync_controller_ids() {
        controller_ids.clear();
        controller_shapes.clear();
        components.for_each([&](const Component& meta) {
            if (meta.kind != ComponentKind::Controller
                || components.controller(meta.id) == nullptr)
            {
                return;
            }
            controller_ids.push_back(meta.id);
            controller_shapes.push_back(components.controller_shape(meta.id));
            controller_token(meta.id);
        });
    }

    const ComponentManager& components;
    const vkkk::Camera& camera;
    std::filesystem::path shader_dir;
    std::vector<ComponentId> controller_ids;
    std::vector<orlviewer::ControllerShape> controller_shapes;
    std::unordered_map<std::uint64_t, std::uint32_t> controller_tokens;
    std::unordered_map<std::uint32_t, ComponentId> controller_ids_by_token;
    std::unordered_map<std::uint64_t, std::uint32_t> locator_tokens;
    std::unordered_map<std::uint32_t, ComponentId> locator_ids_by_token;
    HitCallback hit_callback;
    std::uint32_t next_controller_token = 1;
    std::uint32_t next_locator_token = 1;
    std::uint32_t nodes_per_pixel = 8;
    std::uint32_t pending_x = 0;
    std::uint32_t pending_y = 0;
    std::uint32_t last_image_index = 0;
    std::uint64_t pending_serial = 0;
    std::uint64_t last_render_serial = 0;
    std::uint64_t current_serial = 0;
    std::uint64_t next_request_id = 0;
    std::uint64_t requested_request_id = 0;
    std::uint64_t pending_request_id = 0;
    vkkk::InputEvent pick_event;
    bool want_pick = false;
    bool pick_pending = false;
    bool ready = false;
};

} // namespace ORL
