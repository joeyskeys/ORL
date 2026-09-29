#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

#include "asset_mgr/scene.h"
#include "built_in_shader/common.h"
#include "concepts/camera.h"
#include "gui/input.hpp"
#include "vk_ins/shader_module_pack.hpp"
#include "vp/feature.hpp"
#include "vp/object_picking.hpp"
#include "vp/picking.hpp"

namespace ORL
{

// GPU mesh pick: writes scene-object IDs into an overlap-safe A-buffer so
// SelectOp can cycle through all mesh instances at one pixel.
class MeshPickingFeature final : public vkkk::vp::ViewportFeature<vkkk::vp::ViewportPhase::Picking> {
public:
    MeshPickingFeature(vkkk::Scene& scene, const vkkk::Camera& camera,
        std::filesystem::path shader_dir, std::uint32_t nodes_per_pixel = 8)
        : scene(scene)
        , camera(camera)
        , shader_dir(std::move(shader_dir))
        , nodes_per_pixel(nodes_per_pixel)
    {
    }

    void on_attach(vkkk::Context& context, vk::Extent2D) {
        ready = nodes_per_pixel != 0
            && create_pipeline(context)
            && resize_buffers(context, context.extent());
        if (ready) {
            std::cout << "Mesh picking: GPU\n";
        }
        else {
            std::cout << "Mesh picking: GPU prepare failed\n";
        }
    }

    void on_resize(vkkk::Context& context, vk::Extent2D extent) {
        pick_pending = false;
        last_render_serial = 0;
        ready = ready && resize_buffers(context, extent);
    }

    void on_update(vkkk::Context& context, const vkkk::Context::Frame& frame) {
        current_serial = frame.serial;
        sync_objects(context);
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

        if (context.window()->map_cursor(pick_event.x, pick_event.y, context.extent(),
                pending_x, pending_y))
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

    void on_record(vkkk::Context& context, vk::raii::CommandBuffer& cmd, std::uint32_t image_index) {
        if (!ready || !enabled || !pick_pending || instance_buffer_dirty
            || allocated_instance_count != instances.size())
        {
            return;
        }

        if (!context.clear_abuffer(kABufferName, image_index)) {
            return;
        }
        vkkk::PassDesc pass{};
        pass.colors.clear();
        pass.present = false;
        context.begin_pass(cmd, image_index, pass);
        if (!instances.empty()) {
            context.sync_ubo(kPipeline, vkkk::buf::CameraUBO, &camera.ubo_data, image_index);
            context.sync_ssbo(kPipeline, vkkk::buf::ObjectPickingInstances, instances.data(),
                image_index, static_cast<std::uint32_t>(instances.size() * sizeof(vkkk::vp::ObjectPickingInstance)));
            if (context.bind(cmd, kPipeline, image_index)) {
                for (std::uint32_t index = 0; index < static_cast<std::uint32_t>(instances.size()); ++index) {
                    context.draw(cmd, kPipeline, mesh_names[index], 1, index);
                }
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

    void set_hit_callback(
        std::function<void(
            std::uint64_t, const std::vector<GpuPickHit>&, bool)> callback)
    {
        hit_callback = std::move(callback);
    }

    const std::string& object_name(std::uint32_t object_id) const {
        const auto found = id_names.find(object_id);
        static const std::string empty;
        return found == id_names.end() ? empty : found->second;
    }

    bool enabled = true;

private:
    static constexpr const char* kPipeline = "orl_mesh_picking";
    static constexpr const char* kABufferName = "orl_mesh_picking_abuffer";

    bool create_pipeline(vkkk::Context& context) {
        if (context.pipelines.contains(kPipeline)) {
            return true;
        }
        const auto vert_path = shader_dir / "object_picking.vert";
        const auto frag_path = shader_dir / "object_picking_abuffer.frag";
        vkkk::ShaderModule vert_module;
        vkkk::ShaderModule frag_module;
        if (!context.load_shader(
                vert_module, vert_path, vk::ShaderStageFlagBits::eVertex)
            || !context.load_shader(
                frag_module, frag_path, vk::ShaderStageFlagBits::eFragment))
        {
            return false;
        }
        vkkk::ShaderModulePack pack;
        if (!pack.add_shader_module(vert_module) || !pack.add_shader_module(frag_module)) {
            return false;
        }
        vkkk::PipelineOption option;
        option.setup_input_assembly(vk::PrimitiveTopology::eTriangleList, false);
        option.setup_multisampling(false, vk::SampleCountFlagBits::e1);
        option.setup_rasterizer(false, false, vk::PolygonMode::eFill, 1.0f,
            vk::CullModeFlagBits::eNone, vk::FrontFace::eCounterClockwise, false);
        option.setup_depth_stencil(false, false, vk::CompareOp::eAlways, false, false);
        return context.create_pipeline(kPipeline, pack, option,
            {vkkk::VERTEX, vkkk::NORMAL}, true, true, {},
            static_cast<vk::Format>(context.get_depth_format()));
    }

    bool resize_buffers(vkkk::Context& context, vk::Extent2D extent) {
        return context.resize_abuffer(kABufferName, extent, nodes_per_pixel)
            && context.bind_pipeline_abuffer(kPipeline, kABufferName, 3, 4);
    }

    std::uint32_t id_for(const std::string& name) {
        const auto found = name_ids.find(name);
        if (found != name_ids.end()) {
            return found->second;
        }
        const auto id = next_id++;
        name_ids.emplace(name, id);
        id_names.emplace(id, name);
        return id;
    }

    void sync_objects(vkkk::Context& context) {
        std::vector<std::string> next_meshes;
        std::vector<vkkk::vp::ObjectPickingInstance> next_instances;
        scene.for_each_object([&](const std::string& name, const vkkk::SceneObject& object) {
            vkkk::vp::ObjectPickingInstance instance{};
            instance.model = object.model;
            instance.object_id = id_for(name);
            next_meshes.push_back(object.mesh_name);
            next_instances.push_back(instance);
        });
        if (next_meshes != mesh_names || next_instances.size() != instances.size()) {
            instance_buffer_dirty = true;
        }
        else {
            for (std::size_t i = 0; i < instances.size(); ++i) {
                if (next_instances[i].object_id != instances[i].object_id
                    || next_instances[i].model != instances[i].model)
                {
                    instance_buffer_dirty = true;
                    break;
                }
            }
        }
        mesh_names = std::move(next_meshes);
        instances = std::move(next_instances);
        if (!ready || !instance_buffer_dirty) {
            return;
        }
        if (instances.empty()) {
            allocated_instance_count = 0;
            instance_buffer_dirty = false;
            return;
        }
        if (context.resize_pipeline_ssbo(kPipeline, vkkk::buf::ObjectPickingInstances, instances.size())
            && context.alloc_pipeline_ssbo(kPipeline, vkkk::buf::ObjectPickingInstances))
        {
            allocated_instance_count = instances.size();
            instance_buffer_dirty = false;
        }
    }

    vkkk::Scene& scene;
    const vkkk::Camera& camera;
    std::filesystem::path shader_dir;
    std::vector<vkkk::vp::ObjectPickingInstance> instances;
    std::vector<std::string> mesh_names;
    std::unordered_map<std::string, std::uint32_t> name_ids;
    std::unordered_map<std::uint32_t, std::string> id_names;
    std::function<void(
        std::uint64_t, const std::vector<GpuPickHit>&, bool)> hit_callback;
    std::uint32_t next_id = 1;
    std::uint32_t last_image_index = 0;
    std::uint32_t pending_x = 0;
    std::uint32_t pending_y = 0;
    std::uint64_t pending_serial = 0;
    std::uint64_t last_render_serial = 0;
    std::uint64_t current_serial = 0;
    std::uint64_t next_request_id = 0;
    std::uint64_t requested_request_id = 0;
    std::uint64_t pending_request_id = 0;
    std::uint32_t nodes_per_pixel = 8;
    std::size_t allocated_instance_count = 0;
    vkkk::InputEvent pick_event;
    bool want_pick = false;
    bool pick_pending = false;
    bool instance_buffer_dirty = true;
    bool ready = false;
};

} // namespace ORL
