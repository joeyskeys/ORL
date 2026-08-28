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

#include <GLFW/glfw3.h>

#include "asset_mgr/scene.h"
#include "built_in_shader/common.h"
#include "concepts/camera.h"
#include "vk_ins/shader_module_pack.hpp"
#include "vp/feature.hpp"
#include "vp/object_picking.hpp"

namespace ORL
{

// GPU mesh pick: writes scene-object IDs into an R32Uint target, matching
// vkkk ObjectPickingFeature but request-driven so SelectOp owns the click.
class MeshPickingFeature final : public vkkk::vp::ViewportFeature<vkkk::vp::ViewportPhase::Picking> {
public:
    MeshPickingFeature(vkkk::Scene& scene, const vkkk::Camera& camera,
        std::filesystem::path shader_dir)
        : scene(scene)
        , camera(camera)
        , shader_dir(std::move(shader_dir))
    {
    }

    void on_attach(vkkk::Context& context, vk::Extent2D) {
        target_index = context.add_render_target(
            vk::ImageUsageFlagBits::eColorAttachment | vk::ImageUsageFlagBits::eTransferSrc,
            vk::Format::eR32Uint, 0, 0, vk::ImageLayout::eGeneral);
        ready = target_index != vkkk::kInvalidTargetIndex && create_pipeline(context);
        if (ready) {
            std::cout << "Mesh picking: GPU\n";
        }
        else {
            std::cout << "Mesh picking: GPU prepare failed\n";
        }
    }

    void on_resize(vkkk::Context&, vk::Extent2D) {
        pick_pending = false;
        last_render_serial = 0;
    }

    void on_update(vkkk::Context& context, const vkkk::Context::Frame& frame) {
        current_serial = frame.serial;
        sync_objects(context);
        if (!ready) {
            return;
        }

        if (pick_pending && last_render_serial >= pending_serial) {
            std::uint32_t object_id = 0;
            if (context.read_render_target_pixel(target_index, pending_x, pending_y, object_id)
                && hit_callback)
            {
                hit_callback(object_id);
            }
            pick_pending = false;
        }

        if (!want_pick || context.get_window() == nullptr) {
            return;
        }

        int window_width = 0;
        int window_height = 0;
        glfwGetWindowSize(context.get_window(), &window_width, &window_height);
        const auto extent = context.extent();
        if (window_width > 0 && window_height > 0 && extent.width > 0 && extent.height > 0) {
            pending_x = std::min(static_cast<std::uint32_t>(std::floor(
                cursor_x * static_cast<double>(extent.width) / window_width)), extent.width - 1);
            pending_y = std::min(static_cast<std::uint32_t>(std::floor(
                cursor_y * static_cast<double>(extent.height) / window_height)), extent.height - 1);
            pending_serial = frame.serial;
            pick_pending = true;
        }
        want_pick = false;
    }

    void on_record(vkkk::Context& context, vk::raii::CommandBuffer& cmd, std::uint32_t image_index) {
        if (!ready || !enabled || !pick_pending || instance_buffer_dirty
            || allocated_instance_count != instances.size())
        {
            return;
        }

        vkkk::ColorTargetRef<std::uint32_t> color{};
        color.target_index = static_cast<std::int32_t>(target_index);
        color.clear = {0u, 0u, 0u, 0u};
        vkkk::PassDescT<std::uint32_t> pass{};
        pass.colors = {color};
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
        last_render_serial = current_serial;
    }

    bool available() const { return ready && enabled; }

    bool request(double x, double y) {
        if (!available()) {
            return false;
        }
        cursor_x = x;
        cursor_y = y;
        want_pick = true;
        return true;
    }

    void set_hit_callback(std::function<void(std::uint32_t)> callback) {
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

    bool create_pipeline(vkkk::Context& context) {
        if (context.pipelines.contains(kPipeline)) {
            return true;
        }
        const auto vert_path = shader_dir / "object_picking.vert";
        const auto frag_path = shader_dir / "object_picking.frag";
        vkkk::ShaderModule vert_module;
        vkkk::ShaderModule frag_module;
        if (!vert_module.load(vert_path, vk::ShaderStageFlagBits::eVertex)
            || !frag_module.load(frag_path, vk::ShaderStageFlagBits::eFragment))
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
        option.setup_depth_stencil(true, true, vk::CompareOp::eLess, false, false);
        return context.create_pipeline(kPipeline, pack, option, {vkkk::VERTEX, vkkk::NORMAL},
            true, false, {vk::Format::eR32Uint});
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
    std::function<void(std::uint32_t)> hit_callback;
    std::uint32_t next_id = 1;
    std::uint32_t target_index = vkkk::kInvalidTargetIndex;
    std::uint32_t pending_x = 0;
    std::uint32_t pending_y = 0;
    std::uint64_t pending_serial = 0;
    std::uint64_t last_render_serial = 0;
    std::uint64_t current_serial = 0;
    std::size_t allocated_instance_count = 0;
    double cursor_x = 0.0;
    double cursor_y = 0.0;
    bool want_pick = false;
    bool pick_pending = false;
    bool instance_buffer_dirty = true;
    bool ready = false;
};

} // namespace ORL
