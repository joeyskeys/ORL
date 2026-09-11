#pragma once

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include <glm/mat4x4.hpp>
#include <glm/vec4.hpp>

#include "built_in_shader/common.h"
#include "component_manager.hpp"
#include "concepts/camera.h"
#include "concepts/line.h"
#include "concepts/mesh.h"
#include "selection.hpp"
#include "utils/sizeable.hpp"
#include "vk_ins/shader_module_pack.hpp"
#include "vp/feature.hpp"

namespace ORL
{

struct ControllerModelUBO : public vkkk::Sizeable<ControllerModelUBO> {
    glm::mat4 model{1.0f};
};

struct ControllerColorUBO : public vkkk::Sizeable<ControllerColorUBO> {
    glm::vec4 value{0.95f, 0.55f, 0.18f, 1.0f};
};

class ControllerFeature final : public vkkk::vp::ViewportFeature<vkkk::vp::ViewportPhase::Scene> {
public:
    ControllerFeature(const ComponentManager& components, const vkkk::Camera& camera,
        const Selection& selection, std::filesystem::path shader_dir)
        : components(components)
        , camera(camera)
        , selection(selection)
        , shader_dir(std::move(shader_dir))
    {
    }

    void on_attach(vkkk::Context& context, vk::Extent2D) {
        lines_ready = create_line_pipeline(context)
            && create_controller_curves(context);
        if (!lines_ready) {
            std::cerr << "ControllerFeature: curve pipeline is unavailable\n";
        }
        poly_ready = create_poly_pipeline(context) && create_quad(context);
        if (!poly_ready) {
            std::cerr << "ControllerFeature: polygon pipeline is unavailable\n";
        }
    }

    void on_record(vkkk::Context& context, vk::raii::CommandBuffer& cmd, uint32_t image_index) {
        if (components.size(ComponentKind::Controller) == 0) {
            return;
        }

        components.for_each([&](const Component& meta) {
            if (meta.kind != ComponentKind::Controller) {
                return;
            }
            const auto* controller = components.controller(meta.id);
            if (controller == nullptr) {
                return;
            }
            const bool selected = is_selected(meta.id);
            const auto shape = components.controller_shape(meta.id);
            if (shape == orlviewer::ControllerShape::Polygon)
            {
                record_poly(context, cmd, image_index, *controller, selected);
            } else {
                record_curve(context, cmd, image_index, *controller, shape, selected);
            }
        });
    }

private:
    static constexpr const char* kLinePipeline = "orl_controller_lines";
    static constexpr const char* kPolyPipeline = "orl_controller_poly";
    static constexpr const char* kModelBlock = "ModelUBO";
    static constexpr const char* kColorBlock = "ColorUBO";
    static constexpr const char* kQuad = "orl_controller_quad";

    bool is_selected(ComponentId id) const {
        for (const auto& ref : selection.refs()) {
            if (ref.kind == SelectionRef::Kind::Controller && ref.component == id) {
                return true;
            }
        }
        return false;
    }

    bool create_line_pipeline(vkkk::Context& context) {
        if (context.pipelines.contains(kLinePipeline)) {
            return true;
        }
        vkkk::ShaderModule vert_module;
        vkkk::ShaderModule frag_module;
        if (!vert_module.load(shader_dir / "controller.vert", vk::ShaderStageFlagBits::eVertex)
            || !frag_module.load(shader_dir / "line.frag", vk::ShaderStageFlagBits::eFragment))
        {
            return false;
        }
        vkkk::ShaderModulePack pack;
        if (!pack.add_shader_module(vert_module) || !pack.add_shader_module(frag_module)) {
            return false;
        }
        const float width = context.wide_lines_enabled
            ? std::clamp(2.5f, context.line_width_range[0], context.line_width_range[1])
            : 1.0f;
        vkkk::PipelineOption option;
        option.setup_input_assembly(vk::PrimitiveTopology::eLineList, false);
        option.setup_multisampling(false, vk::SampleCountFlagBits::e1);
        option.setup_rasterizer(false, false, vk::PolygonMode::eFill, width,
            vk::CullModeFlagBits::eNone, vk::FrontFace::eCounterClockwise, false);
        option.setup_depth_stencil(true, false, vk::CompareOp::eLessOrEqual, false, false);
        return context.create_pipeline(kLinePipeline, pack, option, {vkkk::VERTEX});
    }

    bool create_poly_pipeline(vkkk::Context& context) {
        if (context.pipelines.contains(kPolyPipeline)) {
            return true;
        }
        vkkk::ShaderModule vert_module;
        vkkk::ShaderModule frag_module;
        if (!vert_module.load(shader_dir / "controller.vert", vk::ShaderStageFlagBits::eVertex)
            || !frag_module.load(shader_dir / "line.frag", vk::ShaderStageFlagBits::eFragment))
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
        option.setup_depth_stencil(true, true, vk::CompareOp::eLessOrEqual, false, false);
        return context.create_pipeline(kPolyPipeline, pack, option, {vkkk::VERTEX});
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
        for (const auto& p : points) {
            vertices.insert(vertices.end(), {p.x, p.y, p.z});
        }
        const std::uint32_t indices[] = {0, 1, 2, 0, 2, 3};
        vkkk::Mesh mesh({vkkk::VERTEX});
        mesh.load(static_cast<std::uint32_t>(points.size()),
            reinterpret_cast<const char*>(vertices.data()),
            static_cast<std::uint32_t>(vertices.size() * sizeof(float)),
            2, reinterpret_cast<const char*>(indices), sizeof(indices));
        return context.load_mesh(kQuad, mesh);
    }

    ControllerColorUBO color_for(bool selected) const {
        ControllerColorUBO color{};
        color.value = selected
            ? glm::vec4{0.2f, 0.55f, 1.0f, 1.0f}
            : glm::vec4{0.95f, 0.55f, 0.18f, 1.0f};
        return color;
    }

    void record_curve(vkkk::Context& context, vk::raii::CommandBuffer& cmd,
        uint32_t image_index, const orlrig::Controller& controller,
        orlviewer::ControllerShape shape, bool selected)
    {
        if (!lines_ready) {
            return;
        }
        ControllerModelUBO model{};
        model.model = controller.xform;
        context.sync_ubo(kLinePipeline, vkkk::buf::CameraUBO, &camera.ubo_data, image_index);
        context.sync_ubo(kLinePipeline, kModelBlock, &model, image_index);
        const auto color = color_for(selected);
        context.sync_ubo(kLinePipeline, kColorBlock, &color, image_index);
        if (context.bind(cmd, kLinePipeline, image_index)) {
            context.draw_lines(cmd, std::string{
                orlviewer::controller_curves::name(shape)});
        }
    }

    void record_poly(vkkk::Context& context, vk::raii::CommandBuffer& cmd, uint32_t image_index,
        const orlrig::Controller& controller, bool selected)
    {
        if (!poly_ready) {
            return;
        }
        ControllerModelUBO model{};
        model.model = controller.xform;
        context.sync_ubo(kPolyPipeline, vkkk::buf::CameraUBO, &camera.ubo_data, image_index);
        context.sync_ubo(kPolyPipeline, kModelBlock, &model, image_index);
        const auto color = color_for(selected);
        context.sync_ubo(kPolyPipeline, kColorBlock, &color, image_index);
        if (context.bind(cmd, kPolyPipeline, image_index)) {
            context.draw(cmd, kPolyPipeline, kQuad, 1);
        }
    }

    const ComponentManager& components;
    const vkkk::Camera& camera;
    const Selection& selection;
    std::filesystem::path shader_dir;
    bool lines_ready = false;
    bool poly_ready = false;
};

} // namespace ORL
