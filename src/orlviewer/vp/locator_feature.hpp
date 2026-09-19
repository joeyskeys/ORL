#pragma once

#include <cstdint>
#include <filesystem>
#include <iostream>
#include <utility>
#include <vector>

#include <glm/mat4x4.hpp>
#include <glm/vec4.hpp>

#include "component_manager.hpp"
#include "concepts/camera.h"
#include "concepts/line.h"
#include "selection.hpp"
#include "vk_ins/shader_module_pack.hpp"
#include "vp/feature.hpp"

namespace ORL
{

struct LocatorDrawData {
    glm::mat4 model{1.0f};
    glm::vec4 color{0.95f, 0.78f, 0.28f, 1.0f};
};

class LocatorFeature final
    : public vkkk::vp::ViewportFeature<vkkk::vp::ViewportPhase::Scene> {
public:
    LocatorFeature(const ComponentManager& components,
        const vkkk::Camera& camera, std::filesystem::path shader_dir,
        const Selection& selection)
        : components(components)
        , camera(camera)
        , shader_dir(std::move(shader_dir))
        , selection(selection)
    {
    }

    void on_attach(vkkk::Context& context, vk::Extent2D) {
        ready = create_pipeline(context) && create_cross(context);
        if (ready) {
            ready = context.resize_pipeline_ssbo(
                         kPipeline, kLocatorsBlock, 1)
                && context.alloc_pipeline_ssbo(
                    kPipeline, kLocatorsBlock);
        }
        if (!ready) {
            std::cerr << "LocatorFeature: locator pipeline is unavailable\n";
        }
    }

    void on_record(vkkk::Context& context, vk::raii::CommandBuffer& cmd,
        uint32_t image_index)
    {
        if (!ready) {
            return;
        }

        std::vector<LocatorDrawData> locators;
        components.for_each([&](const Component& meta) {
            if (meta.kind != ComponentKind::Locator) {
                return;
            }
            const auto* locator = components.locator(meta.id);
            if (locator == nullptr) {
                return;
            }
            LocatorDrawData draw_data{};
            draw_data.model = locator->xform;
            draw_data.color = is_selected(meta.id)
                ? glm::vec4{0.2f, 0.55f, 1.0f, 1.0f}
                : glm::vec4{0.95f, 0.78f, 0.28f, 1.0f};
            locators.push_back(draw_data);
        });
        if (locators.empty()
            || !context.resize_pipeline_ssbo(
                kPipeline, kLocatorsBlock, locators.size()))
        {
            return;
        }

        context.sync_ubo(kPipeline, vkkk::buf::CameraUBO,
            &camera.ubo_data, image_index);
        context.sync_ssbo(
            kPipeline, kLocatorsBlock, locators.data(), image_index,
            static_cast<uint32_t>(
                locators.size() * sizeof(LocatorDrawData)));
        if (context.bind(cmd, kPipeline, image_index)) {
            context.draw_lines(
                cmd, kCross, 0,
                static_cast<uint32_t>(locators.size()));
        }
    }

private:
    static constexpr const char* kPipeline = "orl_locators";
    static constexpr const char* kLocatorsBlock = "Locators";
    static constexpr const char* kCross = "orl_locator_cross";

    bool is_selected(ComponentId id) const {
        for (const auto& ref : selection.refs()) {
            if (ref.kind == SelectionRef::Kind::Locator
                && ref.component == id)
            {
                return true;
            }
        }
        return false;
    }

    bool create_pipeline(vkkk::Context& context) {
        if (context.pipelines.contains(kPipeline)) {
            return true;
        }
        vkkk::ShaderModule vert_module;
        vkkk::ShaderModule frag_module;
        if (!context.load_shader(
                vert_module, shader_dir / "locator.vert",
                vk::ShaderStageFlagBits::eVertex)
            || !context.load_shader(
                frag_module, shader_dir / "line.frag",
                vk::ShaderStageFlagBits::eFragment))
        {
            return false;
        }
        vkkk::ShaderModulePack pack;
        if (!pack.add_shader_module(vert_module)
            || !pack.add_shader_module(frag_module))
        {
            return false;
        }
        vkkk::PipelineOption option;
        option.setup_input_assembly(vk::PrimitiveTopology::eLineList, false);
        option.setup_multisampling(false, vk::SampleCountFlagBits::e1);
        option.setup_rasterizer(false, false, vk::PolygonMode::eFill, 1.0f,
            vk::CullModeFlagBits::eNone,
            vk::FrontFace::eCounterClockwise, false);
        option.setup_depth_stencil(true, false,
            vk::CompareOp::eLessOrEqual, false, false);
        return context.create_pipeline(
            kPipeline, pack, option, {vkkk::VERTEX});
    }

    bool create_cross(vkkk::Context& context) {
        if (context.lines.contains(kCross)) {
            return true;
        }
        constexpr float vertices[] = {
            -0.15f, 0.0f, 0.0f, 0.15f, 0.0f, 0.0f,
            0.0f, -0.15f, 0.0f, 0.0f, 0.15f, 0.0f,
            0.0f, 0.0f, -0.15f, 0.0f, 0.0f, 0.15f,
        };
        constexpr std::uint32_t indices[] = {0, 1, 2, 3, 4, 5};
        vkkk::Lines lines({vkkk::VERTEX});
        lines.load(6, reinterpret_cast<const char*>(vertices),
            sizeof(vertices), 6,
            reinterpret_cast<const char*>(indices), sizeof(indices));
        return context.load_lines(kCross, lines);
    }

    const ComponentManager& components;
    const vkkk::Camera& camera;
    std::filesystem::path shader_dir;
    const Selection& selection;
    bool ready = false;
};

} // namespace ORL
