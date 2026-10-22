#pragma once

#include <iostream>
#include <string>
#include <string_view>
#include <vector>

#include "asset_mgr/scene.h"
#include "selection.hpp"
#include "vp/rhi_feature.hpp"

namespace ORL
{

class SceneMeshFeature final : public RhiFeature {
public:
    SceneMeshFeature(vkkk::Scene& scene, bool right_handed, const Selection& selection)
        : scene(scene)
        , right_handed(right_handed)
        , selection(selection)
    {
    }

    bool set_display_mode(std::string_view name) {
        if (name == "phong") {
            shading = Rhi::SceneShading::phong;
            return true;
        }
        if (name == "xray") {
            shading = Rhi::SceneShading::xray;
            return true;
        }
        return false;
    }

    std::string_view display_mode() const {
        return shading == Rhi::SceneShading::xray ? "xray" : "phong";
    }

    void on_attach(ViewerAdapter& adapter, Rhi::Extent2D) {
        const auto status = adapter.prepare_scene_meshes({.right_handed = right_handed});
        if (!status) {
            std::cerr << "SceneMeshFeature: " << status.message << '\n';
        }
    }

    void on_submit(ViewerAdapter& adapter, const Rhi::FrameState&) {
        if (scene.camera == nullptr || scene.object_count() == 0) {
            return;
        }

        instances.clear();
        scene.for_each_object([&](const std::string&, const vkkk::SceneObject& object) {
            instances.push_back({object.mesh_name, to_rhi(object.model)});
        });
        highlighted.clear();
        for (const auto& ref : selection.refs()) {
            if (ref.kind != SelectionRef::Kind::SceneObject) {
                continue;
            }
            if (const auto* object = scene.find_object(ref.object_name); object != nullptr) {
                highlighted.push_back({object->mesh_name, to_rhi(object->model)});
            }
        }

        const auto status = adapter.draw_scene_meshes({
            .shading = shading,
            .instances = instances,
            .highlighted = highlighted,
            .surface = surface_material,
            .highlight = highlight_material,
            .xray_color = xray_color,
            .light = light,
        });
        if (!status) {
            std::cerr << "SceneMeshFeature: " << status.message << '\n';
        }
    }

private:
    vkkk::Scene& scene;
    bool right_handed = true;
    const Selection& selection;
    Rhi::SceneShading shading = Rhi::SceneShading::phong;
    std::vector<Rhi::SceneMeshInstance> instances;
    std::vector<Rhi::SceneMeshInstance> highlighted;
    Rhi::SurfaceMaterial surface_material{
        .ambient = {0.08f, 0.08f, 0.08f, 1.0f},
        .diffuse = {0.72f, 0.72f, 0.72f, 1.0f},
        .specular = {0.35f, 0.35f, 0.35f, 1.0f},
        .shininess = 32.0f,
    };
    Rhi::SurfaceMaterial highlight_material{
        .ambient = {0.05f, 0.12f, 0.28f, 1.0f},
        .diffuse = {0.2f, 0.45f, 1.0f, 1.0f},
        .specular = {0.0f, 0.0f, 0.0f, 0.0f},
        .shininess = 1.0f,
    };
    Rhi::Vec4 xray_color{0.45f, 0.78f, 0.95f, 0.22f};
    Rhi::PointLight light{
        .position = {2.0f, 3.0f, 2.0f, 1.0f},
        .color = {1.0f, 1.0f, 1.0f, 1.0f},
    };
};

} // namespace ORL
