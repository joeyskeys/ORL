#pragma once

#include <cstdint>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

#include "asset_mgr/scene.h"
#include "vp/picking.hpp"
#include "vp/rhi_feature.hpp"

namespace ORL
{

// GPU mesh pick: writes scene-object IDs into an overlap-safe A-buffer so
// SelectOp can cycle through all mesh instances at one pixel.
class MeshPickingFeature final : public RhiFeature {
public:
    explicit MeshPickingFeature(vkkk::Scene& scene, std::uint32_t nodes_per_pixel = 8)
        : scene(scene)
        , nodes_per_pixel(nodes_per_pixel)
    {
    }

    void on_attach(ViewerAdapter& adapter, Rhi::Extent2D) {
        ready = prepare(adapter);
        std::cout << (ready ? "Mesh picking: GPU\n" : "Mesh picking: GPU prepare failed\n");
    }

    void on_resize(ViewerAdapter& adapter, Rhi::Extent2D) {
        pick.reset();
        ready = ready && prepare(adapter);
    }

    void on_update(ViewerAdapter& adapter, const Rhi::FrameState& frame) {
        if (ready) {
            pick.update(adapter, frame);
        }
    }

    void on_submit(ViewerAdapter& adapter, const Rhi::FrameState&) {
        if (!ready || !enabled || !pick.pending()) {
            return;
        }
        instances.clear();
        scene.for_each_object([&](const std::string& name, const vkkk::SceneObject& object) {
            instances.push_back({object.mesh_name, to_rhi(object.model), id_for(name)});
        });
        const auto status = adapter.draw_mesh_pick(instances);
        if (!status) {
            std::cerr << "MeshPickingFeature: " << status.message << '\n';
        }
    }

    bool available() const { return ready && enabled; }

    std::uint64_t request(double x, double y) {
        return available() ? pick.request(x, y) : 0;
    }

    void set_hit_callback(PickRequest::HitCallback callback) {
        pick.set_hit_callback(std::move(callback));
    }

    const std::string& object_name(std::uint32_t object_id) const {
        const auto found = id_names.find(object_id);
        static const std::string empty;
        return found == id_names.end() ? empty : found->second;
    }

    bool enabled = true;

private:
    bool prepare(ViewerAdapter& adapter) {
        const auto status = adapter.prepare_pick_layer({
            .layer = Rhi::PickLayer::mesh,
            .nodes_per_pixel = nodes_per_pixel,
        });
        if (!status) {
            std::cerr << "MeshPickingFeature: " << status.message << '\n';
        }
        return static_cast<bool>(status);
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

    vkkk::Scene& scene;
    PickRequest pick{Rhi::PickLayer::mesh, "MeshPickingFeature"};
    std::vector<Rhi::MeshPickInstance> instances;
    std::unordered_map<std::string, std::uint32_t> name_ids;
    std::unordered_map<std::uint32_t, std::string> id_names;
    std::uint32_t next_id = 1;
    std::uint32_t nodes_per_pixel = 8;
    bool ready = false;
};

} // namespace ORL
