#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include "asset_mgr/scene.h"
#include "concepts/mesh.h"
#include "orl_compute.hpp"
#include "vp/rhi_feature.hpp"

namespace ORL
{

// Published adjacency for one mesh. Host buffers are the stable transfer.
// Device views are set only while this mesh is the one still held by the
// compute execution.
struct MeshCsrBuffers {
    const exec::OrlBuffer* offsets = nullptr;
    const exec::OrlBuffer* neighbors = nullptr;
    std::optional<exec::DeviceBufferView> offsets_device;
    std::optional<exec::DeviceBufferView> neighbors_device;
};

// Standalone ORL compute adjacency. The rig pipeline does not run this.
class MeshCsrFeature final : public RhiFeature {
public:
    explicit MeshCsrFeature(vkkk::Scene& scene);

    void on_update(ViewerAdapter& adapter, const Rhi::FrameState&);

    MeshCsrBuffers find(const std::string& mesh_name);
    bool build(const std::string& mesh_name, const vkkk::Mesh& mesh);
    void clear();

private:
    struct Cached {
        exec::OrlBuffer offsets{"int", sizeof(std::int64_t)};
        exec::OrlBuffer neighbors{"int", sizeof(std::int64_t)};
    };

    vkkk::Scene& scene;
    exec::Backend backend;
    exec::MeshCsrCompute compute;
    std::unordered_map<std::string, std::unique_ptr<Cached>> csrs;
    std::unordered_set<std::string> failed;
    std::string device_mesh;
};

} // namespace ORL
