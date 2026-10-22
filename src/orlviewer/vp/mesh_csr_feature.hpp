#pragma once

#include <string>
#include <unordered_map>
#include <unordered_set>

#include "asset_mgr/scene.h"
#include "concepts/mesh.h"
#include "vp/rhi_feature.hpp"

namespace ORL
{

// Unique vertex-vertex CSR built on the GPU from triangle indices.
using MeshCsr = Rhi::MeshAdjacency;

// Viewer-only adjacency precompute. Not part of the ORL language.
class MeshCsrFeature final : public RhiFeature {
public:
    explicit MeshCsrFeature(vkkk::Scene& scene);

    void on_attach(ViewerAdapter& adapter, Rhi::Extent2D);
    void on_update(ViewerAdapter& adapter, const Rhi::FrameState&);

    bool available() const { return ready; }
    const MeshCsr* find(const std::string& mesh_name) const;
    bool build(ViewerAdapter& adapter, const std::string& mesh_name, const vkkk::Mesh& mesh);
    void clear();

private:
    vkkk::Scene& scene;
    std::unordered_map<std::string, MeshCsr> csrs;
    std::unordered_set<std::string> failed;
    bool ready = false;
};

} // namespace ORL
