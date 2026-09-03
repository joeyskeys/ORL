#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "asset_mgr/scene.h"
#include "concepts/mesh.h"
#include "vp/feature.hpp"

namespace ORL
{

// Unique vertex-vertex CSR built on the GPU from triangle indices.
struct MeshCsr {
    std::uint32_t vertex_count = 0;
    std::vector<std::uint32_t> offsets;
    std::vector<std::uint32_t> neighbors;
};

// Viewer-only adjacency precompute. Not part of the ORL language.
class MeshCsrFeature final : public vkkk::vp::ViewportFeature<vkkk::vp::ViewportPhase::Scene> {
public:
    MeshCsrFeature(vkkk::Scene& scene, std::filesystem::path shader_dir);

    void on_attach(vkkk::Context& context, vk::Extent2D);
    void on_update(vkkk::Context& context, const vkkk::Context::Frame&);

    bool available() const { return ready; }
    const MeshCsr* find(const std::string& mesh_name) const;
    bool build(vkkk::Context& context, const std::string& mesh_name, const vkkk::Mesh& mesh);
    void clear();

private:
    bool prepare_buffers(vkkk::Context& context, std::uint32_t vertex_count,
        std::uint32_t triangle_count, std::uint32_t neighbor_capacity);
    bool dispatch_pass(vkkk::Context& context, std::uint32_t vertex_count,
        std::uint32_t triangle_count, std::uint32_t neighbor_capacity, std::uint32_t pass);
    bool read_uints(vkkk::Context& context, const char* block, std::uint32_t count,
        std::vector<std::uint32_t>& out) const;

    vkkk::Scene& scene;
    std::filesystem::path shader_dir;
    std::unordered_map<std::string, MeshCsr> csrs;
    std::unordered_set<std::string> failed;
    bool ready = false;
};

} // namespace ORL
