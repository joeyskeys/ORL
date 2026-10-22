#include "vp/mesh_csr_feature.hpp"

#include <iostream>
#include <span>

#include "asset_mgr/drawable_mgr.h"

namespace ORL
{

MeshCsrFeature::MeshCsrFeature(vkkk::Scene& scene)
    : scene(scene)
{
}

void MeshCsrFeature::on_attach(ViewerAdapter& adapter, Rhi::Extent2D) {
    const auto status = adapter.prepare_mesh_adjacency();
    ready = static_cast<bool>(status);
    if (ready) {
        std::cout << "Mesh CSR: GPU\n";
    }
    else {
        std::cout << "Mesh CSR: CPU skipped (GPU prepare failed: " << status.message << ")\n";
    }
}

void MeshCsrFeature::on_update(ViewerAdapter& adapter, const Rhi::FrameState&) {
    if (!ready || scene.drawable_mgr == nullptr) {
        return;
    }
    for (const auto& name : scene.drawable_mgr->mesh_names()) {
        if (csrs.contains(name) || failed.contains(name)) {
            continue;
        }
        const auto* mesh = scene.drawable_mgr->find_mesh(name);
        if (mesh == nullptr || !build(adapter, name, *mesh)) {
            failed.insert(name);
        }
    }
}

const MeshCsr* MeshCsrFeature::find(const std::string& mesh_name) const {
    const auto it = csrs.find(mesh_name);
    return it == csrs.end() ? nullptr : &it->second;
}

void MeshCsrFeature::clear() {
    csrs.clear();
    failed.clear();
}

bool MeshCsrFeature::build(ViewerAdapter& adapter, const std::string& mesh_name,
    const vkkk::Mesh& mesh)
{
    if (!ready || !mesh.indexed || mesh.ibuf == nullptr || mesh.vcnt == 0 || mesh.icnt == 0) {
        return false;
    }

    // vkkk::Mesh::icnt counts triangles.
    MeshCsr csr;
    const auto status = adapter.build_mesh_adjacency({
        .triangle_indices = std::span<const std::uint32_t>{
            mesh.ibuf, static_cast<std::size_t>(mesh.icnt) * 3u},
        .vertex_count = mesh.vcnt,
    }, csr);
    if (!status) {
        std::cerr << "Mesh CSR: " << status.message << " for '" << mesh_name << "'\n";
        return false;
    }

    std::cout << "Mesh CSR: '" << mesh_name << "' " << csr.vertex_count << " verts, "
        << csr.neighbors.size() << " unique directed edges\n";
    csrs[mesh_name] = std::move(csr);
    return true;
}

} // namespace ORL
