#include "vp/mesh_csr_feature.hpp"

#include <iostream>
#include <span>

#include "asset_mgr/drawable_mgr.h"
#include "orl_exec.hpp"
#include "runtime_config.hpp"

namespace ORL
{

MeshCsrFeature::MeshCsrFeature(vkkk::Scene& scene)
    : scene(scene)
    , backend(runtime_config.device == ComputeDevice::Gpu
        ? exec::Backend::Cuda
        : exec::Backend::Cpu)
    , compute(backend)
{
}

void MeshCsrFeature::on_update(ViewerAdapter&, const Rhi::FrameState&) {
    if (scene.drawable_mgr == nullptr) {
        return;
    }
    for (const auto& name : scene.drawable_mgr->mesh_names()) {
        if (csrs.contains(name) || failed.contains(name)) {
            continue;
        }
        const auto* mesh = scene.drawable_mgr->find_mesh(name);
        if (mesh == nullptr || !build(name, *mesh)) {
            failed.insert(name);
        }
    }
}

MeshCsrBuffers MeshCsrFeature::find(const std::string& mesh_name) {
    const auto it = csrs.find(mesh_name);
    if (it == csrs.end()) {
        return {};
    }
    MeshCsrBuffers result;
    result.offsets = &it->second->offsets;
    result.neighbors = &it->second->neighbors;
    if (mesh_name == device_mesh) {
        result.offsets_device = compute.offsets_device();
        result.neighbors_device = compute.neighbors_device();
    }
    return result;
}

void MeshCsrFeature::clear() {
    csrs.clear();
    failed.clear();
    device_mesh.clear();
}

bool MeshCsrFeature::build(const std::string& mesh_name, const vkkk::Mesh& mesh)
{
    if (csrs.contains(mesh_name)) {
        return true;
    }
    if (!mesh.indexed || mesh.ibuf == nullptr || mesh.vcnt == 0 || mesh.icnt == 0) {
        return false;
    }

    const std::span<const std::uint32_t> indices{
        mesh.ibuf, static_cast<std::size_t>(mesh.icnt) * 3u};
    bool built = compute.build(indices, mesh.vcnt);
    if (!built && backend == exec::Backend::Cuda) {
        std::cerr << "Mesh CSR: CUDA compute failed, retrying on CPU\n";
        for (const auto& error : compute.errors()) {
            std::cerr << "Mesh CSR: " << error << '\n';
        }
        compute = exec::MeshCsrCompute(exec::Backend::Cpu);
        device_mesh.clear();
        built = compute.build(indices, mesh.vcnt);
    }
    if (!built) {
        for (const auto& error : compute.errors()) {
            std::cerr << "Mesh CSR: " << error << '\n';
        }
        return false;
    }

    auto cached = std::make_unique<Cached>();
    if (!exec::transfer_host_buffer(compute.offsets(), cached->offsets)
        || !exec::transfer_host_buffer(compute.neighbors(), cached->neighbors))
    {
        std::cerr << "Mesh CSR: failed to transfer '" << mesh_name << "'\n";
        return false;
    }
    device_mesh = compute.offsets_device().has_value() ? mesh_name : std::string{};
    std::cout << "Mesh CSR: '" << mesh_name << "' " << mesh.vcnt
        << " verts, " << cached->neighbors.count()
        << " unique directed edges\n";
    csrs.emplace(mesh_name, std::move(cached));
    return true;
}

} // namespace ORL
