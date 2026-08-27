#include "vp/mesh_csr_feature.hpp"

#include <algorithm>
#include <cstring>
#include <iostream>

#include <glm/vec4.hpp>

#include "utils/sizeable.hpp"
#include "vk_ins/context.hpp"

namespace ORL
{
namespace
{

constexpr const char* kPipeline = "orl_mesh_csr";
constexpr const char* kParams = "CsrBuildParams";
constexpr const char* kIndices = "Indices";
constexpr const char* kDegrees = "Degrees";
constexpr const char* kOffsets = "Offsets";
constexpr const char* kFill = "Fill";
constexpr const char* kNeighbors = "Neighbors";
constexpr const char* kPacked = "Packed";
constexpr std::uint32_t kGroupSize = 64;
constexpr std::uint32_t kFrame = 0;

struct CsrBuildParamsUBO : public vkkk::Sizeable<CsrBuildParamsUBO> {
    glm::uvec4 counts{0};
};

std::uint32_t groups_for(std::uint32_t count) {
    return std::max(1u, (count + kGroupSize - 1) / kGroupSize);
}

} // namespace

MeshCsrFeature::MeshCsrFeature(vkkk::Scene& scene, std::filesystem::path shader_dir)
    : scene(scene)
    , shader_dir(std::move(shader_dir))
{
}

void MeshCsrFeature::on_attach(vkkk::Context& context, vk::Extent2D) {
    ready = context.load_compute_pipeline(kPipeline, shader_dir / "mesh_csr.comp");
    if (ready) {
        std::cout << "Mesh CSR: GPU\n";
    }
    else {
        std::cout << "Mesh CSR: CPU skipped (GPU prepare failed)\n";
    }
}

void MeshCsrFeature::on_update(vkkk::Context& context, const vkkk::Context::Frame&) {
    if (!ready || scene.drawable_mgr == nullptr) {
        return;
    }
    for (const auto& name : scene.drawable_mgr->mesh_names()) {
        if (csrs.contains(name) || failed.contains(name)) {
            continue;
        }
        const auto* mesh = scene.drawable_mgr->find_mesh(name);
        if (mesh == nullptr || !build(context, name, *mesh)) {
            failed.insert(name);
        }
    }
}

const MeshCsr* MeshCsrFeature::find(const std::string& mesh_name) const {
    const auto it = csrs.find(mesh_name);
    return it == csrs.end() ? nullptr : &it->second;
}

bool MeshCsrFeature::build(vkkk::Context& context, const std::string& mesh_name,
    const vkkk::Mesh& mesh)
{
    if (!ready || !mesh.indexed || mesh.ibuf == nullptr || mesh.vcnt == 0 || mesh.icnt == 0) {
        return false;
    }

    const auto vertex_count = mesh.vcnt;
    const auto triangle_count = mesh.icnt;
    const auto neighbor_capacity = std::max(1u, triangle_count * 6u);
    if (!prepare_buffers(context, vertex_count, triangle_count, neighbor_capacity)) {
        std::cerr << "Mesh CSR: buffer alloc failed for '" << mesh_name << "'\n";
        return false;
    }

    const auto index_bytes = static_cast<std::uint32_t>(triangle_count * 3u * sizeof(std::uint32_t));
    if (!context.sync_compute_ssbo(kPipeline, kIndices, mesh.ibuf, kFrame, index_bytes)) {
        std::cerr << "Mesh CSR: index upload failed for '" << mesh_name << "'\n";
        return false;
    }

    for (std::uint32_t pass = 0; pass < 6; ++pass) {
        if (!dispatch_pass(context, vertex_count, triangle_count, neighbor_capacity, pass)) {
            std::cerr << "Mesh CSR: pass " << pass << " failed for '" << mesh_name << "'\n";
            return false;
        }
    }
    context.wait_idle();

    MeshCsr csr;
    csr.vertex_count = vertex_count;
    if (!read_uints(context, kOffsets, vertex_count + 1, csr.offsets)) {
        return false;
    }
    const auto packed_count = csr.offsets.back();
    if (packed_count > neighbor_capacity) {
        std::cerr << "Mesh CSR: packed count overflow for '" << mesh_name << "'\n";
        return false;
    }
    if (packed_count > 0 && !read_uints(context, kPacked, packed_count, csr.neighbors)) {
        return false;
    }

    std::cout << "Mesh CSR: '" << mesh_name << "' " << vertex_count << " verts, "
        << packed_count << " unique directed edges\n";
    csrs[mesh_name] = std::move(csr);
    return true;
}

bool MeshCsrFeature::prepare_buffers(vkkk::Context& context, std::uint32_t vertex_count,
    std::uint32_t triangle_count, std::uint32_t neighbor_capacity)
{
    return context.resize_compute_ssbo(kPipeline, kIndices, triangle_count * 3u)
        && context.resize_compute_ssbo(kPipeline, kDegrees, vertex_count)
        && context.resize_compute_ssbo(kPipeline, kOffsets, vertex_count + 1)
        && context.resize_compute_ssbo(kPipeline, kFill, vertex_count)
        && context.resize_compute_ssbo(kPipeline, kNeighbors, neighbor_capacity)
        && context.resize_compute_ssbo(kPipeline, kPacked, neighbor_capacity)
        && context.alloc_compute_ssbo(kPipeline, kIndices)
        && context.alloc_compute_ssbo(kPipeline, kDegrees)
        && context.alloc_compute_ssbo(kPipeline, kOffsets)
        && context.alloc_compute_ssbo(kPipeline, kFill)
        && context.alloc_compute_ssbo(kPipeline, kNeighbors)
        && context.alloc_compute_ssbo(kPipeline, kPacked);
}

bool MeshCsrFeature::dispatch_pass(vkkk::Context& context, std::uint32_t vertex_count,
    std::uint32_t triangle_count, std::uint32_t neighbor_capacity, std::uint32_t pass)
{
    CsrBuildParamsUBO params{};
    params.counts = glm::uvec4(vertex_count, triangle_count, neighbor_capacity, pass);
    try {
        auto& ubo = context.require_ubo(std::string(kPipeline) + ":" + kParams);
        if (kFrame >= ubo.memos.size()) {
            return false;
        }
        context.sync_uniform(ubo.memos[kFrame], &params, static_cast<std::uint32_t>(sizeof(params)));
    }
    catch (const std::exception&) {
        return false;
    }

    std::uint32_t items = 1;
    if (pass == 0 || pass == 4) {
        items = vertex_count;
    }
    else if (pass == 1 || pass == 3) {
        items = triangle_count;
    }
    context.dispatch_compute(kPipeline, groups_for(items), 1, 1, kFrame);
    return true;
}

bool MeshCsrFeature::read_uints(vkkk::Context& context, const char* block, std::uint32_t count,
    std::vector<std::uint32_t>& out) const
{
    if (count == 0) {
        out.clear();
        return true;
    }
    try {
        const auto& ssbo = context.require_compute_ssbo(std::string(kPipeline) + ":" + block);
        if (kFrame >= ssbo.memos.size()) {
            return false;
        }
        const auto bytes = static_cast<vk::DeviceSize>(count) * sizeof(std::uint32_t);
        auto& memo = const_cast<vk::raii::DeviceMemory&>(ssbo.memos[kFrame]);
        auto* mapped = static_cast<const std::uint32_t*>(memo.mapMemory(0, bytes));
        out.assign(mapped, mapped + count);
        memo.unmapMemory();
        return true;
    }
    catch (const std::exception& error) {
        std::cerr << "Mesh CSR: read '" << block << "' failed: " << error.what() << '\n';
        return false;
    }
}

} // namespace ORL
