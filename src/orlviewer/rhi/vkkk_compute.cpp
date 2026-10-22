#include "vkkk_adapter.hpp"

#include <algorithm>
#include <string>
#include <vector>

#include <glm/vec4.hpp>

#include "vkkk_adapter_impl.hpp"

namespace ORL::Rhi {
namespace {

constexpr const char* kCsrPipeline = "orl_mesh_csr";
constexpr const char* kParams = "CsrBuildParams";
constexpr const char* kIndices = "Indices";
constexpr const char* kDegrees = "Degrees";
constexpr const char* kOffsets = "Offsets";
constexpr const char* kFill = "Fill";
constexpr const char* kNeighbors = "Neighbors";
constexpr const char* kPacked = "Packed";
constexpr std::uint32_t kGroupSize = 64;
constexpr std::uint32_t kComputeFrame = 0;
constexpr std::uint32_t kCsrPasses = 6;

struct CsrBuildParams {
    glm::uvec4 counts{0};
};

std::uint32_t groups_for(std::uint32_t count) {
    return std::max(1u, (count + kGroupSize - 1) / kGroupSize);
}

bool prepare_buffers(vkkk::Context& context, std::uint32_t vertex_count,
    std::uint32_t triangle_count, std::uint32_t neighbor_capacity)
{
    return context.resize_compute_ssbo(kCsrPipeline, kIndices, triangle_count * 3u)
        && context.resize_compute_ssbo(kCsrPipeline, kDegrees, vertex_count)
        && context.resize_compute_ssbo(kCsrPipeline, kOffsets, vertex_count + 1)
        && context.resize_compute_ssbo(kCsrPipeline, kFill, vertex_count)
        && context.resize_compute_ssbo(kCsrPipeline, kNeighbors, neighbor_capacity)
        && context.resize_compute_ssbo(kCsrPipeline, kPacked, neighbor_capacity)
        && context.alloc_compute_ssbo(kCsrPipeline, kIndices)
        && context.alloc_compute_ssbo(kCsrPipeline, kDegrees)
        && context.alloc_compute_ssbo(kCsrPipeline, kOffsets)
        && context.alloc_compute_ssbo(kCsrPipeline, kFill)
        && context.alloc_compute_ssbo(kCsrPipeline, kNeighbors)
        && context.alloc_compute_ssbo(kCsrPipeline, kPacked);
}

bool dispatch_pass(vkkk::Context& context, std::uint32_t vertex_count,
    std::uint32_t triangle_count, std::uint32_t neighbor_capacity, std::uint32_t pass)
{
    CsrBuildParams params{};
    params.counts = glm::uvec4(vertex_count, triangle_count, neighbor_capacity, pass);
    auto& ubo = context.require_ubo(std::string(kCsrPipeline) + ":" + kParams);
    if (kComputeFrame >= ubo.memos.size()) {
        return false;
    }
    context.sync_uniform(ubo.memos[kComputeFrame], &params,
        static_cast<std::uint32_t>(sizeof(params)));

    std::uint32_t items = 1;
    if (pass == 0 || pass == 4) {
        items = vertex_count;
    }
    else if (pass == 1 || pass == 3) {
        items = triangle_count;
    }
    context.dispatch_compute(kCsrPipeline, groups_for(items), 1, 1, kComputeFrame);
    return true;
}

void read_uints(vkkk::Context& context, const char* block, std::uint32_t count,
    std::vector<std::uint32_t>& out)
{
    if (count == 0) {
        out.clear();
        return;
    }
    const auto& ssbo = context.require_compute_ssbo(std::string(kCsrPipeline) + ":" + block);
    if (kComputeFrame >= ssbo.memos.size()) {
        throw std::runtime_error(std::string{"mesh CSR block '"} + block + "' has no memory");
    }
    const auto bytes = static_cast<vk::DeviceSize>(count) * sizeof(std::uint32_t);
    auto& memo = const_cast<vk::raii::DeviceMemory&>(ssbo.memos[kComputeFrame]);
    auto* mapped = static_cast<const std::uint32_t*>(memo.mapMemory(0, bytes));
    out.assign(mapped, mapped + count);
    memo.unmapMemory();
}

} // namespace

Status VkkkAdapter::prepare_mesh_adjacency() {
    if (auto status = impl_->require_context("prepare_mesh_adjacency"); !status) {
        return status;
    }
    if (!impl_->mesh_adjacency_ready) {
        try {
            impl_->mesh_adjacency_ready = impl_->context->load_compute_pipeline(
                kCsrPipeline, impl_->shader_path("mesh_csr.comp"));
        }
        catch (const std::exception& error) {
            return exception_status(StatusCode::resource_failure, error);
        }
    }
    if (!impl_->mesh_adjacency_ready) {
        return Status::failure(StatusCode::resource_failure,
            "mesh CSR compute pipeline is unavailable");
    }
    return Status::success();
}

Status VkkkAdapter::build_mesh_adjacency(const MeshAdjacencyInput& input,
    MeshAdjacency& output)
{
    if (auto status = impl_->require_context("build_mesh_adjacency"); !status) {
        return status;
    }
    if (!impl_->mesh_adjacency_ready) {
        return Status::failure(StatusCode::not_initialized,
            "build_mesh_adjacency requires prepare_mesh_adjacency");
    }
    if (input.vertex_count == 0 || input.triangle_indices.empty()
        || input.triangle_indices.size() % 3 != 0)
    {
        return Status::failure(StatusCode::invalid_argument,
            "build_mesh_adjacency needs vertices and complete triangles");
    }

    auto& context = *impl_->context;
    const auto vertex_count = input.vertex_count;
    const auto triangle_count = static_cast<std::uint32_t>(input.triangle_indices.size() / 3);
    const auto neighbor_capacity = std::max(1u, triangle_count * 6u);
    try {
        if (!prepare_buffers(context, vertex_count, triangle_count, neighbor_capacity)) {
            return Status::failure(StatusCode::resource_failure,
                "mesh CSR buffer allocation failed");
        }
        const auto index_bytes = static_cast<std::uint32_t>(
            input.triangle_indices.size() * sizeof(std::uint32_t));
        if (!context.sync_compute_ssbo(kCsrPipeline, kIndices,
                input.triangle_indices.data(), kComputeFrame, index_bytes))
        {
            return Status::failure(StatusCode::resource_failure,
                "mesh CSR index upload failed");
        }
        for (std::uint32_t pass = 0; pass < kCsrPasses; ++pass) {
            if (!dispatch_pass(context, vertex_count, triangle_count, neighbor_capacity, pass)) {
                return Status::failure(StatusCode::device_failure,
                    "mesh CSR pass " + std::to_string(pass) + " failed");
            }
        }
        context.wait_idle();

        MeshAdjacency result;
        result.vertex_count = vertex_count;
        read_uints(context, kOffsets, vertex_count + 1, result.offsets);
        const auto packed_count = result.offsets.back();
        if (packed_count > neighbor_capacity) {
            return Status::failure(StatusCode::device_failure,
                "mesh CSR packed count overflow");
        }
        read_uints(context, kPacked, packed_count, result.neighbors);
        output = std::move(result);
    }
    catch (const std::exception& error) {
        return exception_status(StatusCode::device_failure, error);
    }
    return Status::success();
}

} // namespace ORL::Rhi
