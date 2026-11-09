#include "orl_compute.hpp"

#include <algorithm>
#include <cstring>

namespace ORL::exec
{
namespace
{

constexpr std::size_t kIntStride = sizeof(std::int64_t);

std::uint32_t launch_count(std::int64_t count) {
    if (count <= 0) {
        return 1;
    }
    return static_cast<std::uint32_t>(count);
}

bool fill_indices(OrlBuffer& destination,
    std::span<const std::uint32_t> indices)
{
    if (!destination.resize(indices.size())) {
        return false;
    }
    auto* output = static_cast<std::int64_t*>(destination.data());
    for (std::size_t index = 0; index < indices.size(); ++index) {
        output[index] = static_cast<std::int64_t>(indices[index]);
    }
    destination.mark_modified();
    return true;
}

} // namespace

MeshCsrCompute::MeshCsrCompute(Backend backend)
    : backend_(backend)
    , indices_("int", kIntStride)
    , degrees_("int", kIntStride)
    , offsets_("int", kIntStride)
    , fill_("int", kIntStride)
    , scratch_neighbors_("int", kIntStride)
    , packed_("int", kIntStride)
    , neighbors_("int", kIntStride)
{
}

bool MeshCsrCompute::ensure_program() {
    if (program_.has_value() && program_->valid()
        && execution_.has_value() && execution_->valid()
        && execution_->backend() == backend_)
    {
        return true;
    }
    program_.reset();
    execution_.reset();
    auto compiled = OrlProgram::Compile("use compute/mesh_csr;\n", {
        .entry_function = "mesh_csr",
        .source_name = "orl_compute_mesh_csr",
    });
    if (!compiled.valid()) {
        errors_ = compiled.errors();
        return false;
    }
    auto created = OrlExecution::Create(compiled, backend_);
    if (!created.valid()) {
        errors_ = created.errors();
        return false;
    }
    program_ = std::move(compiled);
    execution_ = std::move(created);
    return true;
}

bool MeshCsrCompute::launch(std::int64_t pass, std::uint32_t element_count,
    bool readback)
{
    if (!execution_->bind_int("pass", pass)) {
        errors_ = execution_->errors();
        return false;
    }
    if (backend_ == Backend::Cuda && !readback) {
        if (!execution_->evaluate_device(element_count)) {
            errors_ = execution_->errors();
            return false;
        }
        return true;
    }
    if (!execution_->evaluate(element_count).has_value()) {
        errors_ = execution_->errors();
        return false;
    }
    return true;
}

bool MeshCsrCompute::build(std::span<const std::uint32_t> triangle_indices,
    std::int64_t vertex_count)
{
    errors_.clear();
    neighbors_.clear();
    if (vertex_count <= 0 || triangle_indices.size() % 3 != 0) {
        errors_.emplace_back(
            "Mesh CSR requires vertices and complete triangles");
        return false;
    }
    if (!ensure_program()) {
        return false;
    }

    const auto triangle_count = static_cast<std::int64_t>(
        triangle_indices.size() / 3);
    const auto neighbor_capacity = std::max<std::int64_t>(
        1, triangle_count * 6);
    if (!fill_indices(indices_, triangle_indices)
        || !degrees_.resize(static_cast<std::size_t>(vertex_count))
        || !offsets_.resize(static_cast<std::size_t>(vertex_count) + 1)
        || !fill_.resize(static_cast<std::size_t>(vertex_count))
        || !scratch_neighbors_.resize(static_cast<std::size_t>(neighbor_capacity))
        || !packed_.resize(static_cast<std::size_t>(neighbor_capacity)))
    {
        errors_.emplace_back("Failed to allocate mesh CSR buffers");
        return false;
    }
    degrees_.mark_modified();
    offsets_.mark_modified();
    fill_.mark_modified();
    scratch_neighbors_.mark_modified();
    packed_.mark_modified();

    execution_->clear_bindings();
    const bool bound = execution_->bind_buffer("indices", indices_)
        && execution_->bind_buffer("degrees", degrees_)
        && execution_->bind_buffer("offsets", offsets_)
        && execution_->bind_buffer("fill", fill_)
        && execution_->bind_buffer("neighbors", scratch_neighbors_)
        && execution_->bind_buffer("packed", packed_)
        && execution_->bind_int("vertex_count", vertex_count)
        && execution_->bind_int("triangle_count", triangle_count)
        && execution_->bind_int("neighbor_capacity", neighbor_capacity)
        && execution_->bind_int("serial_count", 1)
        && execution_->bind_int("pass", 0);
    if (!bound) {
        errors_ = execution_->errors();
        return false;
    }

    const std::uint32_t vertices = launch_count(vertex_count);
    const std::uint32_t triangles = launch_count(triangle_count);
    const std::uint32_t serial = 1;
    const bool cuda = backend_ == Backend::Cuda;
    if (!launch(0, vertices, !cuda)
        || !launch(1, triangles, !cuda)
        || !launch(2, serial, !cuda)
        || !launch(3, triangles, !cuda)
        || !launch(4, vertices, !cuda)
        || !launch(5, serial, true))
    {
        return false;
    }

    std::int64_t packed_count = 0;
    if (!offsets_.read(static_cast<std::size_t>(vertex_count), &packed_count)
        || packed_count < 0
        || static_cast<std::size_t>(packed_count) > packed_.count())
    {
        errors_.emplace_back("Mesh CSR packed count is invalid");
        return false;
    }
    if (!neighbors_.resize(static_cast<std::size_t>(packed_count))) {
        errors_.emplace_back("Failed to publish mesh CSR neighbors");
        return false;
    }
    if (packed_count > 0) {
        std::memcpy(neighbors_.data(), packed_.data(),
            static_cast<std::size_t>(packed_count) * kIntStride);
    }
    neighbors_.mark_modified();
    return true;
}

std::optional<DeviceBufferView> MeshCsrCompute::offsets_device() {
    if (!execution_.has_value() || backend_ != Backend::Cuda) {
        return std::nullopt;
    }
    return execution_->device_buffer_view("offsets");
}

std::optional<DeviceBufferView> MeshCsrCompute::neighbors_device() {
    if (!execution_.has_value() || backend_ != Backend::Cuda) {
        return std::nullopt;
    }
    return execution_->device_buffer_view("packed");
}

} // namespace ORL::exec
