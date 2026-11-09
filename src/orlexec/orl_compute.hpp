#pragma once

#include "orl_exec.hpp"

#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace ORL::exec
{

// Host-triggered compute entry. It is not a solver or deformer stage.
// The resulting buffers can be copied with transfer_host_buffer, or, on
// CUDA, bound into a later execution through the device views below.
// Those views stay valid while this object is alive.
class MeshCsrCompute {
public:
    explicit MeshCsrCompute(Backend backend = Backend::Cpu);

    bool build(std::span<const std::uint32_t> triangle_indices,
        std::int64_t vertex_count);

    const OrlBuffer& offsets() const { return offsets_; }
    const OrlBuffer& neighbors() const { return neighbors_; }
    std::optional<DeviceBufferView> offsets_device();
    std::optional<DeviceBufferView> neighbors_device();
    const std::vector<std::string>& errors() const { return errors_; }

private:
    bool ensure_program();
    bool launch(std::int64_t pass, std::uint32_t element_count, bool readback);

    Backend backend_;
    std::optional<OrlProgram> program_;
    std::optional<OrlExecution> execution_;
    OrlBuffer indices_;
    OrlBuffer degrees_;
    OrlBuffer offsets_;
    OrlBuffer fill_;
    OrlBuffer scratch_neighbors_;
    OrlBuffer packed_;
    OrlBuffer neighbors_;
    std::vector<std::string> errors_;
};

} // namespace ORL::exec
