#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "component_store.hpp"
#include "mesh.hpp"
#include "../orl_exec.hpp"

namespace orlrig
{

struct RunnerStatus {
    bool ok = false;
    std::vector<std::string> errors;

    explicit operator bool() const { return ok; }
};

class LbsRunner {
public:
    explicit LbsRunner(ORL::exec::Backend backend = ORL::exec::Backend::Cpu);

    RunnerStatus capture_bind(DeformerData& deformer,
        const MeshData& mesh,
        const std::vector<Joint>& joints);
    RunnerStatus evaluate(DeformerData& deformer,
        WeightData& weights,
        const std::vector<Joint>& joints,
        bool device_only = false);
    RunnerStatus readback();

    ORL::exec::Backend backend() const { return compute_backend; }
    const ORL::exec::OrlBuffer& output_positions() const { return output; }
    std::optional<ORL::exec::DeviceBufferView> output_device();

private:
    RunnerStatus ensure_programs(const std::string& type);
    RunnerStatus bind_capture(const std::vector<Joint>& joints,
        DeformerData& deformer);
    RunnerStatus bind_deform(const std::vector<Joint>& joints,
        DeformerData& deformer,
        WeightData& weights,
        std::int64_t vertex_count);

    ORL::exec::Backend compute_backend;
    std::string compiled_type;
    std::optional<ORL::exec::OrlProgram> capture_program;
    std::optional<ORL::exec::OrlExecution> capture_execution;
    std::optional<ORL::exec::OrlProgram> deform_program;
    std::optional<ORL::exec::OrlExecution> deform_execution;
    ORL::exec::OrlBuffer joints;
    ORL::exec::OrlBuffer output;
    std::int64_t last_vertex_count = 0;
};

class AutoWeightRunner {
public:
    AutoWeightRunner(std::string algorithm,
        ORL::exec::Backend backend = ORL::exec::Backend::Cpu);

    bool set_algorithm(std::string_view name);
    const std::string& algorithm() const { return algorithm_name; }
    RunnerStatus run(const MeshData& mesh,
        const std::vector<Joint>& joints,
        const MeshCsrData* csr,
        WeightData& weights,
        double dropoff);

private:
    RunnerStatus ensure_program();

    std::string algorithm_name;
    ORL::exec::Backend compute_backend;
    std::optional<ORL::exec::OrlProgram> program;
    std::optional<ORL::exec::OrlExecution> execution;
    ORL::exec::OrlBuffer positions;
    ORL::exec::OrlBuffer packed_joints;
    ORL::exec::OrlBuffer offsets;
    ORL::exec::OrlBuffer neighbors;
    ORL::exec::OrlBuffer radii;
    ORL::exec::OrlBuffer scratch;
};

class SolverRunner {
public:
    explicit SolverRunner(ORL::exec::Backend backend = ORL::exec::Backend::Cpu);

    RunnerStatus evaluate_two_bone(std::vector<Joint>& joints,
        std::int64_t root,
        std::int64_t mid,
        std::int64_t end,
        const Controller& target,
        const Controller& pole);

private:
    RunnerStatus ensure_program();

    ORL::exec::Backend compute_backend;
    std::optional<ORL::exec::OrlProgram> program;
    std::optional<ORL::exec::OrlExecution> execution;
    ORL::exec::OrlBuffer packed_joints;
    ORL::exec::OrlBuffer target_xform;
    ORL::exec::OrlBuffer pole_xform;
};

} // namespace orlrig
