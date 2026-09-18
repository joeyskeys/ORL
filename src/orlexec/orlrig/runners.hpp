#pragma once

#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <glm/mat4x4.hpp>

#include "component_store.hpp"
#include "evaluation.hpp"
#include "hierarchy.hpp"
#include "locator.hpp"
#include "mesh.hpp"
#include "../orl_exec.hpp"
#include "runner_status.hpp"
#include "two_stage_runner.hpp"

namespace orlrig
{

class LbsRunner {
public:
    explicit LbsRunner(ORL::exec::Backend backend = ORL::exec::Backend::Cpu);

    RunnerStatus capture_bind(DeformerData& deformer,
        const MeshData& mesh,
        const std::vector<Joint>& joints);
    RunnerStatus capture_bind(DeformerData& deformer,
        const MeshData& mesh,
        ORL::exec::OrlBuffer& joints);
    RunnerStatus evaluate(DeformerData& deformer,
        WeightData& weights,
        const std::vector<Joint>& joints,
        bool device_only = false);
    RunnerStatus evaluate(DeformerData& deformer,
        WeightData& weights,
        ORL::exec::OrlBuffer& joints,
        bool device_only = false);
    RunnerStatus readback();

    ORL::exec::Backend backend() const { return compute_backend; }
    const ORL::exec::OrlBuffer& output_positions() const { return output; }
    std::optional<ORL::exec::DeviceBufferView> output_device();

private:
    RunnerStatus ensure_programs(const std::string& type);
    RunnerStatus bind_capture(ORL::exec::OrlBuffer& joints,
        DeformerData& deformer);
    RunnerStatus bind_deform(ORL::exec::OrlBuffer& joints,
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
        const Locator& target,
        const Locator& pole);
    RunnerStatus evaluate_two_bone(ORL::exec::OrlBuffer& joints,
        std::int64_t root,
        std::int64_t mid,
        std::int64_t end,
        const Locator& target,
        const Locator& pole);
    RunnerStatus evaluate_two_bone(std::vector<Joint>& joints,
        std::int64_t root,
        std::int64_t mid,
        std::int64_t end,
        const Controller& target,
        const Controller& pole);
    RunnerStatus set_hierarchy_plan(const HierarchyPlan& plan);
    const std::optional<HierarchyPlan>& hierarchy_plan() const {
        return compiled_hierarchy_plan;
    }
    RunnerStatus evaluate_two_bone(ComponentStore& components,
        ComponentId root,
        ComponentId mid,
        ComponentId end,
        const Locator& target,
        const Locator& pole);

private:
    RunnerStatus ensure_program();

    ORL::exec::Backend compute_backend;
    std::optional<ORL::exec::OrlProgram> program;
    std::optional<ORL::exec::OrlExecution> execution;
    ORL::exec::OrlBuffer packed_joints;
    ORL::exec::OrlBuffer target_xform;
    ORL::exec::OrlBuffer pole_xform;
    ORL::exec::OrlBuffer hierarchy_data;
    HierarchyContext hierarchy_context;
    std::optional<HierarchyPlan> compiled_hierarchy_plan;
};

using SolverRegionCallback = std::function<RunnerStatus(const SolverRegion&)>;

RunnerStatus dispatch_cpu_levels(
    const EvaluationPlan& plan,
    const DynamicDispatchPlan& dispatch,
    const SolverRegionCallback& callback);

RunnerStatus compute_world_matrices_parallel(
    const HierarchyPlan& hierarchy,
    const ComponentStore& components,
    std::vector<glm::mat4>* world_matrices);

} // namespace orlrig
