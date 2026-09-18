#pragma once

#include "component_store.hpp"
#include "hierarchy.hpp"
#include "graph_ir.hpp"
#include "graph_schedule.hpp"

#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace orlrig
{

struct SolverRegion {
    orlgraph::StableId node_id;
    orlgraph::StableId definition_id;
    bool declared = false;
    bool global = true;
    bool stateful = false;
    bool supports_sparse_dispatch = false;
    orlgraph::PartialPropagation propagation =
        orlgraph::PartialPropagation::Full;

    std::vector<ComponentId> read_joints;
    std::vector<ComponentId> write_joints;
    std::vector<ComponentId> affected_joints;
    std::vector<ComponentId> read_controllers;
    std::vector<ComponentId> read_locators;
    std::vector<orlgraph::StableId> read_resources;
    std::vector<orlgraph::StableId> write_resources;

    // Backend execution artifacts. These are per-region packed indices, not
    // a retained stable-ID-to-dense hierarchy map.
    std::vector<std::int64_t> read_packed_indices;
    std::vector<std::int64_t> write_packed_indices;
    std::vector<std::int64_t> affected_packed_indices;
    std::vector<std::size_t> dependencies;
};

struct DirtyInputs {
    bool full_evaluation = false;
    std::vector<ComponentId> joints;
    std::vector<ComponentId> controllers;
    std::vector<ComponentId> locators;

    bool empty() const {
        return !full_evaluation && joints.empty()
            && controllers.empty() && locators.empty();
    }
};

struct DirtyDispatchRange {
    std::uint32_t first = 0;
    std::uint32_t count = 0;
};

struct DynamicDispatchPlan {
    bool full_evaluation = false;
    std::vector<std::vector<std::size_t>> levels;
    std::vector<std::uint32_t> region_indices;
    std::vector<DirtyDispatchRange> cuda_ranges;
};

struct SolverDispatchRuntimeData {
    SolverDispatchContext context;
    std::vector<std::int64_t> data;
};

struct EvaluationPlan {
    std::uint64_t topology_revision = 0;
    std::uint64_t dependency_revision = 0;
    bool full_evaluation_required = false;
    bool stateful_evaluation_required = false;

    std::vector<SolverRegion> regions;
    std::vector<orlgraph::StableId> order;
    std::vector<std::vector<orlgraph::StableId>> batches;

    std::map<std::uint64_t, std::vector<orlgraph::StableId>>
        controller_regions;
    std::map<std::uint64_t, std::vector<orlgraph::StableId>>
        locator_regions;
    std::map<std::uint64_t, std::vector<orlgraph::StableId>>
        joint_readers;
    std::map<std::uint64_t, std::vector<orlgraph::StableId>>
        joint_writers;

    const SolverRegion* region(
        const orlgraph::StableId& node_id) const;
    const std::vector<orlgraph::StableId>& regions_for_controller(
        ComponentId id) const;
    const std::vector<orlgraph::StableId>& regions_for_locator(
        ComponentId id) const;
};

struct EvaluationPlanCompileResult {
    std::optional<EvaluationPlan> plan;
    std::vector<std::string> errors;

    explicit operator bool() const {
        return plan.has_value() && errors.empty();
    }
};

EvaluationPlanCompileResult compile_evaluation_plan(
    const orlgraph::GraphModule& graph,
    const orlgraph::NodeRegistry& registry,
    const ComponentStore& components,
    const HierarchyPlan& hierarchy);

DynamicDispatchPlan build_dynamic_dispatch_plan(
    const EvaluationPlan& plan, const DirtyInputs& dirty,
    double full_evaluation_threshold = 0.6);
DynamicDispatchPlan build_cuda_dispatch_plan(
    const EvaluationPlan& plan, const DirtyInputs& dirty,
    double full_evaluation_threshold = 0.6);
SolverDispatchRuntimeData pack_dynamic_dispatch_plan(
    const DynamicDispatchPlan& dispatch);

} // namespace orlrig
