#pragma once

#include "graph_validation.hpp"

#include <vector>

namespace orlgraph
{

struct OptimizationOptions {
    bool specialize_constants = true;
    bool simplify_identity = true;
    bool eliminate_dead_nodes = true;
    bool eliminate_common_subgraphs = true;
};

struct OptimizationResult {
    bool ok = false;
    std::vector<Diagnostic> diagnostics;
    std::vector<StableId> removed_nodes;
    std::vector<StableId> merged_nodes;
    ScheduleResult schedule;
};

class GraphOptimizer {
public:
    explicit GraphOptimizer(OptimizationOptions options = {});

    OptimizationResult optimize(GraphModule& module,
        const NodeRegistry& registry) const;

private:
    OptimizationOptions options_;
};

} // namespace orlgraph
