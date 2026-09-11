#pragma once

#include "graph_ir.hpp"

#include <string>
#include <vector>

namespace orlgraph
{

struct ScheduleResult {
    bool ok = false;
    std::vector<StableId> order;
    std::vector<std::string> errors;
};

ScheduleResult topological_schedule(const GraphModule& module);

} // namespace orlgraph
