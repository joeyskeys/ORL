#pragma once

#include "graph_ir.hpp"
#include "graph_validation.hpp"

#include <string>
#include <vector>

namespace orlrig
{

struct RigGraphResourceIds {
    orlgraph::StableId bind_positions;
    orlgraph::StableId posed_positions;
    orlgraph::StableId joints;
    orlgraph::StableId inverse_binds;
    orlgraph::StableId weights;
};

struct RigGraph {
    orlgraph::GraphModule module;
    orlgraph::NodeRegistry registry;
    RigGraphResourceIds resources;

    orlgraph::ValidationResult validate() const;
};

// Describes renderer-independent resources used by the standard LBS graph.
RigGraphResourceIds add_lbs_resources(orlgraph::GraphModule& module);

// Registers runtime node definitions for the standalone rigging pipeline.
bool register_rig_node_definitions(orlgraph::NodeRegistry& registry,
    std::string* error = nullptr);

// Builds a dependency graph for bind capture followed by LBS deformation.
// Execution is still supplied by the standalone runners; the graph owns the
// ordering and resource contracts.
RigGraph make_lbs_graph();

} // namespace orlrig
