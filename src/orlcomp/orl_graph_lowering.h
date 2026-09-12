#pragma once

#include "graph_ir.hpp"

#include <string>
#include <vector>

namespace orlcomp
{

struct GraphLoweringDiagnostic {
    std::string code;
    std::string message;
};

struct GraphLoweringOptions {
    std::string entry_function = "orl_graph_compute";
    std::string source_preamble;
    // Include directories needed by ORL modules referenced by graph nodes.
    std::vector<std::string> include_paths;
    bool emit_module_uses = true;
};

struct LoweredGraph {
    bool ok = false;
    std::string source;
    std::string entry_function;
    std::vector<GraphLoweringDiagnostic> diagnostics;
};

class OrlGraphLowerer {
public:
    LoweredGraph lower(const orlgraph::GraphModule& module,
        const orlgraph::NodeRegistry& registry,
        GraphLoweringOptions options = {}) const;
};

} // namespace orlcomp
