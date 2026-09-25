#pragma once

#include "graph_ir.hpp"

#include <cstddef>
#include <functional>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace orlcomp
{

struct GraphLoweringDiagnostic {
    std::string code;
    std::string message;
};

using RuntimeOutputExpressionResolver = std::function<
    std::optional<std::string>(
        const orlgraph::NodeInstance&,
        const orlgraph::NodeDefinition&,
        const orlgraph::Port&)>;

using RuntimeHandleIndexExpressionResolver = std::function<
    std::optional<std::string>(
        const orlgraph::NodeInstance&,
        const orlgraph::NodeDefinition&,
        const orlgraph::Port&,
        std::string_view)>;

struct GraphLoweringOptions {
    std::string entry_function = "orl_graph_compute";
    std::string source_preamble;
    // Include directories needed by ORL modules referenced by graph nodes.
    std::vector<std::string> include_paths;
    bool emit_module_uses = true;
    // Supplies ORL expressions for runtime-node outputs that are backed by
    // application data rather than an ORL function. The resolver is also
    // used as the source expression for implicit output adapters.
    RuntimeOutputExpressionResolver runtime_output_expression;
    // Resolves a stable scene handle to the current packed-array index when
    // a writable adapter's logical selector is a handle.
    RuntimeHandleIndexExpressionResolver runtime_handle_index_expression;
    // Revision of the scene snapshot used by runtime output resolvers.
    std::size_t scene_revision = 0;
};

// Describes how one logical graph output is represented by the lowered ORL
// entry point. Buffer outputs may alias a bound graph-input parameter; scalar
// outputs are currently returned through the legacy int entry-point result.
struct LoweredGraphOutput {
    orlgraph::StableId id;
    std::string name;
    orlgraph::LogicalType type;
    orlgraph::Domain domain = orlgraph::Domain::constant();
    orlgraph::Shape shape = orlgraph::Shape::scalar();
    std::string binding;
    std::string semantic;
    std::string coordinate_space;
    std::string source_parameter;
    bool returned = false;
};

struct LoweredGraph {
    bool ok = false;
    std::string source;
    std::string entry_function;
    std::vector<GraphLoweringDiagnostic> diagnostics;
    std::vector<LoweredGraphOutput> outputs;
    std::map<std::string, std::string> handle_type_identities;
    std::size_t scene_revision = 0;
};

class OrlGraphLowerer {
public:
    LoweredGraph lower(const orlgraph::GraphModule& module,
        const orlgraph::NodeRegistry& registry,
        GraphLoweringOptions options = {}) const;
};

} // namespace orlcomp
