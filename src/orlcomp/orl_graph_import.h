#pragma once

#include "orl_analysis.h"

#include "graph_ir.hpp"

#include <cstddef>
#include <string>
#include <string_view>
#include <vector>

namespace orlcomp
{

struct NodeImportResult {
    orlgraph::NodeRegistry registry;
    std::vector<AnalysisDiagnostic> diagnostics;

    bool ok() const;
};

struct NodeImportOptions {
    std::string module_name = "orl";
    std::string source_name = "<source>";
    std::vector<std::string> include_paths;
    // Empty means import every analyzed function; otherwise only these
    // public entry names are exposed as graph nodes.
    std::vector<std::string> exported_functions;
};

NodeImportResult import_node_definitions(const AnalysisResult& analysis,
    std::string module_name = "orl");

// Parses and analyzes external ORL source, then imports every function as a
// graph node definition. The returned registry can be merged into any
// application-owned NodeRegistry without rebuilding the application.
NodeImportResult import_node_definitions(std::string_view source,
    NodeImportOptions options = {});

NodeImportResult import_node_definitions_file(const std::string& path,
    NodeImportOptions options = {});

struct NodeRegistrationResult {
    std::size_t registered_count = 0;
    std::vector<AnalysisDiagnostic> diagnostics;

    bool ok() const;
};

// Preflights and registers all definitions from an imported external module.
// Duplicate IDs are reported before any registration occurs.
NodeRegistrationResult register_orl_node_definitions(
    orlgraph::NodeRegistry& registry, NodeImportResult imported);

NodeRegistrationResult register_orl_node_definitions(
    orlgraph::NodeRegistry& registry, std::string_view source,
    NodeImportOptions options = {});

NodeRegistrationResult register_orl_node_definitions_file(
    orlgraph::NodeRegistry& registry, const std::string& path,
    NodeImportOptions options = {});

} // namespace orlcomp
