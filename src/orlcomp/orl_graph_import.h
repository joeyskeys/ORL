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
    // Empty means import every exported function; otherwise only these
    // exported entry names are exposed as graph nodes.
    std::vector<std::string> exported_functions;
    // Stdlib use path such as "solver/ik_two_bone". When set, the compiler
    // stamps each exported function as orlrig.<category>.<name> before the
    // .oro document is written.
    std::string use_path;
};

struct OroCompileResult {
    std::string text;
    std::vector<AnalysisDiagnostic> diagnostics;

    bool ok() const;
};

NodeImportResult import_node_definitions(const AnalysisResult& analysis,
    std::string module_name = "orl");

// Parses and analyzes external ORL source, then imports only functions
// prefixed with `export` as graph node definitions. The returned registry can
// be merged into any application-owned NodeRegistry without rebuilding the
// application.
NodeImportResult import_node_definitions(std::string_view source,
    NodeImportOptions options = {});

NodeImportResult import_node_definitions_file(const std::string& path,
    NodeImportOptions options = {});

// Reads a compiled .oro document and returns the node definitions stored in
// it. Syntax failures are already rejected by the compiler that wrote the
// document, so this path does not analyze ORL source.
NodeImportResult import_node_definitions_from_oro(std::string_view oro_text);

NodeImportResult import_node_definitions_from_oro_file(const std::string& path);

// Compiles ORL source to an .oro document. Registration reads that document;
// it does not consume the analysis result directly.
OroCompileResult compile_node_definitions_to_oro(std::string_view source,
    NodeImportOptions options = {});

OroCompileResult compile_node_definitions_to_oro_file(const std::string& path,
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

NodeRegistrationResult register_orl_node_definitions_from_oro(
    orlgraph::NodeRegistry& registry, std::string_view oro_text);

NodeRegistrationResult register_orl_node_definitions_from_oro_file(
    orlgraph::NodeRegistry& registry, const std::string& path);

} // namespace orlcomp
