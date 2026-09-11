#pragma once

#include "orl_analysis.h"

#include "graph_ir.hpp"

#include <string>
#include <vector>

namespace orlcomp
{

struct NodeImportResult {
    orlgraph::NodeRegistry registry;
    std::vector<AnalysisDiagnostic> diagnostics;

    bool ok() const;
};

NodeImportResult import_node_definitions(const AnalysisResult& analysis,
    std::string module_name = "orl");

} // namespace orlcomp
