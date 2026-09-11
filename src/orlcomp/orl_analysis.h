#pragma once

#include "orl_ast.h"

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "graph_ir.hpp"

namespace orlcomp
{

enum class ParameterAccess : std::uint8_t {
    None = 0,
    Read = 1,
    Write = 2,
    ReadWrite = 3,
};

struct FunctionParameterSummary {
    std::string name;
    std::string type_name;
    orlgraph::LogicalType logical_type;
    bool is_buffer = false;
    ParameterAccess access = ParameterAccess::None;
};

struct FunctionSummary {
    std::string name;
    std::string return_type_name;
    orlgraph::LogicalType return_type;
    std::vector<FunctionParameterSummary> parameters;
    std::vector<std::string> calls;
    orlgraph::SourceLocation source;
    bool pure = true;
    bool stateful = false;
    bool has_parallel_for = false;
    bool has_loop = false;
    bool has_external_call = false;
};

struct AnalysisDiagnostic {
    std::string code;
    std::string message;
    orlgraph::SourceLocation source;
    bool error = true;
};

struct AnalysisResult {
    std::vector<FunctionSummary> functions;
    std::vector<AnalysisDiagnostic> diagnostics;

    bool ok() const;
    const FunctionSummary* function(std::string_view name) const;
};

class SemanticAnalyzer {
public:
    AnalysisResult analyze(const Program& program,
        std::string source_name = "<source>") const;
};

bool is_supported_orl_type(std::string_view name);
bool is_known_orl_builtin(std::string_view name);

} // namespace orlcomp
