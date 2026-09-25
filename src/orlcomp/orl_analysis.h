#pragma once

#include "orl_ast.h"

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
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

enum class SemanticTypeKind : std::uint8_t {
    Unknown,
    Builtin,
    Struct,
    UniversalHandle,
    NominalHandle,
    HandleUnion,
};

struct SemanticType {
    SemanticTypeKind kind = SemanticTypeKind::Unknown;
    std::string name;
    std::string canonical_name;
    std::vector<std::string> accepted_handles;
    bool open_handle = false;

    bool is_handle() const;
    bool operator==(const SemanticType& other) const;
};

struct HandleViewEffectSummary {
    std::string parameter;
    std::string view_type;
    std::string field;
    ParameterAccess access = ParameterAccess::None;

    friend bool operator==(
        const HandleViewEffectSummary&,
        const HandleViewEffectSummary&) = default;
};

struct HandleCallSiteSummary {
    std::string callee;
    std::vector<std::optional<std::size_t>> handle_sources;
};

struct HandleTypeSummary {
    std::string name;
    std::string canonical_name;
    orlgraph::SourceLocation source;
};

struct FunctionParameterSummary {
    std::string name;
    std::string type_name;
    orlgraph::LogicalType logical_type;
    SemanticType resolved_type;
    bool is_buffer = false;
    ParameterAccess access = ParameterAccess::None;
};

struct FunctionSummary {
    std::string name;
    std::string return_type_name;
    orlgraph::LogicalType return_type;
    SemanticType resolved_return_type;
    std::vector<FunctionParameterSummary> parameters;
    std::vector<std::string> calls;
    std::map<std::string, orlgraph::ConstantValue> metadata;
    orlgraph::SourceLocation source;
    bool exported = false;
    bool pure = true;
    bool stateful = false;
    bool has_parallel_for = false;
    bool has_loop = false;
    bool has_external_call = false;
    std::vector<HandleViewEffectSummary> handle_view_effects;
    std::vector<HandleCallSiteSummary> handle_calls;
};

struct AnalysisDiagnostic {
    std::string code;
    std::string message;
    orlgraph::SourceLocation source;
    bool error = true;
};

struct AnalysisResult {
    std::vector<HandleTypeSummary> handle_types;
    std::vector<FunctionSummary> functions;
    std::vector<AnalysisDiagnostic> diagnostics;

    bool ok() const;
    const FunctionSummary* function(std::string_view name) const;
};

class SemanticAnalyzer {
public:
    AnalysisResult analyze(const Program& program,
        std::string source_name = "<source>",
        const std::unordered_map<std::string, std::string>&
            handle_type_identities = {}) const;
};

bool is_supported_orl_type(std::string_view name);
bool is_known_orl_builtin(std::string_view name);

} // namespace orlcomp
