#pragma once

#include "orl_ast.h"

#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace orlcomp
{

inline constexpr std::string_view kSolverContextParameterName =
    "__orl_solver_context";
inline constexpr std::string_view kSolverContextTypeName = "SolverContext";

enum class OrlRuntimeParameterKind {
    Buffer,
    Int64,
    Float64,
    Unsupported,
};

struct OrlRuntimeParameter {
    std::string name;
    std::string type_name;
    OrlRuntimeParameterKind kind = OrlRuntimeParameterKind::Unsupported;
};

struct OrlRuntimeFunctionSignature {
    std::string name;
    std::string return_type;
    std::vector<OrlRuntimeParameter> parameters;
    bool uses_solver_context = false;
};

inline OrlRuntimeParameterKind RuntimeParameterKindFor(const Parameter& parameter) {
    if (parameter.is_buffer) {
        return OrlRuntimeParameterKind::Buffer;
    }
    if (parameter.type_name == "int") {
        return OrlRuntimeParameterKind::Int64;
    }
    if (parameter.type_name == "float") {
        return OrlRuntimeParameterKind::Float64;
    }
    return OrlRuntimeParameterKind::Unsupported;
}

inline std::optional<OrlRuntimeFunctionSignature> DescribeRuntimeFunction(
    const Program& program, std::string_view name)
{
    for (const auto& item : program.items) {
        const auto* function = dynamic_cast<const FunctionDefinitionStatement*>(item.get());
        if (function == nullptr || function->name != name) {
            continue;
        }

        OrlRuntimeFunctionSignature signature;
        signature.name = function->name;
        signature.return_type = function->return_type;
        signature.uses_solver_context = program.uses_solver_context;
        signature.parameters.reserve(function->parameters.size());
        for (const Parameter& parameter : function->parameters) {
            signature.parameters.push_back({
                parameter.name,
                parameter.type_name,
                RuntimeParameterKindFor(parameter),
            });
        }
        if (signature.uses_solver_context) {
            signature.parameters.push_back({
                std::string{kSolverContextParameterName},
                std::string{kSolverContextTypeName},
                OrlRuntimeParameterKind::Buffer,
            });
        }
        return signature;
    }
    return std::nullopt;
}

} // namespace orlcomp
