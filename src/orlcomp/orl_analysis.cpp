#include "orl_analysis.h"

#include <algorithm>
#include <array>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace orlcomp
{

using orlgraph::LogicalType;

namespace
{

constexpr std::array<std::string_view, 18> kSupportedTypes = {
    "bool", "int", "int64", "float", "double", "float64", "string",
    "vector", "normal", "point", "vec3", "dvec3", "vec4", "dvec4",
    "quat", "matrix", "mat4", "dmat4",
};

bool has_type(std::string_view name) {
    return std::find(kSupportedTypes.begin(), kSupportedTypes.end(), name)
        != kSupportedTypes.end();
}

bool has_flag(ParameterAccess value, ParameterAccess flag) {
    return (static_cast<std::uint8_t>(value) & static_cast<std::uint8_t>(flag)) != 0;
}

ParameterAccess add_access(ParameterAccess value, ParameterAccess access) {
    return static_cast<ParameterAccess>(
        static_cast<std::uint8_t>(value) | static_cast<std::uint8_t>(access));
}

std::string decode_string_literal(std::string_view raw) {
    if (raw.size() < 2 || raw.front() != '"' || raw.back() != '"') {
        return std::string{raw};
    }
    std::string result;
    result.reserve(raw.size() - 2);
    bool escaped = false;
    for (std::size_t i = 1; i + 1 < raw.size(); ++i) {
        const char c = raw[i];
        if (escaped) {
            switch (c) {
            case 'n': result.push_back('\n'); break;
            case 'r': result.push_back('\r'); break;
            case 't': result.push_back('\t'); break;
            default: result.push_back(c); break;
            }
            escaped = false;
        } else if (c == '\\') {
            escaped = true;
        } else {
            result.push_back(c);
        }
    }
    return result;
}

const IdentifierExpression* identifier(const Expression* expression) {
    return dynamic_cast<const IdentifierExpression*>(expression);
}

const IdentifierExpression* base_identifier(const Expression* expression) {
    if (const auto* direct = identifier(expression)) {
        return direct;
    }
    if (const auto* component = dynamic_cast<const ComponentExpression*>(expression)) {
        return base_identifier(component->base.get());
    }
    if (const auto* index = dynamic_cast<const IndexExpression*>(expression)) {
        return base_identifier(index->base.get());
    }
    return nullptr;
}

class FunctionAnalyzer {
public:
    FunctionAnalyzer(const std::unordered_set<std::string>& functions,
        const std::unordered_set<std::string>& structs,
        const std::string& source_name,
        const FunctionDefinitionStatement& definition,
        std::vector<AnalysisDiagnostic>* diagnostics)
        : functions_(functions)
        , structs_(structs)
        , source_name_(source_name)
        , definition_(definition)
        , diagnostics_(diagnostics)
    {
        summary_.name = definition.name;
        summary_.return_type_name = definition.return_type;
        summary_.return_type = LogicalType::from_orl_name(definition.return_type);
        summary_.source.file = source_name_;
        summary_.exported = definition.exported;
        collect_metadata();
        for (const auto& parameter : definition.parameters) {
            if (parameter.name == "solver_context"
                || parameter.name == "hierarchy_context"
                || parameter.name == "hierarchy_data")
            {
                add_error("ORL_ANALYSIS_RESERVED_NAME",
                    "The name '" + parameter.name
                        + "' is reserved for an implicit runtime global");
            }
            FunctionParameterSummary value;
            value.name = parameter.name;
            value.type_name = parameter.type_name;
            value.logical_type = LogicalType::from_orl_name(parameter.type_name);
            value.is_buffer = parameter.is_buffer;
            summary_.parameters.push_back(std::move(value));
            parameter_indices_.emplace(parameter.name, summary_.parameters.size() - 1);
        }
    }

    FunctionSummary run() {
        validate_type(definition_.return_type, "return type");
        for (const auto& parameter : definition_.parameters) {
            validate_type(parameter.type_name, "parameter type");
        }
        visit_block(*definition_.body);
        summary_.pure = !summary_.has_external_call
            && !summary_.stateful
            && !summary_.has_parallel_for;
        for (const auto& parameter : summary_.parameters) {
            if (has_flag(parameter.access, ParameterAccess::Write)) {
                summary_.pure = false;
            }
        }
        return summary_;
    }

private:
    void collect_metadata() {
        if (!definition_.exported && !definition_.metadata.empty()) {
            add_error("ORL_ANALYSIS_METADATA_EXPORT",
                "Function metadata is only allowed on exported functions");
        }
        for (const auto& entry : definition_.metadata) {
            if (summary_.metadata.contains(entry.name)) {
                add_error("ORL_ANALYSIS_METADATA_DUPLICATE",
                    "Duplicate function metadata: " + entry.name);
                continue;
            }
            const auto type = LogicalType::from_orl_name(entry.type_name);
            if (type.kind != orlgraph::LogicalTypeKind::Int64
                && type.kind != orlgraph::LogicalTypeKind::Float64
                && type.kind != orlgraph::LogicalTypeKind::String)
            {
                add_error("ORL_ANALYSIS_METADATA_TYPE",
                    "Function metadata must use int, float, or string: "
                    + entry.type_name);
                continue;
            }

            orlgraph::ConstantValue value;
            value.type = type;
            if (type.kind == orlgraph::LogicalTypeKind::String) {
                if (entry.value_kind != LiteralKind::String) {
                    add_error("ORL_ANALYSIS_METADATA_VALUE",
                        "String metadata requires a string literal: "
                        + entry.name);
                    continue;
                }
                value.value = decode_string_literal(entry.raw_value);
            } else if (type.kind == orlgraph::LogicalTypeKind::Int64) {
                if (entry.value_kind != LiteralKind::Int) {
                    add_error("ORL_ANALYSIS_METADATA_VALUE",
                        "Integer metadata requires an integer literal: "
                        + entry.name);
                    continue;
                }
                value.value = entry.int_value;
            } else {
                if (entry.value_kind == LiteralKind::Float) {
                    value.value = entry.float_value;
                } else if (entry.value_kind == LiteralKind::Int) {
                    value.value = static_cast<double>(entry.int_value);
                } else {
                    add_error("ORL_ANALYSIS_METADATA_VALUE",
                        "Float metadata requires a numeric literal: "
                        + entry.name);
                    continue;
                }
            }
            summary_.metadata.emplace(entry.name, std::move(value));
        }
    }

    void add_error(std::string code, std::string message) {
        diagnostics_->push_back(AnalysisDiagnostic{
            std::move(code),
            std::move(message),
            summary_.source,
            true,
        });
    }

    void validate_type(std::string_view type_name, const char* context) {
        if (!has_type(type_name) && !structs_.contains(std::string{type_name})) {
            add_error("ORL_ANALYSIS_UNSUPPORTED_TYPE",
                std::string{"Unsupported "} + context + ": " + std::string{type_name});
        }
    }

    void access(std::string_view name, ParameterAccess value) {
        const auto found = parameter_indices_.find(std::string{name});
        if (found == parameter_indices_.end()) {
            return;
        }
        auto& parameter = summary_.parameters[found->second];
        parameter.access = add_access(parameter.access, value);
    }

    void visit_block(const BlockStatement& block) {
        for (const auto& statement : block.statements) {
            if (statement != nullptr) {
                visit_statement(*statement);
            }
        }
    }

    void visit_statement(const Statement& statement) {
        if (const auto* block = dynamic_cast<const BlockStatement*>(&statement)) {
            visit_block(*block);
        } else if (const auto* expression = dynamic_cast<const ExpressionStatement*>(&statement)) {
            visit_expression(*expression->expression);
        } else if (const auto* declaration = dynamic_cast<const DeclarationStatement*>(&statement)) {
            if (declaration->variable_name == "solver_context"
                || declaration->variable_name == "hierarchy_context"
                || declaration->variable_name == "hierarchy_data")
            {
                add_error("ORL_ANALYSIS_RESERVED_NAME",
                    "The name '" + declaration->variable_name
                        + "' is reserved for an implicit runtime global");
            }
            validate_type(declaration->type_name, "declaration type");
            if (declaration->initializer != nullptr) {
                visit_expression(*declaration->initializer);
            }
            for (const auto& argument : declaration->constructor_arguments) {
                visit_expression(*argument);
            }
        } else if (const auto* return_statement = dynamic_cast<const ReturnStatement*>(&statement)) {
            if (return_statement->value != nullptr) {
                visit_expression(*return_statement->value);
            }
        } else if (const auto* conditional = dynamic_cast<const IfStatement*>(&statement)) {
            visit_expression(*conditional->condition);
            visit_statement(*conditional->then_branch);
            if (conditional->else_branch != nullptr) {
                visit_statement(*conditional->else_branch);
            }
        } else if (const auto* loop = dynamic_cast<const WhileStatement*>(&statement)) {
            summary_.has_loop = true;
            visit_expression(*loop->condition);
            visit_statement(*loop->body);
        } else if (const auto* loop = dynamic_cast<const DoWhileStatement*>(&statement)) {
            summary_.has_loop = true;
            visit_statement(*loop->body);
            visit_expression(*loop->condition);
        } else if (const auto* loop = dynamic_cast<const ForStatement*>(&statement)) {
            summary_.has_loop = true;
            if (loop->init != nullptr) {
                visit_expression(*loop->init);
            }
            if (loop->condition != nullptr) {
                visit_expression(*loop->condition);
            }
            if (loop->increment != nullptr) {
                visit_expression(*loop->increment);
            }
            visit_statement(*loop->body);
        } else if (const auto* loop = dynamic_cast<const ParallelForStatement*>(&statement)) {
            summary_.has_parallel_for = true;
            summary_.has_loop = true;
            visit_expression(*loop->bound);
            visit_block(*loop->body);
        }
    }

    void visit_expression(const Expression& expression) {
        if (const auto* value = dynamic_cast<const IdentifierExpression*>(&expression)) {
            access(value->name, ParameterAccess::Read);
        } else if (const auto* unary = dynamic_cast<const UnaryExpression*>(&expression)) {
            visit_expression(*unary->operand);
        } else if (const auto* binary = dynamic_cast<const BinaryExpression*>(&expression)) {
            visit_expression(*binary->left);
            visit_expression(*binary->right);
        } else if (const auto* assignment = dynamic_cast<const AssignmentExpression*>(&expression)) {
            if (assignment->target_name == "solver_context"
                || assignment->target_name == "hierarchy_context"
                || assignment->target_name == "hierarchy_data")
            {
                add_error("ORL_ANALYSIS_CONTEXT_WRITE",
                    "The implicit runtime context global is read-only");
            }
            access(assignment->target_name, ParameterAccess::Write);
            visit_expression(*assignment->value);
        } else if (const auto* call = dynamic_cast<const CallExpression*>(&expression)) {
            const auto* callee = identifier(call->callee.get());
            if (callee == nullptr) {
                add_error("ORL_ANALYSIS_INDIRECT_CALL",
                    "Only direct function calls can be imported as graph nodes");
            } else {
                const std::string& name = callee->name;
                if (!functions_.contains(name) && !is_known_orl_builtin(name)) {
                    add_error("ORL_ANALYSIS_UNKNOWN_CALL",
                        "Unresolved function call: " + name);
                    summary_.has_external_call = true;
                }
                if (name == "print" || name == "global_id") {
                    summary_.has_external_call = true;
                }
                if (!std::count(summary_.calls.begin(), summary_.calls.end(), name)) {
                    summary_.calls.push_back(name);
                }
            }
            for (const auto& argument : call->arguments) {
                visit_expression(*argument);
            }
        } else if (const auto* index = dynamic_cast<const IndexExpression*>(&expression)) {
            visit_expression(*index->base);
            visit_expression(*index->index);
        } else if (const auto* assignment = dynamic_cast<const IndexAssignmentExpression*>(&expression)) {
            if (const auto* base = base_identifier(assignment->target.get())) {
                if (base->name == "hierarchy_data") {
                    add_error("ORL_ANALYSIS_CONTEXT_WRITE",
                        "The implicit hierarchy_data global is read-only");
                }
                access(base->name, ParameterAccess::Write);
            }
            if (assignment->target != nullptr && assignment->target->index != nullptr) {
                visit_expression(*assignment->target->index);
            }
            visit_expression(*assignment->value);
        } else if (const auto* component = dynamic_cast<const ComponentExpression*>(&expression)) {
            visit_expression(*component->base);
        } else if (const auto* assignment = dynamic_cast<const MemberAssignmentExpression*>(&expression)) {
            if (assignment->target != nullptr) {
                if (const auto* base = base_identifier(assignment->target->base.get())) {
                    if (base->name == "solver_context"
                        || base->name == "hierarchy_context"
                        || base->name == "hierarchy_data")
                    {
                        add_error("ORL_ANALYSIS_CONTEXT_WRITE",
                            "The implicit runtime context global is read-only");
                    }
                    access(base->name, ParameterAccess::Write);
                }
            }
            visit_expression(*assignment->value);
        }
    }

    const std::unordered_set<std::string>& functions_;
    const std::unordered_set<std::string>& structs_;
    const std::string& source_name_;
    const FunctionDefinitionStatement& definition_;
    std::vector<AnalysisDiagnostic>* diagnostics_;
    FunctionSummary summary_;
    std::unordered_map<std::string, std::size_t> parameter_indices_;
};

} // namespace

bool is_supported_orl_type(std::string_view name) {
    return has_type(name);
}

bool is_known_orl_builtin(std::string_view name) {
    static constexpr std::array<std::string_view, 18> builtins = {
        "global_id", "print", "dot", "cross", "length", "normalize",
        "clamp", "lerp", "mat_identity", "mat_transpose", "mat_inverse",
        "mat_mul", "quat_mul", "quat_conjugate", "quat_normalize",
        "quat_rotate", "min", "max",
    };
    return std::find(builtins.begin(), builtins.end(), name) != builtins.end();
}

bool AnalysisResult::ok() const {
    return std::none_of(diagnostics.begin(), diagnostics.end(),
        [](const AnalysisDiagnostic& diagnostic) { return diagnostic.error; });
}

const FunctionSummary* AnalysisResult::function(std::string_view name) const {
    for (const auto& function_value : functions) {
        if (function_value.name == name) {
            return &function_value;
        }
    }
    return nullptr;
}

AnalysisResult SemanticAnalyzer::analyze(const Program& program,
    std::string source_name) const
{
    AnalysisResult result;
    std::unordered_set<std::string> functions;
    std::unordered_set<std::string> structs;

    for (const auto& item : program.items) {
        if (const auto* structure = dynamic_cast<const StructDefinitionStatement*>(item.get())) {
            if (!structs.insert(structure->name).second) {
                result.diagnostics.push_back({
                    "ORL_ANALYSIS_DUPLICATE_STRUCT",
                    "Duplicate struct definition: " + structure->name,
                    {source_name, 0, 0, 0, 0},
                    true,
                });
            }
        }
    }
    for (const auto& item : program.items) {
        if (const auto* function = dynamic_cast<const FunctionDefinitionStatement*>(item.get())) {
            if (!functions.insert(function->name).second) {
                result.diagnostics.push_back({
                    "ORL_ANALYSIS_DUPLICATE_FUNCTION",
                    "Duplicate function definition: " + function->name,
                    {source_name, 0, 0, 0, 0},
                    true,
                });
            }
        }
    }

    for (const auto& item : program.items) {
        const auto* function = dynamic_cast<const FunctionDefinitionStatement*>(item.get());
        if (function == nullptr) {
            continue;
        }
        FunctionAnalyzer analyzer(functions, structs, source_name, *function,
            &result.diagnostics);
        result.functions.push_back(analyzer.run());
    }

    // Propagate conservative call effects. A function is only pure if every
    // internal callee is pure and no unknown/external behavior is present.
    bool changed = true;
    while (changed) {
        changed = false;
        for (auto& function : result.functions) {
            if (!function.pure) {
                continue;
            }
            for (const auto& call : function.calls) {
                const auto* callee = result.function(call);
                if (callee != nullptr && !callee->pure) {
                    function.pure = false;
                    changed = true;
                    break;
                }
            }
        }
    }
    return result;
}

} // namespace orlcomp
