#include "orl_analysis.h"
#include "orl_handle_views.hpp"

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

struct FunctionSignature {
    SemanticType return_type;
    std::vector<SemanticType> parameters;
};

SemanticType resolve_type(std::string_view name,
    const std::unordered_set<std::string>& structs,
    const std::unordered_map<std::string, std::string>& handles,
    const std::unordered_map<std::string, std::vector<std::string>>& leaves,
    const std::unordered_set<std::string>& open_handles,
    const std::unordered_map<std::string, std::string>& aliases)
{
    if (name == "handle") {
        if (const auto alias = aliases.find("handle");
            alias != aliases.end())
        {
            return {SemanticTypeKind::NominalHandle, "handle",
                alias->second, {}, false};
        }
        return {SemanticTypeKind::UniversalHandle, "handle", "handle"};
    }
    if (const auto found = handles.find(std::string{name});
        found != handles.end())
    {
        auto alias = aliases.find(found->first);
        if (alias == aliases.end()) {
            alias = aliases.find(found->second);
        }
        const auto canonical = alias == aliases.end()
            ? found->second : alias->second;
        const auto union_leaves = leaves.find(found->first);
        if (open_handles.contains(found->first)) {
            return {
                SemanticTypeKind::UniversalHandle,
                found->first, canonical, {}, true};
        }
        if (union_leaves != leaves.end()) {
            return {
                SemanticTypeKind::HandleUnion,
                found->first, canonical, union_leaves->second, false};
        }
        return {
            SemanticTypeKind::NominalHandle,
            found->first, canonical, {}, false};
    }
    if (structs.contains(std::string{name})) {
        return {SemanticTypeKind::Struct, std::string{name}, std::string{name}};
    }
    if (has_type(name)) {
        return {SemanticTypeKind::Builtin, std::string{name}, std::string{name}};
    }
    return {SemanticTypeKind::Unknown, std::string{name}, {}};
}

class FunctionAnalyzer {
public:
    FunctionAnalyzer(
        const std::unordered_map<std::string, FunctionSignature>& functions,
        const std::unordered_set<std::string>& structs,
        const std::unordered_map<std::string, std::string>& handles,
        const std::unordered_map<std::string, std::vector<std::string>>& leaves,
        const std::unordered_set<std::string>& open_handles,
        const std::unordered_map<std::string, std::string>& aliases,
        const std::string& source_name,
        const FunctionDefinitionStatement& definition,
        std::vector<AnalysisDiagnostic>* diagnostics)
        : functions_(functions)
        , structs_(structs)
        , handles_(handles)
        , leaves_(leaves)
        , open_handles_(open_handles)
        , aliases_(aliases)
        , source_name_(source_name)
        , definition_(definition)
        , diagnostics_(diagnostics)
    {
        summary_.name = definition.name;
        summary_.return_type_name = definition.return_type;
        summary_.return_type = LogicalType::from_orl_name(definition.return_type);
        summary_.resolved_return_type = type_of(definition.return_type);
        if (summary_.resolved_return_type.is_handle()) {
            summary_.return_type = {};
        }
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
            value.resolved_type = type_of(parameter.type_name);
            if (value.resolved_type.is_handle()) {
                value.logical_type = {};
            }
            value.is_buffer = parameter.is_buffer;
            summary_.parameters.push_back(std::move(value));
            parameter_indices_.emplace(parameter.name, summary_.parameters.size() - 1);
            variables_.emplace(parameter.name,
                summary_.parameters.back().resolved_type);
            if (summary_.parameters.back().resolved_type.is_handle())
            {
                handle_sources_.emplace(
                    parameter.name, summary_.parameters.size() - 1);
            }
        }
    }

    FunctionSummary run() {
        validate_type(definition_.return_type, "return type");
        for (const auto& parameter : definition_.parameters) {
            validate_type(parameter.type_name, "parameter type");
            if (parameter.is_buffer && type_of(parameter.type_name).is_handle()) {
                add_error("ORL_ANALYSIS_HANDLE_COLLECTION",
                    "Handle parameters cannot be buffers");
            }
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
    struct ViewBinding {
        std::size_t source_parameter = 0;
        const HandleViewDescriptor* descriptor = nullptr;
    };

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
        if (type_of(type_name).kind == SemanticTypeKind::Unknown) {
            add_error("ORL_ANALYSIS_UNSUPPORTED_TYPE",
                std::string{"Unsupported "} + context + ": " + std::string{type_name});
        }
    }

    SemanticType type_of(std::string_view name) const {
        return resolve_type(name, structs_, handles_, leaves_,
            open_handles_, aliases_);
    }

    static bool compatible(
        const SemanticType& expected, const SemanticType& actual)
    {
        if (expected.kind == SemanticTypeKind::Unknown
            || actual.kind == SemanticTypeKind::Unknown)
        {
            return true;
        }
        if (expected.kind == SemanticTypeKind::UniversalHandle
            && actual.is_handle())
        {
            return true;
        }
        if (expected.kind == SemanticTypeKind::HandleUnion) {
            if (actual.kind == SemanticTypeKind::NominalHandle) {
                return std::find(expected.accepted_handles.begin(),
                    expected.accepted_handles.end(),
                    actual.canonical_name)
                    != expected.accepted_handles.end();
            }
            if (actual.kind == SemanticTypeKind::HandleUnion) {
                return std::includes(
                    expected.accepted_handles.begin(),
                    expected.accepted_handles.end(),
                    actual.accepted_handles.begin(),
                    actual.accepted_handles.end());
            }
            return false;
        }
        return expected == actual;
    }

    static bool equality_compatible(
        const SemanticType& left, const SemanticType& right)
    {
        return compatible(left, right) || compatible(right, left);
    }

    void require_compatible(const SemanticType& expected,
        const SemanticType& actual, std::string_view context)
    {
        if (!compatible(expected, actual)
            && (expected.is_handle() || actual.is_handle()))
        {
            add_error("ORL_ANALYSIS_HANDLE_TYPE_MISMATCH",
                std::string{context} + " requires '" + expected.name
                    + "' but received '" + actual.name + "'");
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

    std::optional<std::size_t> handle_source(
        const Expression& expression) const
    {
        const auto* value =
            dynamic_cast<const IdentifierExpression*>(&expression);
        if (value == nullptr) {
            return std::nullopt;
        }
        const auto found = handle_sources_.find(value->name);
        return found == handle_sources_.end()
            ? std::nullopt : std::optional<std::size_t>{found->second};
    }

    SemanticType type_of_expression(const Expression& expression) const {
        const auto* value =
            dynamic_cast<const IdentifierExpression*>(&expression);
        if (value == nullptr) {
            return {};
        }
        const auto found = variables_.find(value->name);
        return found == variables_.end() ? SemanticType{}
            : found->second;
    }

    void add_view_effect(
        std::size_t parameter, std::string_view view,
        std::string_view field, ParameterAccess access_mode)
    {
        if (parameter >= summary_.parameters.size()) {
            return;
        }
        const std::string& parameter_name =
            summary_.parameters[parameter].name;
        for (auto& effect : summary_.handle_view_effects) {
            if (effect.parameter == parameter_name
                && effect.view_type == view
                && effect.field == field)
            {
                effect.access = add_access(effect.access, access_mode);
                return;
            }
        }
        summary_.handle_view_effects.push_back({
            parameter_name, std::string{view}, std::string{field},
            access_mode});
    }

    SemanticType view_field_type(
        const HandleViewFieldDescriptor& field) const
    {
        return field.type_name.empty() ? SemanticType{}
            : type_of(field.type_name);
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
            const SemanticType declared_type = type_of(declaration->type_name);
            if (declared_type.is_handle() && declaration->array_size != 0) {
                add_error("ORL_ANALYSIS_HANDLE_COLLECTION",
                    "Handle declarations cannot be arrays");
            }
            variables_[declaration->variable_name] = declared_type;
            if (declared_type.is_handle()
                && declaration->initializer != nullptr)
            {
                if (const auto source =
                        handle_source(*declaration->initializer))
                {
                    handle_sources_[declaration->variable_name] = *source;
                }
            }
            if (declaration->initializer != nullptr) {
                if (const auto* call = dynamic_cast<const CallExpression*>(
                        declaration->initializer.get());
                    call != nullptr && call->arguments.size() == 1)
                {
                    const auto source_type = type_of_expression(
                        *call->arguments.front());
                    const auto* view = source_type.kind
                            == SemanticTypeKind::NominalHandle
                        ? global_handle_view_registry().find(
                            source_type.canonical_name,
                            declaration->type_name)
                        : nullptr;
                    if (view != nullptr) {
                        if (const auto source = handle_source(
                                *call->arguments.front()))
                        {
                            view_bindings_[declaration->variable_name] =
                                {*source, view};
                        }
                    }
                }
            }
            if (declaration->initializer != nullptr) {
                require_compatible(declared_type,
                    visit_expression(*declaration->initializer),
                    "Declaration initializer");
            }
            if (declared_type.is_handle()
                && !declaration->constructor_arguments.empty())
            {
                add_error("ORL_ANALYSIS_HANDLE_CONVERSION",
                    "Handles cannot be constructed or converted");
            }
            for (const auto& argument : declaration->constructor_arguments) {
                visit_expression(*argument);
            }
        } else if (const auto* return_statement = dynamic_cast<const ReturnStatement*>(&statement)) {
            if (return_statement->value != nullptr) {
                require_compatible(summary_.resolved_return_type,
                    visit_expression(*return_statement->value),
                    "Return");
            } else if (summary_.resolved_return_type.is_handle()) {
                add_error("ORL_ANALYSIS_HANDLE_TYPE_MISMATCH",
                    "A handle-returning function must return a handle value");
            }
        } else if (const auto* conditional = dynamic_cast<const IfStatement*>(&statement)) {
            const auto* handle_test =
                dynamic_cast<const HandleTestExpression*>(
                    conditional->condition.get());
            const auto exact_before_test =
                handle_test == nullptr ? SemanticType{}
                    : type_of_expression(*handle_test->operand);
            visit_expression(*conditional->condition);
            if (handle_test != nullptr) {
                const auto* operand = dynamic_cast<const IdentifierExpression*>(
                    handle_test->operand.get());
                const auto tested = type_of(handle_test->type_name);
                if (exact_before_test.kind
                        == SemanticTypeKind::NominalHandle
                    && tested.kind == SemanticTypeKind::NominalHandle)
                {
                    const bool true_branch =
                        exact_before_test.canonical_name
                            == tested.canonical_name;
                    if (true_branch) {
                        visit_statement(*conditional->then_branch);
                    } else if (conditional->else_branch != nullptr) {
                        visit_statement(*conditional->else_branch);
                    }
                    return;
                }
                const auto found = operand == nullptr
                    ? variables_.end() : variables_.find(operand->name);
                const bool had_previous = found != variables_.end();
                const auto previous = had_previous
                    ? found->second : SemanticType{};
                auto restore = [&] {
                    if (operand == nullptr) {
                        return;
                    }
                    if (had_previous) {
                        variables_[operand->name] = previous;
                    } else {
                        variables_.erase(operand->name);
                    }
                };
                if (operand != nullptr
                    && tested.kind == SemanticTypeKind::NominalHandle)
                {
                    variables_[operand->name] = tested;
                }
                visit_statement(*conditional->then_branch);
                restore();

                if (conditional->else_branch != nullptr) {
                    if (operand != nullptr
                        && exact_before_test.kind
                            == SemanticTypeKind::HandleUnion
                        && tested.kind == SemanticTypeKind::NominalHandle)
                    {
                        auto remaining = exact_before_test.accepted_handles;
                        remaining.erase(std::remove(
                            remaining.begin(), remaining.end(),
                            tested.canonical_name), remaining.end());
                        if (remaining.size() == 1) {
                            variables_[operand->name] = {
                                SemanticTypeKind::NominalHandle,
                                tested.name, remaining.front(),
                                {}, false};
                        } else if (!remaining.empty()) {
                            variables_[operand->name] = {
                                SemanticTypeKind::HandleUnion,
                                exact_before_test.name,
                                exact_before_test.canonical_name,
                                std::move(remaining), false};
                        }
                    }
                    visit_statement(*conditional->else_branch);
                    restore();
                }
                return;
            } else {
                visit_statement(*conditional->then_branch);
                if (conditional->else_branch != nullptr) {
                    visit_statement(*conditional->else_branch);
                }
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

    SemanticType visit_expression(const Expression& expression) {
        if (const auto* value = dynamic_cast<const IdentifierExpression*>(&expression)) {
            if (view_bindings_.contains(value->name)) {
                add_error("ORL_ANALYSIS_HANDLE_VIEW_ESCAPE",
                    "Handle-backed views cannot be used as whole values");
            }
            access(value->name, ParameterAccess::Read);
            if (const auto found = variables_.find(value->name);
                found != variables_.end())
            {
                return found->second;
            }
            return {};
        } else if (const auto* literal = dynamic_cast<const LiteralExpression*>(&expression)) {
            if (literal->kind == LiteralKind::Int) {
                return type_of("int");
            }
            if (literal->kind == LiteralKind::Float) {
                return type_of("float");
            }
            return type_of("string");
        } else if (const auto* unary = dynamic_cast<const UnaryExpression*>(&expression)) {
            const SemanticType operand = visit_expression(*unary->operand);
            if (operand.is_handle()) {
                add_error("ORL_ANALYSIS_HANDLE_OPERATION",
                    "Unary operators are not supported on handles");
                return {};
            }
            return operand;
        } else if (const auto* binary = dynamic_cast<const BinaryExpression*>(&expression)) {
            const SemanticType left = visit_expression(*binary->left);
            const SemanticType right = visit_expression(*binary->right);
            if (left.is_handle() || right.is_handle()) {
                if (binary->op != BinaryOp::Equal
                    && binary->op != BinaryOp::NotEqual)
                {
                    add_error("ORL_ANALYSIS_HANDLE_OPERATION",
                        "Only equality operators are supported on handles");
                    return {};
                }
                if (!equality_compatible(left, right)) {
                    add_error("ORL_ANALYSIS_HANDLE_TYPE_MISMATCH",
                        "Handle equality requires compatible handle types");
                }
                return type_of("bool");
            }
            return left;
        } else if (const auto* handle_test =
            dynamic_cast<const HandleTestExpression*>(&expression)) {
            const auto actual = visit_expression(*handle_test->operand);
            const auto tested = type_of(handle_test->type_name);
            if (!actual.is_handle()) {
                add_error("ORL_ANALYSIS_HANDLE_TEST",
                    "The left operand of 'is' must be a handle");
            }
            if (tested.kind != SemanticTypeKind::NominalHandle) {
                add_error("ORL_ANALYSIS_HANDLE_TEST",
                    "The right operand of 'is' must be an exact handle type");
            }
            return type_of("bool");
        } else if (const auto* assignment = dynamic_cast<const AssignmentExpression*>(&expression)) {
            if (assignment->target_name == "solver_context"
                || assignment->target_name == "hierarchy_context"
                || assignment->target_name == "hierarchy_data")
            {
                add_error("ORL_ANALYSIS_CONTEXT_WRITE",
                    "The implicit runtime context global is read-only");
            }
            access(assignment->target_name, ParameterAccess::Write);
            const SemanticType actual = visit_expression(*assignment->value);
            if (type_of(assignment->target_name).is_handle()) {
                if (const auto source = handle_source(*assignment->value)) {
                    handle_sources_[assignment->target_name] = *source;
                }
            }
            if (const auto found = variables_.find(assignment->target_name);
                found != variables_.end())
            {
                require_compatible(found->second, actual, "Assignment");
                return found->second;
            }
            return actual;
        } else if (const auto* call = dynamic_cast<const CallExpression*>(&expression)) {
            const auto* callee = identifier(call->callee.get());
            std::vector<SemanticType> argument_types;
            argument_types.reserve(call->arguments.size());
            for (const auto& argument : call->arguments) {
                argument_types.push_back(visit_expression(*argument));
            }
            if (callee == nullptr) {
                add_error("ORL_ANALYSIS_INDIRECT_CALL",
                    "Only direct function calls can be imported as graph nodes");
            } else {
                const std::string& name = callee->name;
                const bool constructor = is_supported_orl_type(name)
                    || structs_.contains(name) || name == "handle"
                    || handles_.contains(name);
                if (!functions_.contains(name) && !is_known_orl_builtin(name)
                    && !constructor)
                {
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
                if (const auto found = functions_.find(name);
                    found != functions_.end())
                {
                    HandleCallSiteSummary call_site;
                    call_site.callee = name;
                    for (const auto& argument : call->arguments) {
                        call_site.handle_sources.push_back(
                            handle_source(*argument));
                    }
                    summary_.handle_calls.push_back(std::move(call_site));
                    if (found->second.parameters.size() != argument_types.size()) {
                        add_error("ORL_ANALYSIS_CALL_ARGUMENT",
                            "Function '" + name + "' expects "
                                + std::to_string(found->second.parameters.size())
                                + " arguments");
                    }
                    const std::size_t count = std::min(
                        found->second.parameters.size(), argument_types.size());
                    for (std::size_t i = 0; i < count; ++i) {
                        require_compatible(found->second.parameters[i],
                            argument_types[i], "Call argument");
                    }
                    return found->second.return_type;
                }
                if (constructor) {
                    const SemanticType result = type_of(name);
                    const bool has_handle_argument = std::any_of(
                        argument_types.begin(), argument_types.end(),
                        [](const SemanticType& type) {
                            return type.is_handle();
                        });
                    if (has_handle_argument) {
                        const bool exact_view = result.kind
                                == SemanticTypeKind::Struct
                            && argument_types.size() == 1
                            && argument_types.front().kind
                                == SemanticTypeKind::NominalHandle
                            && global_handle_view_registry().find(
                                argument_types.front().canonical_name,
                                result.name) != nullptr;
                        if (exact_view) {
                            return result;
                        }
                        add_error("ORL_ANALYSIS_HANDLE_CONVERSION",
                            "Implicit conversion to or from a handle is not supported");
                        return {};
                    }
                    return result;
                }
            }
            return {};
        } else if (const auto* index = dynamic_cast<const IndexExpression*>(&expression)) {
            const SemanticType base = visit_expression(*index->base);
            visit_expression(*index->index);
            if (base.is_handle()) {
                add_error("ORL_ANALYSIS_HANDLE_INDEX",
                    "Handles cannot be indexed");
                return {};
            }
            return base;
        } else if (const auto* assignment = dynamic_cast<const IndexAssignmentExpression*>(&expression)) {
            SemanticType base_type;
            if (const auto* base = base_identifier(assignment->target.get())) {
                if (base->name == "hierarchy_data") {
                    add_error("ORL_ANALYSIS_CONTEXT_WRITE",
                        "The implicit hierarchy_data global is read-only");
                }
                access(base->name, ParameterAccess::Write);
                if (const auto found = variables_.find(base->name);
                    found != variables_.end())
                {
                    base_type = found->second;
                }
            }
            if (assignment->target != nullptr && assignment->target->index != nullptr) {
                visit_expression(*assignment->target->index);
            }
            visit_expression(*assignment->value);
            if (base_type.is_handle()) {
                add_error("ORL_ANALYSIS_HANDLE_INDEX",
                    "Handles cannot be indexed");
            }
            return base_type;
        } else if (const auto* component = dynamic_cast<const ComponentExpression*>(&expression)) {
            if (const auto* base = identifier(component->base.get())) {
                if (const auto found = view_bindings_.find(base->name);
                    found != view_bindings_.end())
                {
                    const auto* field =
                        global_handle_view_registry().field(
                            *found->second.descriptor,
                            component->component);
                    if (field == nullptr) {
                        add_error("ORL_ANALYSIS_HANDLE_VIEW_FIELD",
                            "Unknown field '" + component->component
                                + "' on "
                                + found->second.descriptor
                                    ->destination_struct);
                        return {};
                    }
                    add_view_effect(
                        found->second.source_parameter,
                        found->second.descriptor->destination_struct,
                        field->name, ParameterAccess::Read);
                    return view_field_type(*field);
                }
            }
            const SemanticType base = visit_expression(*component->base);
            if (base.is_handle()) {
                add_error("ORL_ANALYSIS_HANDLE_MEMBER",
                    "Handles do not support direct member access");
                return {};
            }
            return {};
        } else if (const auto* assignment = dynamic_cast<const MemberAssignmentExpression*>(&expression)) {
            SemanticType base_type;
            if (assignment->target != nullptr) {
                if (const auto* base = identifier(
                        assignment->target->base.get()))
                {
                    if (const auto found = view_bindings_.find(base->name);
                        found != view_bindings_.end())
                    {
                        const auto* field =
                            global_handle_view_registry().field(
                                *found->second.descriptor,
                                assignment->target->component);
                        if (field == nullptr) {
                            add_error("ORL_ANALYSIS_HANDLE_VIEW_FIELD",
                                "Unknown field '"
                                    + assignment->target->component
                                    + "' on "
                                    + found->second.descriptor
                                        ->destination_struct);
                            return {};
                        }
                        add_view_effect(
                            found->second.source_parameter,
                            found->second.descriptor->destination_struct,
                            field->name, ParameterAccess::Write);
                        visit_expression(*assignment->value);
                        return view_field_type(*field);
                    }
                }
                if (const auto* base = base_identifier(assignment->target->base.get())) {
                    if (base->name == "solver_context"
                        || base->name == "hierarchy_context"
                        || base->name == "hierarchy_data")
                    {
                        add_error("ORL_ANALYSIS_CONTEXT_WRITE",
                            "The implicit runtime context global is read-only");
                    }
                    access(base->name, ParameterAccess::Write);
                    if (const auto found = variables_.find(base->name);
                        found != variables_.end())
                    {
                        base_type = found->second;
                    }
                }
            }
            visit_expression(*assignment->value);
            if (base_type.is_handle()) {
                add_error("ORL_ANALYSIS_HANDLE_MEMBER",
                    "Handles do not support direct member access");
            }
            return base_type;
        }
        return {};
    }

    const std::unordered_map<std::string, FunctionSignature>& functions_;
    const std::unordered_set<std::string>& structs_;
    const std::unordered_map<std::string, std::string>& handles_;
    const std::unordered_map<std::string, std::vector<std::string>>& leaves_;
    const std::unordered_set<std::string>& open_handles_;
    const std::unordered_map<std::string, std::string>& aliases_;
    const std::string& source_name_;
    const FunctionDefinitionStatement& definition_;
    std::vector<AnalysisDiagnostic>* diagnostics_;
    FunctionSummary summary_;
    std::unordered_map<std::string, std::size_t> parameter_indices_;
    std::unordered_map<std::string, SemanticType> variables_;
    std::unordered_map<std::string, std::size_t> handle_sources_;
    std::unordered_map<std::string, ViewBinding> view_bindings_;
};

} // namespace

bool SemanticType::is_handle() const {
    return kind == SemanticTypeKind::UniversalHandle
        || kind == SemanticTypeKind::NominalHandle
        || kind == SemanticTypeKind::HandleUnion;
}

bool SemanticType::operator==(const SemanticType& other) const {
    if (kind != other.kind) {
        return false;
    }
    if (kind == SemanticTypeKind::NominalHandle
        || kind == SemanticTypeKind::HandleUnion
        || kind == SemanticTypeKind::UniversalHandle)
    {
        return canonical_name == other.canonical_name
            && accepted_handles == other.accepted_handles
            && open_handle == other.open_handle;
    }
    return name == other.name;
}

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
    std::string source_name,
    const std::unordered_map<std::string, std::string>&
        handle_type_identities) const
{
    AnalysisResult result;
    std::unordered_map<std::string, FunctionSignature> functions;
    std::unordered_set<std::string> structs;
    std::unordered_map<std::string, std::string> handles;
    std::unordered_map<std::string, std::vector<std::string>> leaves;
    std::unordered_set<std::string> open_handles;

    for (const auto& item : program.items) {
        if (const auto* structure = dynamic_cast<const StructDefinitionStatement*>(item.get())) {
            if (handles.contains(structure->name)) {
                result.diagnostics.push_back({
                    "ORL_ANALYSIS_TYPE_COLLISION",
                    "Type name is declared as both struct and handle: "
                        + structure->name,
                    {source_name, 0, 0, 0, 0},
                    true,
                });
            }
            if (!structs.insert(structure->name).second) {
                result.diagnostics.push_back({
                    "ORL_ANALYSIS_DUPLICATE_STRUCT",
                    "Duplicate struct definition: " + structure->name,
                    {source_name, 0, 0, 0, 0},
                    true,
                });
            }
        } else if (const auto* handle =
            dynamic_cast<const HandleDefinitionStatement*>(item.get()))
        {
            if (structs.contains(handle->name)) {
                result.diagnostics.push_back({
                    "ORL_ANALYSIS_TYPE_COLLISION",
                    "Type name is declared as both struct and handle: "
                        + handle->name,
                    {handle->source.source,
                        static_cast<std::uint32_t>(handle->source.line),
                        static_cast<std::uint32_t>(handle->source.column), 0, 0},
                    true,
                });
            }
            if (!handles.emplace(
                    handle->name, handle->canonical_name).second)
            {
                result.diagnostics.push_back({
                    "ORL_ANALYSIS_DUPLICATE_HANDLE",
                    "Duplicate handle definition: " + handle->name,
                    {handle->source.source,
                        static_cast<std::uint32_t>(handle->source.line),
                        static_cast<std::uint32_t>(handle->source.column), 0, 0},
                    true,
                });
            } else {
                if (!handle->accepted_handles.empty()) {
                    leaves.emplace(handle->name,
                        handle->accepted_handles);
                }
                if (handle->open) {
                    open_handles.insert(handle->name);
                }
                result.handle_types.push_back({
                    handle->name,
                    handle->canonical_name,
                    {handle->source.source,
                        static_cast<std::uint32_t>(handle->source.line),
                        static_cast<std::uint32_t>(handle->source.column), 0, 0},
                });
            }
        }
    }
    for (const auto& item : program.items) {
        if (const auto* function = dynamic_cast<const FunctionDefinitionStatement*>(item.get())) {
            FunctionSignature signature;
            signature.return_type = resolve_type(
                function->return_type, structs, handles, leaves,
                open_handles,
                handle_type_identities);
            for (const auto& parameter : function->parameters) {
                signature.parameters.push_back(resolve_type(
                    parameter.type_name, structs, handles, leaves,
                    open_handles,
                    handle_type_identities));
            }
            if (!functions.emplace(function->name, std::move(signature)).second) {
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
        FunctionAnalyzer analyzer(functions, structs, handles,
            leaves, open_handles,
            handle_type_identities, source_name,
            *function, &result.diagnostics);
        result.functions.push_back(analyzer.run());
    }

    // Remap direct view effects through helper calls. A call site records
    // which caller handle parameter supplied each callee argument, so the
    // source handle remains compiler-internal provenance rather than a
    // user-visible reference type.
    bool view_effects_changed = true;
    while (view_effects_changed) {
        view_effects_changed = false;
        for (auto& function : result.functions) {
            for (const auto& call_site : function.handle_calls) {
                const auto* callee = result.function(call_site.callee);
                if (callee == nullptr) {
                    continue;
                }
                for (const auto& effect : callee->handle_view_effects) {
                    std::size_t callee_parameter = 0;
                    while (callee_parameter < callee->parameters.size()
                        && callee->parameters[callee_parameter].name
                            != effect.parameter)
                    {
                        ++callee_parameter;
                    }
                    if (callee_parameter >= call_site.handle_sources.size()
                        || callee_parameter >= callee->parameters.size())
                    {
                        continue;
                    }
                    const auto source =
                        call_site.handle_sources[callee_parameter];
                    if (!source.has_value()
                        || *source >= function.parameters.size())
                    {
                        continue;
                    }
                    const std::string& caller_parameter =
                        function.parameters[*source].name;
                    auto found = std::find_if(
                        function.handle_view_effects.begin(),
                        function.handle_view_effects.end(),
                        [&](const HandleViewEffectSummary& candidate) {
                            return candidate.parameter == caller_parameter
                                && candidate.view_type == effect.view_type
                                && candidate.field == effect.field;
                        });
                    if (found == function.handle_view_effects.end()) {
                        function.handle_view_effects.push_back({
                            caller_parameter, effect.view_type, effect.field,
                            effect.access});
                        view_effects_changed = true;
                    } else {
                        const auto merged = add_access(
                            found->access, effect.access);
                        if (merged != found->access) {
                            found->access = merged;
                            view_effects_changed = true;
                        }
                    }
                }
            }
        }
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
