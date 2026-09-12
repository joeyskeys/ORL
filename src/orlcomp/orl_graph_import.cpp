#include "orl_graph_import.h"
#include "orl_parser.h"

#include <filesystem>
#include <fstream>
#include <sstream>
#include <utility>

namespace orlcomp
{

namespace
{

orlgraph::Port make_input(const FunctionSummary& function,
    const FunctionParameterSummary& parameter)
{
    orlgraph::Port port;
    port.id = orlgraph::StableId::from(
        "orl.port", function.name + ".in." + parameter.name);
    port.name = parameter.name;
    port.direction = orlgraph::PortDirection::Input;
    port.cardinality = parameter.is_buffer
        ? orlgraph::PortCardinality::Buffer
        : orlgraph::PortCardinality::Scalar;
    port.type = parameter.is_buffer
        ? orlgraph::LogicalType::buffer(parameter.logical_type)
        : parameter.logical_type;
    port.domain = parameter.is_buffer
        ? orlgraph::Domain::buffer()
        : orlgraph::Domain::constant();
    port.shape = parameter.is_buffer
        ? orlgraph::Shape::one(parameter.name + "_count")
        : orlgraph::Shape::scalar();
    port.semantic = parameter.name;
    return port;
}

} // namespace

bool NodeImportResult::ok() const {
    return diagnostics.empty();
}

bool NodeRegistrationResult::ok() const {
    return diagnostics.empty();
}

NodeImportResult import_node_definitions(const AnalysisResult& analysis,
    std::string module_name)
{
    NodeImportResult result;
    result.diagnostics = analysis.diagnostics;
    if (!analysis.ok()) {
        return result;
    }

    for (const auto& function : analysis.functions) {
        orlgraph::NodeDefinition definition;
        definition.id = orlgraph::StableId::from("orl.function", function.name);
        definition.qualified_name = module_name + "." + function.name;
        definition.implementation.kind = orlgraph::ImplementationKind::OrlFunction;
        definition.implementation.module = module_name;
        definition.implementation.function = function.name;
        definition.inline_policy = function.pure
            ? orlgraph::InlinePolicy::Default
            : orlgraph::InlinePolicy::Never;
        definition.pure = function.pure;
        definition.stateful = function.stateful;
        definition.provenance.locations.push_back(function.source);
        definition.capabilities.push_back("cpu");
        if (function.has_parallel_for) {
            definition.capabilities.push_back("cuda");
        }

        for (const auto& parameter : function.parameters) {
            definition.inputs.push_back(make_input(function, parameter));
            if (parameter.access == ParameterAccess::None) {
                continue;
            }
            orlgraph::ResourceEffect effect;
            effect.resource = orlgraph::StableId::from(
                "orl.parameter", function.name + "." + parameter.name);
            const auto access = static_cast<std::uint8_t>(parameter.access);
            effect.observable = (access & static_cast<std::uint8_t>(
                ParameterAccess::Write)) != 0;
            const bool reads = (access & static_cast<std::uint8_t>(
                ParameterAccess::Read)) != 0;
            const bool writes = (access & static_cast<std::uint8_t>(
                ParameterAccess::Write)) != 0;
            effect.access = writes
                ? (reads ? orlgraph::AccessMode::ReadWrite : orlgraph::AccessMode::Write)
                : orlgraph::AccessMode::Read;
            definition.effects.push_back(std::move(effect));
        }

        if (function.return_type.kind != orlgraph::LogicalTypeKind::Void) {
            orlgraph::Port output;
            output.id = orlgraph::StableId::from(
                "orl.port", function.name + ".out.result");
            output.name = "result";
            output.direction = orlgraph::PortDirection::Output;
            output.cardinality = orlgraph::PortCardinality::Scalar;
            output.type = function.return_type;
            output.domain = orlgraph::Domain::constant();
            output.shape = orlgraph::Shape::scalar();
            output.required = false;
            definition.outputs.push_back(std::move(output));
        }

        std::string error;
        if (!result.registry.register_definition(std::move(definition), &error)) {
            result.diagnostics.push_back({
                "ORL_IMPORT_REGISTRY",
                std::move(error),
                function.source,
                true,
            });
        }
    }
    return result;
}

NodeImportResult import_node_definitions(std::string_view source,
    NodeImportOptions options)
{
    NodeImportResult result;
    if (options.module_name.empty()) {
        options.module_name = "orl";
    }
    if (options.source_name.empty()) {
        options.source_name = "<source>";
    }
    orlcomp::Parser parser(std::string{source});
    for (const auto& include_path : options.include_paths) {
        parser.AddIncludePath(include_path);
    }
    if (!parser.Parse() || parser.Ast() == nullptr) {
        for (const auto& error : parser.Errors()) {
            result.diagnostics.push_back({
                "ORL_IMPORT_PARSE",
                error,
                {options.source_name, 0, 0, 0, 0},
                true,
            });
        }
        if (result.diagnostics.empty()) {
            result.diagnostics.push_back({
                "ORL_IMPORT_PARSE",
                "Parser did not produce an ORL AST",
                {options.source_name, 0, 0, 0, 0},
                true,
            });
        }
        return result;
    }

    const auto analysis = SemanticAnalyzer{}.analyze(
        *parser.Ast(), options.source_name);
    if (options.exported_functions.empty()) {
        return import_node_definitions(analysis, std::move(options.module_name));
    }

    AnalysisResult exported;
    exported.diagnostics = analysis.diagnostics;
    for (const auto& function_name : options.exported_functions) {
        const auto* function = analysis.function(function_name);
        if (function == nullptr) {
            exported.diagnostics.push_back({
                "ORL_IMPORT_FUNCTION",
                "Requested exported function was not found: " + function_name,
                {options.source_name, 0, 0, 0, 0},
                true,
            });
            continue;
        }
        exported.functions.push_back(*function);
    }
    return import_node_definitions(exported, std::move(options.module_name));
}

NodeImportResult import_node_definitions_file(const std::string& path,
    NodeImportOptions options)
{
    std::ifstream input(path, std::ios::in | std::ios::binary);
    if (!input.is_open()) {
        NodeImportResult result;
        result.diagnostics.push_back({
            "ORL_IMPORT_READ",
            "Unable to open ORL source file: " + path,
            {path, 0, 0, 0, 0},
            true,
        });
        return result;
    }

    std::ostringstream contents;
    contents << input.rdbuf();
    if (!input && !input.eof()) {
        NodeImportResult result;
        result.diagnostics.push_back({
            "ORL_IMPORT_READ",
            "Failed to read ORL source file: " + path,
            {path, 0, 0, 0, 0},
            true,
        });
        return result;
    }

    if (options.source_name.empty() || options.source_name == "<source>") {
        options.source_name = path;
    }
    const std::filesystem::path source_path(path);
    const std::string include_directory =
        source_path.parent_path().empty()
            ? std::string{"."}
            : source_path.parent_path().string();
    options.include_paths.insert(options.include_paths.begin(),
        include_directory);
    return import_node_definitions(contents.str(), std::move(options));
}

NodeRegistrationResult register_orl_node_definitions(
    orlgraph::NodeRegistry& registry, NodeImportResult imported)
{
    NodeRegistrationResult result;
    result.diagnostics = std::move(imported.diagnostics);
    if (!result.diagnostics.empty()) {
        return result;
    }

    for (const auto& [id, definition] : imported.registry.definitions()) {
        if (registry.find(id) != nullptr) {
            result.diagnostics.push_back({
                "ORL_IMPORT_DUPLICATE_DEFINITION",
                "Node definition is already registered: " + id.value,
                {},
                true,
            });
        }
    }
    if (!result.diagnostics.empty()) {
        return result;
    }

    for (const auto& [_, definition] : imported.registry.definitions()) {
        std::string error;
        if (!registry.register_definition(definition, &error)) {
            result.diagnostics.push_back({
                "ORL_IMPORT_REGISTRY",
                std::move(error),
                {},
                true,
            });
            return result;
        }
        ++result.registered_count;
    }
    return result;
}

NodeRegistrationResult register_orl_node_definitions(
    orlgraph::NodeRegistry& registry, std::string_view source,
    NodeImportOptions options)
{
    return register_orl_node_definitions(registry,
        import_node_definitions(source, std::move(options)));
}

NodeRegistrationResult register_orl_node_definitions_file(
    orlgraph::NodeRegistry& registry, const std::string& path,
    NodeImportOptions options)
{
    return register_orl_node_definitions(registry,
        import_node_definitions_file(path, std::move(options)));
}

} // namespace orlcomp
