#include "orl_graph_import.h"
#include "orl_parser.h"

#include <filesystem>
#include <fstream>
#include <optional>
#include <sstream>
#include <utility>
#include <variant>

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

std::optional<orlgraph::GraphStageMask> stage_mask(
    const FunctionSummary& function, std::string* error)
{
    const auto found = function.metadata.find("stage");
    if (found == function.metadata.end()) {
        return orlgraph::GraphStageMask::All;
    }
    const auto* value = std::get_if<std::string>(&found->second.value);
    if (value == nullptr) {
        if (error != nullptr) {
            *error = "Function metadata 'stage' must be a string";
        }
        return std::nullopt;
    }
    if (*value == "solver") {
        return orlgraph::GraphStageMask::Solver;
    }
    if (*value == "deformer") {
        return orlgraph::GraphStageMask::Deformer;
    }
    if (*value == "all" || *value == "both") {
        return orlgraph::GraphStageMask::All;
    }
    if (error != nullptr) {
        *error = "Unknown function metadata stage '" + *value
            + "' (expected solver, deformer, or all)";
    }
    return std::nullopt;
}

std::vector<std::string> metadata_list(
    const FunctionSummary& function, std::string_view key)
{
    const auto found = function.metadata.find(std::string{key});
    if (found == function.metadata.end()) {
        return {};
    }
    const auto* value = std::get_if<std::string>(&found->second.value);
    if (value == nullptr) {
        return {};
    }
    std::vector<std::string> result;
    std::size_t begin = 0;
    while (begin < value->size()) {
        const auto end = value->find(',', begin);
        const auto token_end = end == std::string::npos
            ? value->size() : end;
        const auto first = value->find_first_not_of(" \t", begin);
        if (first != std::string::npos && first < token_end) {
            const auto last = value->find_last_not_of(" \t",
                token_end - 1);
            result.emplace_back(value->substr(first, last - first + 1));
        }
        if (end == std::string::npos) {
            break;
        }
        begin = end + 1;
    }
    return result;
}

bool metadata_flag(const FunctionSummary& function, std::string_view key) {
    const auto found = function.metadata.find(std::string{key});
    if (found == function.metadata.end()) {
        return false;
    }
    if (const auto* value = std::get_if<std::int64_t>(
            &found->second.value))
    {
        return *value != 0;
    }
    if (const auto* value = std::get_if<std::string>(
            &found->second.value))
    {
        return *value == "true" || *value == "1" || *value == "yes";
    }
    return false;
}

bool add_partial_footprint(const FunctionSummary& function,
    orlgraph::NodeDefinition* definition, std::string* error)
{
    const bool declared = function.metadata.contains("partial")
        || function.metadata.contains("partial_propagation")
        || function.metadata.contains("partial_global")
        || function.metadata.contains("partial_sparse")
        || function.metadata.contains("partial_read_joint_ports")
        || function.metadata.contains("partial_write_joint_ports")
        || function.metadata.contains("partial_read_controller_ports")
        || function.metadata.contains("partial_read_locator_ports")
        || function.metadata.contains("partial_read_joints")
        || function.metadata.contains("partial_write_joints")
        || function.metadata.contains("partial_read_controllers")
        || function.metadata.contains("partial_read_locators")
        || function.metadata.contains("partial_read_resources")
        || function.metadata.contains("partial_write_resources");
    if (!declared) {
        return true;
    }

    orlgraph::PartialEvaluationFootprint footprint;
    footprint.declared = true;
    footprint.global = metadata_flag(function, "partial_global");
    footprint.supports_sparse_dispatch =
        metadata_flag(function, "partial_sparse");
    if (const auto found = function.metadata.find("partial_propagation");
        found != function.metadata.end())
    {
        const auto* value = std::get_if<std::string>(&found->second.value);
        if (value == nullptr) {
            if (error != nullptr) {
                *error = "Function metadata 'partial_propagation' must be a string";
            }
            return false;
        }
        if (*value == "none") {
            footprint.propagation = orlgraph::PartialPropagation::None;
        } else if (*value == "ancestors") {
            footprint.propagation =
                orlgraph::PartialPropagation::Ancestors;
        } else if (*value == "descendants") {
            footprint.propagation =
                orlgraph::PartialPropagation::Descendants;
        } else if (*value == "ancestors_and_descendants") {
            footprint.propagation =
                orlgraph::PartialPropagation::AncestorsAndDescendants;
        } else if (*value == "full") {
            footprint.propagation = orlgraph::PartialPropagation::Full;
        } else {
            if (error != nullptr) {
                *error = "Unknown partial propagation rule '" + *value + "'";
            }
            return false;
        }
    } else {
        footprint.propagation = orlgraph::PartialPropagation::None;
    }
    footprint.read_joint_ports = metadata_list(
        function, "partial_read_joint_ports");
    footprint.write_joint_ports = metadata_list(
        function, "partial_write_joint_ports");
    footprint.read_controller_ports = metadata_list(
        function, "partial_read_controller_ports");
    footprint.read_locator_ports = metadata_list(
        function, "partial_read_locator_ports");
    for (const auto& value :
        metadata_list(function, "partial_read_controllers"))
    {
        footprint.read_controllers.push_back(orlgraph::StableId{value});
    }
    for (const auto& value :
        metadata_list(function, "partial_read_locators"))
    {
        footprint.read_locators.push_back(orlgraph::StableId{value});
    }
    for (const auto& value : metadata_list(function, "partial_read_joints")) {
        footprint.read_joints.push_back(orlgraph::StableId{value});
    }
    for (const auto& value : metadata_list(function, "partial_write_joints")) {
        footprint.write_joints.push_back(orlgraph::StableId{value});
    }
    for (const auto& value :
        metadata_list(function, "partial_read_resources"))
    {
        footprint.read_resources.push_back(orlgraph::StableId{value});
    }
    for (const auto& value :
        metadata_list(function, "partial_write_resources"))
    {
        footprint.write_resources.push_back(orlgraph::StableId{value});
    }
    if (footprint.read_joint_ports.empty()
        && footprint.write_joint_ports.empty()
        && footprint.read_joints.empty()
        && footprint.write_joints.empty()
        && footprint.read_controller_ports.empty()
        && footprint.read_locator_ports.empty()
        && footprint.read_controllers.empty()
        && footprint.read_locators.empty()
        && footprint.read_resources.empty()
        && footprint.write_resources.empty())
    {
        footprint.global = true;
    }
    definition->partial_footprint = std::move(footprint);
    return true;
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
        if (!function.exported) {
            continue;
        }
        std::string stage_error;
        const auto allowed_stages = stage_mask(function, &stage_error);
        if (!allowed_stages.has_value()) {
            result.diagnostics.push_back({
                "ORL_IMPORT_STAGE",
                std::move(stage_error),
                function.source,
                true,
            });
            continue;
        }
        orlgraph::NodeDefinition definition;
        definition.id = orlgraph::StableId::from("orl.function", function.name);
        definition.qualified_name = module_name + "." + function.name;
        definition.allowed_stages = *allowed_stages;
        definition.metadata = function.metadata;
        definition.implementation.kind = orlgraph::ImplementationKind::OrlFunction;
        definition.implementation.module = module_name;
        definition.implementation.function = function.name;
        definition.inline_policy = function.pure
            ? orlgraph::InlinePolicy::Default
            : orlgraph::InlinePolicy::Never;
        definition.pure = function.pure;
        definition.stateful = function.stateful;
        std::string footprint_error;
        if (!add_partial_footprint(
                function, &definition, &footprint_error))
        {
            result.diagnostics.push_back({
                "ORL_IMPORT_PARTIAL_FOOTPRINT",
                std::move(footprint_error),
                function.source,
                true,
            });
            continue;
        }
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
        if (!function->exported) {
            exported.diagnostics.push_back({
                "ORL_IMPORT_EXPORT",
                "Requested function is not exported: " + function_name,
                function->source,
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
