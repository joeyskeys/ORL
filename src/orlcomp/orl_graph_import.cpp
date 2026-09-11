#include "orl_graph_import.h"

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

} // namespace orlcomp
