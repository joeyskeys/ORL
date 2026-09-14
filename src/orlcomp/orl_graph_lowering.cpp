#include "orl_graph_lowering.h"

#include "graph_schedule.hpp"
#include "graph_validation.hpp"

#include <algorithm>
#include <cctype>
#include <map>
#include <set>
#include <sstream>
#include <type_traits>
#include <utility>

namespace orlcomp
{

namespace
{

void error(LoweredGraph& result, std::string code, std::string message) {
    result.diagnostics.push_back({
        std::move(code),
        std::move(message),
    });
}

std::string identifier(std::string_view value) {
    std::string result;
    result.reserve(value.size() + 1);
    for (const char character : value) {
        result.push_back(std::isalnum(static_cast<unsigned char>(character))
            || character == '_' ? character : '_');
    }
    if (result.empty() || std::isdigit(static_cast<unsigned char>(result.front()))) {
        result.insert(result.begin(), '_');
    }
    return result;
}

std::string type_name(const orlgraph::LogicalType& type) {
    using Kind = orlgraph::LogicalTypeKind;
    switch (type.kind) {
    case Kind::Bool: return "bool";
    case Kind::Int64: return "int";
    case Kind::Float64: return "float";
    case Kind::String: return "string";
    case Kind::Vector: return "vector";
    case Kind::Point: return "point";
    case Kind::Normal: return "normal";
    case Kind::Vec4: return "vec4";
    case Kind::Quaternion: return "quat";
    case Kind::Matrix: return "matrix";
    case Kind::Struct: return type.name;
    case Kind::Buffer:
        return type.element == nullptr ? "int" : type_name(*type.element);
    default:
        return {};
    }
}

void add_type_module(const orlgraph::LogicalType& type,
    std::set<std::string>& modules)
{
    using Kind = orlgraph::LogicalTypeKind;
    if (type.kind == Kind::Buffer && type.element != nullptr) {
        add_type_module(*type.element, modules);
        return;
    }
    if (type.kind != Kind::Struct) {
        return;
    }
    if (type.name == "Joint") {
        modules.insert("joint");
    } else if (type.name == "Weight") {
        modules.insert("weight");
    } else if (type.name == "Locator") {
        modules.insert("locator");
    } else if (type.name == "Controller") {
        modules.insert("controller");
    }
}

std::string literal(const orlgraph::ConstantValue& value) {
    std::ostringstream stream;
    std::visit([&stream](const auto& item) {
        using T = std::decay_t<decltype(item)>;
        if constexpr (std::is_same_v<T, std::monostate>) {
            stream << "0";
        } else if constexpr (std::is_same_v<T, bool>) {
            stream << (item ? "1" : "0");
        } else if constexpr (std::is_same_v<T, std::int64_t>
            || std::is_same_v<T, double>)
        {
            stream << item;
        } else if constexpr (std::is_same_v<T, std::string>) {
            stream << "\"" << item << "\"";
        } else {
            stream << "0";
        }
    }, value.value);
    return stream.str();
}

const orlgraph::Connection* connection_to(
    const orlgraph::GraphModule& module, const orlgraph::StableId& node,
    const orlgraph::StableId& port)
{
    for (const auto& connection : module.connections()) {
        if (connection.destination.kind == orlgraph::EndpointKind::NodePort
            && connection.destination.owner == node
            && connection.destination.port == port)
        {
            return &connection;
        }
    }
    return nullptr;
}

const orlgraph::Connection* connection_to_output(
    const orlgraph::GraphModule& module, const orlgraph::StableId& output)
{
    for (const auto& connection : module.connections()) {
        if (connection.destination.kind == orlgraph::EndpointKind::GraphOutput
            && connection.destination.owner == output)
        {
            return &connection;
        }
    }
    return nullptr;
}

bool output_is_used(const orlgraph::GraphModule& module,
    const orlgraph::StableId& node, const orlgraph::Port& port)
{
    return std::any_of(module.connections().begin(), module.connections().end(),
        [&node, &port](const orlgraph::Connection& connection) {
            return connection.source.kind == orlgraph::EndpointKind::NodePort
                && connection.source.owner == node
                && (connection.source.port == port.id
                    || connection.source.port.value == port.name);
        });
}

const orlgraph::InterfacePort* find_auxiliary_input(
    const orlgraph::GraphModule& module, const orlgraph::Port& auxiliary)
{
    for (const auto& [id, input] : module.inputs()) {
        if ((!auxiliary.semantic.empty()
                && (input.binding == auxiliary.semantic
                    || input.semantic == auxiliary.semantic))
            || input.id == auxiliary.id
            || input.name == auxiliary.name)
        {
            return &input;
        }
    }
    return nullptr;
}

} // namespace

LoweredGraph OrlGraphLowerer::lower(const orlgraph::GraphModule& module,
    const orlgraph::NodeRegistry& registry, GraphLoweringOptions options) const
{
    LoweredGraph result;
    result.entry_function = options.entry_function;
    result.scene_revision = options.scene_revision;
    if (result.entry_function.empty()) {
        error(result, "ORL_LOWERING_ENTRY", "Graph entry function name is empty");
        return result;
    }

    const auto validation = orlgraph::validate(module, registry);
    if (!validation.ok()) {
        for (const auto& diagnostic : validation.diagnostics) {
            error(result, diagnostic.code, diagnostic.message);
        }
        return result;
    }

    std::set<std::string> modules;
    if (options.emit_module_uses) {
        for (const auto& [_, input] : module.inputs()) {
            add_type_module(input.type, modules);
        }
        for (const auto& [_, node] : module.nodes()) {
            const auto* definition = registry.find(node.definition);
            if (definition != nullptr
                && definition->implementation.kind == orlgraph::ImplementationKind::OrlFunction
                && !definition->implementation.module.empty())
            {
                modules.insert(definition->implementation.module);
            }
            if (definition == nullptr) {
                continue;
            }
            for (const auto& output : definition->outputs) {
                if (!output.output_adapter.has_value()
                    || !output_is_used(module, node.id, output))
                {
                    continue;
                }
                const auto* conversion = registry.find_conversion(
                    output.output_adapter->conversion);
                if (conversion != nullptr
                    && conversion->implementation.kind
                        == orlgraph::ImplementationKind::OrlFunction
                    && !conversion->implementation.module.empty())
                {
                    modules.insert(conversion->implementation.module);
                }
                if (output.output_adapter->writeback_conversion.has_value()) {
                    const auto* writeback = registry.find_conversion(
                        *output.output_adapter->writeback_conversion);
                    if (writeback != nullptr
                        && writeback->implementation.kind
                            == orlgraph::ImplementationKind::OrlFunction
                        && !writeback->implementation.module.empty())
                    {
                        modules.insert(writeback->implementation.module);
                    }
                }
            }
        }
        for (const auto& connection : module.connections()) {
            if (connection.conversion.empty()) {
                continue;
            }
            const auto* conversion =
                registry.find_conversion(connection.conversion);
            if (conversion != nullptr
                && conversion->implementation.kind
                    == orlgraph::ImplementationKind::OrlFunction
                && !conversion->implementation.module.empty())
            {
                modules.insert(conversion->implementation.module);
            }
        }
    }

    std::ostringstream source;
    for (const auto& module_name : modules) {
        source << "use " << module_name << ";\n";
    }
    if (!modules.empty()) {
        source << '\n';
    }
    source << options.source_preamble;
    if (!options.source_preamble.empty()
        && options.source_preamble.back() != '\n')
    {
        source << '\n';
    }

    source << "int " << identifier(result.entry_function) << "(";
    bool first = true;
    std::map<orlgraph::StableId, std::string> input_names;
    for (const auto& [id, input] : module.inputs()) {
        if (!first) {
            source << ", ";
        }
        first = false;
        const std::string name = identifier(input.name.empty() ? id.value : input.name);
        input_names.emplace(id, name);
        const std::string value_type = type_name(input.type);
        if (value_type.empty()) {
            error(result, "ORL_LOWERING_TYPE",
                "Graph input has no ORL type: " + id.value);
            return result;
        }
        source << value_type;
        if (input.type.kind == orlgraph::LogicalTypeKind::Buffer) {
            source << " " << name << "[]";
        } else {
            source << " " << name;
        }
    }
    source << ") {\n";

    std::map<std::string, std::string> output_expressions;
    const auto expression_for_endpoint =
        [&input_names, &output_expressions](
            const orlgraph::Endpoint& endpoint) {
            if (endpoint.kind == orlgraph::EndpointKind::GraphInput) {
                const auto found = input_names.find(endpoint.owner);
                return found == input_names.end() ? std::string{} : found->second;
            }
            if (endpoint.kind == orlgraph::EndpointKind::NodePort) {
                const auto found = output_expressions.find(
                    endpoint.owner.value + ":" + endpoint.port.value);
                return found == output_expressions.end()
                    ? std::string{} : found->second;
            }
            return std::string{};
        };

    const auto emit_conversion =
        [&source, &module, &input_names, &result](
            const orlgraph::ConversionDefinition& conversion,
            std::string_view source_expression,
            std::string_view instance_key,
            std::string* output_expression) {
            if (conversion.implementation.function.empty()) {
                error(result, "ORL_LOWERING_CONVERSION_FUNCTION",
                    "Conversion has no ORL function: "
                        + conversion.id.value);
                return false;
            }
            std::vector<std::string> arguments;
            for (const auto& auxiliary : conversion.auxiliary_inputs) {
                const auto* input = find_auxiliary_input(module, auxiliary);
                if (input == nullptr) {
                    error(result, "ORL_LOWERING_CONVERSION_INPUT",
                        "Unable to resolve conversion input: "
                            + auxiliary.name);
                    return false;
                }
                const auto found = input_names.find(input->id);
                if (found == input_names.end()) {
                    error(result, "ORL_LOWERING_CONVERSION_SOURCE",
                        "Unable to lower conversion input: "
                            + auxiliary.name);
                    return false;
                }
                arguments.push_back(found->second);
            }
            arguments.emplace_back(source_expression);

            const std::string variable = "conversion_"
                + identifier(instance_key);
            if (conversion.emitter
                == orlgraph::ConversionEmitterKind::OrlMatrixBuffer)
            {
                if (conversion.output.type.kind
                    != orlgraph::LogicalTypeKind::Buffer
                    || conversion.output.type.element == nullptr
                    || conversion.output.type.element->kind
                        != orlgraph::LogicalTypeKind::Matrix)
                {
                    error(result, "ORL_LOWERING_CONVERSION_TYPE",
                        "Matrix-buffer conversion has a non-matrix output: "
                            + conversion.id.value);
                    return false;
                }
                source << "    matrix " << variable << "[1];\n";
                source << "    " << variable << "[0] = "
                       << conversion.implementation.function << "(";
            } else {
                const std::string output_type =
                    type_name(conversion.output.type);
                if (output_type.empty()
                    || conversion.output.type.kind
                        == orlgraph::LogicalTypeKind::Buffer)
                {
                    error(result, "ORL_LOWERING_CONVERSION_TYPE",
                        "Unsupported conversion output type: "
                            + conversion.id.value);
                    return false;
                }
                source << "    " << output_type << " " << variable
                       << " = " << conversion.implementation.function << "(";
            }
            for (std::size_t index = 0; index < arguments.size(); ++index) {
                if (index != 0) {
                    source << ", ";
                }
                source << arguments[index];
            }
            source << ");\n";
            *output_expression = variable;
            return true;
        };

    struct WritableAdapterState {
        std::string temporary;
        std::string stable_handle;
        std::string writeback_index;
        orlgraph::StableId writeback_conversion;
    };
    std::map<std::string, WritableAdapterState> writable_adapters;

    const auto emit_writeback =
        [&source, &module, &input_names, &result](
            const orlgraph::ConversionDefinition& conversion,
            std::string_view temporary,
            std::string_view selector,
            std::string_view instance_key) {
            if (conversion.implementation.function.empty()) {
                error(result, "ORL_LOWERING_WRITEBACK_FUNCTION",
                    "Writeback conversion has no ORL function: "
                        + conversion.id.value);
                return false;
            }
            if (conversion.emitter
                != orlgraph::ConversionEmitterKind::OrlFunctionWriteback)
            {
                error(result, "ORL_LOWERING_WRITEBACK_EMITTER",
                    "Conversion is not a writable adapter: "
                        + conversion.id.value);
                return false;
            }
            const auto* target =
                find_auxiliary_input(module, conversion.output);
            if (target == nullptr) {
                error(result, "ORL_LOWERING_WRITEBACK_TARGET",
                    "Unable to resolve writeback target: "
                        + conversion.output.name);
                return false;
            }
            const auto target_name = input_names.find(target->id);
            if (target_name == input_names.end()) {
                error(result, "ORL_LOWERING_WRITEBACK_TARGET",
                    "Unable to lower writeback target: "
                        + conversion.output.name);
                return false;
            }
            if (selector.empty()) {
                error(result, "ORL_LOWERING_WRITEBACK_SELECTOR",
                    "Writeback selector expression is empty: "
                        + std::string{instance_key});
                return false;
            }
            source << "    " << conversion.implementation.function << "("
                   << target_name->second << ", " << selector << ", "
                   << temporary << ");\n";
            return true;
        };

    const auto writable_state_for =
        [&writable_adapters](const orlgraph::Endpoint& endpoint)
            -> const WritableAdapterState* {
            if (endpoint.kind != orlgraph::EndpointKind::NodePort) {
                return nullptr;
            }
            const auto found = writable_adapters.find(
                endpoint.owner.value + ":" + endpoint.port.value);
            return found == writable_adapters.end()
                ? nullptr : &found->second;
        };

    for (const auto& node_id : validation.schedule.order) {
        const auto* node = module.node(node_id);
        const auto* definition = node == nullptr
            ? nullptr : registry.find(node->definition);
        if (node == nullptr || definition == nullptr) {
            error(result, "ORL_LOWERING_NODE", "Scheduled node is unavailable");
            return result;
        }

        if (definition->implementation.kind
            != orlgraph::ImplementationKind::OrlFunction)
        {
            if (definition->operation == "identity"
                && !definition->outputs.empty())
            {
                const auto* input = definition->inputs.empty()
                    ? nullptr : connection_to(
                        module, node_id, definition->inputs.front().id);
                if (input == nullptr) {
                    error(result, "ORL_LOWERING_IDENTITY",
                        "Identity node has no input connection: " + node_id.value);
                    return result;
                }
                const std::string expression =
                    expression_for_endpoint(input->source);
                if (expression.empty()) {
                    error(result, "ORL_LOWERING_SOURCE",
                        "Unable to resolve identity input: " + node_id.value);
                    return result;
                }
                output_expressions[node_id.value + ":"
                    + definition->outputs.front().id.value] = expression;
            } else if (definition->implementation.kind
                    == orlgraph::ImplementationKind::Runtime
                && options.runtime_output_expression)
            {
                for (const auto& output : definition->outputs) {
                    const auto expression = options.runtime_output_expression(
                        *node, *definition, output);
                    if (expression.has_value()) {
                        output_expressions[node_id.value + ":"
                            + output.id.value] = *expression;
                    }
                }
            } else {
                bool used = false;
                for (const auto& output : definition->outputs) {
                    used = used || output_is_used(module, node_id, output);
                }
                if (used) {
                    error(result, "ORL_LOWERING_IMPLEMENTATION",
                        "Graph node is not lowerable as an ORL function: "
                            + node_id.value);
                    return result;
                }
            }
        } else {
            if (definition->implementation.function.empty()) {
                error(result, "ORL_LOWERING_FUNCTION",
                    "ORL node has no function name: " + node_id.value);
                return result;
            }

            std::vector<std::string> arguments;
            std::vector<std::string> pending_writebacks;
            for (const auto& port : definition->inputs) {
                const auto* connection = connection_to(module, node_id, port.id);
                if (connection == nullptr) {
                    if (port.default_value.has_value()) {
                        arguments.push_back(literal(*port.default_value));
                        continue;
                    }
                    error(result, "ORL_LOWERING_INPUT",
                        "Node input has no lowerable connection: " + port.name);
                    return result;
                }
                std::string expression =
                    expression_for_endpoint(connection->source);
                if (expression.empty()) {
                    error(result, "ORL_LOWERING_SOURCE",
                        "Unable to resolve node input source: " + port.name);
                    return result;
                }
                if (!connection->conversion.empty()) {
                    const auto* conversion = registry.find_conversion(
                        connection->conversion);
                    if (conversion == nullptr) {
                        error(result, "ORL_LOWERING_CONVERSION",
                            "Unknown connection conversion: "
                                + connection->conversion);
                        return result;
                    }
                    std::string converted;
                    if (!emit_conversion(*conversion, expression,
                            node_id.value + "_" + port.name, &converted))
                    {
                        return result;
                    }
                    expression = std::move(converted);
                }
                if (port.access != orlgraph::AccessMode::Read
                    && connection->source.kind
                        == orlgraph::EndpointKind::NodePort)
                {
                    const auto* writable =
                        writable_state_for(connection->source);
                    if (writable != nullptr) {
                        const std::string key =
                            connection->source.owner.value + ":"
                            + connection->source.port.value;
                        if (std::find(pending_writebacks.begin(),
                                pending_writebacks.end(), key)
                            == pending_writebacks.end())
                        {
                            pending_writebacks.push_back(key);
                        }
                    }
                }
                arguments.push_back(std::move(expression));
            }

            source << "    ";
            if (definition->outputs.empty()) {
                source << definition->implementation.function << "(";
            } else {
                const auto& output = definition->outputs.front();
                const std::string variable = "node_" + identifier(node_id.value)
                    + "_" + identifier(output.name);
                source << type_name(output.type) << " " << variable << " = "
                       << definition->implementation.function << "(";
                output_expressions[node_id.value + ":" + output.id.value] =
                    variable;
            }
            for (std::size_t index = 0; index < arguments.size(); ++index) {
                if (index != 0) {
                    source << ", ";
                }
                source << arguments[index];
            }
            source << ");\n";
            for (const auto& key : pending_writebacks) {
                const auto writable = writable_adapters.find(key);
                if (writable == writable_adapters.end()) {
                    continue;
                }
                const auto* conversion = registry.find_conversion(
                    writable->second.writeback_conversion);
                if (conversion == nullptr) {
                    error(result, "ORL_LOWERING_WRITEBACK_CONVERSION",
                        "Unknown writeback conversion: "
                            + writable->second.writeback_conversion.value);
                    return result;
                }
                if (!emit_writeback(*conversion,
                        writable->second.temporary,
                        writable->second.writeback_index,
                        key))
                {
                    return result;
                }
            }
        }

        for (const auto& output : definition->outputs) {
            if (!output.output_adapter.has_value()
                || !output_is_used(module, node_id, output))
            {
                continue;
            }
            const auto* conversion = registry.find_conversion(
                output.output_adapter->conversion);
            if (conversion == nullptr) {
                error(result, "ORL_LOWERING_CONVERSION",
                    "Unknown output conversion: "
                        + output.output_adapter->conversion.value);
                return result;
            }
            const auto source = output_expressions.find(
                node_id.value + ":" + output.output_adapter->source_port.value);
            if (source == output_expressions.end()) {
                error(result, "ORL_LOWERING_CONVERSION_SOURCE",
                    "Unable to resolve output adapter source: "
                        + output.output_adapter->source_port.value);
                return result;
            }
            std::string converted;
            if (!emit_conversion(*conversion, source->second,
                    node_id.value + "_" + output.name, &converted))
            {
                return result;
            }
            output_expressions[node_id.value + ":" + output.id.value] =
                converted;
            if (output.output_adapter->writeback_conversion.has_value()) {
                const auto writeback_source_port =
                    output.output_adapter->writeback_source_port.value_or(
                        output.output_adapter->source_port);
                const auto writeback_source = output_expressions.find(
                    node_id.value + ":"
                    + writeback_source_port.value);
                if (writeback_source == output_expressions.end()) {
                    error(result, "ORL_LOWERING_WRITEBACK_SOURCE",
                        "Unable to resolve writeback selector source: "
                            + writeback_source_port.value);
                    return result;
                }
                std::string writeback_index = source->second;
                if (writeback_source_port
                    != output.output_adapter->source_port
                    && options.runtime_handle_index_expression)
                {
                    const auto* selector_port =
                        definition->output(writeback_source_port.value);
                    const auto resolved = selector_port == nullptr
                        ? std::nullopt
                        : options.runtime_handle_index_expression(
                            *node, *definition, *selector_port,
                            writeback_source->second);
                    if (resolved.has_value()) {
                        writeback_index = *resolved;
                    }
                }
                writable_adapters[node_id.value + ":" + output.id.value] =
                    WritableAdapterState{
                        converted,
                        writeback_source->second,
                        writeback_index,
                        *output.output_adapter->writeback_conversion};
            }
        }
    }

    const auto parameter_for_expression =
        [&input_names](std::string_view expression) -> std::string {
        for (const auto& [id, name] : input_names) {
            (void)id;
            if (name == expression) {
                return name;
            }
        }
        return {};
    };

    bool returned = false;
    for (const auto& [output_id, output] : module.outputs()) {
        const auto* connection = connection_to_output(module, output_id);
        if (connection == nullptr) {
            continue;
        }
        std::string expression;
        if (connection->source.kind == orlgraph::EndpointKind::GraphInput) {
            expression = input_names[connection->source.owner];
        } else if (connection->source.kind == orlgraph::EndpointKind::NodePort) {
            expression = output_expressions[
                connection->source.owner.value + ":" + connection->source.port.value];
        }
        if (expression.empty()) {
            error(result, "ORL_LOWERING_OUTPUT",
                "Unable to resolve graph output: " + output.name);
            return result;
        }

        LoweredGraphOutput lowered_output;
        lowered_output.id = output.id;
        lowered_output.name = output.name;
        lowered_output.type = output.type;
        lowered_output.domain = output.domain;
        lowered_output.shape = output.shape;
        lowered_output.binding = output.binding;
        lowered_output.semantic = output.semantic;
        lowered_output.coordinate_space = output.coordinate_space;
        lowered_output.source_parameter =
            parameter_for_expression(expression);

        if (output.type.kind == orlgraph::LogicalTypeKind::Buffer) {
            if (lowered_output.source_parameter.empty()) {
                error(result, "ORL_LOWERING_RESULT",
                    "Buffer graph output '" + output.name
                    + "' must resolve to a bound graph-input parameter");
                return result;
            }
            result.outputs.push_back(std::move(lowered_output));
            continue;
        }
        if (output.type.kind == orlgraph::LogicalTypeKind::Float64) {
            if (lowered_output.source_parameter.empty()) {
                error(result, "ORL_LOWERING_RESULT",
                    "Float graph output '" + output.name
                    + "' must resolve to a bound graph-input parameter");
                return result;
            }
            result.outputs.push_back(std::move(lowered_output));
            continue;
        }
        if (output.type.kind != orlgraph::LogicalTypeKind::Int64) {
            error(result, "ORL_LOWERING_RESULT",
                "ORL graph output '" + output.name
                + "' has an unsupported scalar type");
            return result;
        }
        if (returned) {
            error(result, "ORL_LOWERING_RESULT",
                "ORL graph lowering supports one scalar int graph output");
            return result;
        }
        source << "    return " << expression << ";\n";
        returned = true;
        lowered_output.returned = true;
        result.outputs.push_back(std::move(lowered_output));
    }
    if (!returned) {
        source << "    return 0;\n";
    }
    source << "}\n";

    result.source = source.str();
    result.ok = result.diagnostics.empty();
    return result;
}

} // namespace orlcomp
