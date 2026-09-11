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

} // namespace

LoweredGraph OrlGraphLowerer::lower(const orlgraph::GraphModule& module,
    const orlgraph::NodeRegistry& registry, GraphLoweringOptions options) const
{
    LoweredGraph result;
    result.entry_function = options.entry_function;
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
        for (const auto& [_, node] : module.nodes()) {
            const auto* definition = registry.find(node.definition);
            if (definition != nullptr
                && definition->implementation.kind == orlgraph::ImplementationKind::OrlFunction
                && !definition->implementation.module.empty())
            {
                modules.insert(definition->implementation.module);
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
    for (const auto& node_id : validation.schedule.order) {
        const auto* node = module.node(node_id);
        const auto* definition = node == nullptr
            ? nullptr : registry.find(node->definition);
        if (node == nullptr || definition == nullptr) {
            error(result, "ORL_LOWERING_NODE", "Scheduled node is unavailable");
            return result;
        }
        if (definition->implementation.kind != orlgraph::ImplementationKind::OrlFunction) {
            if (definition->operation == "identity" && !definition->outputs.empty()) {
                const auto* input = definition->inputs.empty()
                    ? nullptr : connection_to(module, node_id, definition->inputs.front().id);
                if (input == nullptr) {
                    error(result, "ORL_LOWERING_IDENTITY",
                        "Identity node has no input connection: " + node_id.value);
                    return result;
                }
                std::string expression;
                if (input->source.kind == orlgraph::EndpointKind::GraphInput) {
                    expression = input_names[input->source.owner];
                } else if (input->source.kind == orlgraph::EndpointKind::NodePort) {
                    expression = output_expressions[input->source.owner.value + ":"
                        + input->source.port.value];
                }
                output_expressions[node_id.value + ":"
                    + definition->outputs.front().id.value] = expression;
                continue;
            }
            error(result, "ORL_LOWERING_IMPLEMENTATION",
                "Graph node is not lowerable as an ORL function: " + node_id.value);
            return result;
        }
        if (definition->implementation.function.empty()) {
            error(result, "ORL_LOWERING_FUNCTION",
                "ORL node definition has no function name: " + node_id.value);
            return result;
        }

        std::vector<std::string> arguments;
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
            std::string expression;
            if (connection->source.kind == orlgraph::EndpointKind::GraphInput) {
                const auto found = input_names.find(connection->source.owner);
                if (found != input_names.end()) {
                    expression = found->second;
                }
            } else if (connection->source.kind == orlgraph::EndpointKind::NodePort) {
                expression = output_expressions[
                    connection->source.owner.value + ":" + connection->source.port.value];
            }
            if (expression.empty()) {
                error(result, "ORL_LOWERING_SOURCE",
                    "Unable to resolve node input source: " + port.name);
                return result;
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
            output_expressions[node_id.value + ":" + output.id.value] = variable;
        }
        for (std::size_t index = 0; index < arguments.size(); ++index) {
            if (index != 0) {
                source << ", ";
            }
            source << arguments[index];
        }
        source << ");\n";
    }

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
        if (output.type.kind != orlgraph::LogicalTypeKind::Int64) {
            error(result, "ORL_LOWERING_RESULT",
                "Initial ORL graph lowering requires an int graph output");
            return result;
        }
        source << "    return " << expression << ";\n";
        returned = true;
        break;
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
