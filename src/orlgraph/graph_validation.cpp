#include "graph_validation.hpp"

#include <map>
#include <set>
#include <utility>

namespace orlgraph
{

namespace
{

const Port* find_port(const NodeDefinition* definition,
    StableId port_id, bool input)
{
    if (definition == nullptr) {
        return nullptr;
    }
    const auto& ports = input ? definition->inputs : definition->outputs;
    for (const auto& port : ports) {
        if (port.id == port_id || port.name == port_id.value) {
            return &port;
        }
    }
    return nullptr;
}

struct EndpointInfo {
    const Port* port = nullptr;
    LogicalType type;
    Domain domain = Domain::constant();
    Shape shape = Shape::scalar();
    PortDirection direction = PortDirection::Input;
    bool valid = false;
};

EndpointInfo endpoint_info(const GraphModule& module,
    const NodeRegistry& registry, const Endpoint& endpoint, bool source)
{
    EndpointInfo result;
    if (endpoint.kind == EndpointKind::NodePort) {
        const auto* node = module.node(endpoint.owner);
        const auto* definition = node == nullptr ? nullptr : registry.find(node->definition);
        result.port = find_port(definition, endpoint.port, !source);
        if (result.port == nullptr) {
            return result;
        }
        result.type = result.port->type;
        result.domain = result.port->domain;
        result.shape = result.port->shape;
        result.direction = result.port->direction;
        result.valid = true;
        return result;
    }

    if (source && endpoint.kind == EndpointKind::GraphInput) {
        const auto* input = module.input(endpoint.owner);
        if (input == nullptr) {
            return result;
        }
        result.type = input->type;
        result.domain = input->domain;
        result.shape = input->shape;
        result.direction = PortDirection::Output;
        result.valid = true;
        return result;
    }

    if (!source && endpoint.kind == EndpointKind::GraphOutput) {
        const auto* output = module.output(endpoint.owner);
        if (output == nullptr) {
            return result;
        }
        result.type = output->type;
        result.domain = output->domain;
        result.shape = output->shape;
        result.direction = PortDirection::Input;
        result.valid = true;
        return result;
    }

    return result;
}

bool shape_compatible(const Shape& source, const Shape& destination) {
    return source == destination
        || source.is_scalar()
        || destination.is_scalar();
}

} // namespace

bool ValidationResult::ok() const {
    if (!schedule.ok) {
        return false;
    }
    for (const auto& diagnostic : diagnostics) {
        if (diagnostic.severity == DiagnosticSeverity::Error) {
            return false;
        }
    }
    return true;
}

void ValidationResult::error(std::string code, std::string message,
    StableId node, StableId port)
{
    diagnostics.push_back(Diagnostic{
        DiagnosticSeverity::Error,
        std::move(code),
        std::move(message),
        std::move(node),
        std::move(port),
        {},
    });
}

ValidationResult validate(const GraphModule& module, const NodeRegistry& registry) {
    ValidationResult result;
    std::map<StableId, std::set<StableId>> connected_inputs;
    std::map<StableId, std::size_t> output_connections;

    for (const auto& [node_id, node] : module.nodes()) {
        const auto* definition = registry.find(node.definition);
        if (definition == nullptr) {
            result.error("ORLGRAPH_UNKNOWN_DEFINITION",
                "Node references an unknown definition: " + node.definition.value,
                node_id);
            continue;
        }

        std::set<StableId> port_ids;
        for (const auto& port : definition->inputs) {
            if (!port_ids.insert(port.id).second) {
                result.error("ORLGRAPH_DUPLICATE_PORT",
                    "Duplicate input port: " + port.id.value, node_id, port.id);
            }
        }
        for (const auto& port : definition->outputs) {
            if (!port_ids.insert(port.id).second) {
                result.error("ORLGRAPH_DUPLICATE_PORT",
                    "Duplicate port: " + port.id.value, node_id, port.id);
            }
        }

        for (const auto& [name, value] : node.parameter_values) {
            const auto* parameter = definition->parameter(name);
            if (parameter == nullptr) {
                result.error("ORLGRAPH_UNKNOWN_PARAMETER",
                    "Node parameter is not declared: " + name, node_id);
            } else if (parameter->type != value.type) {
                result.error("ORLGRAPH_PARAMETER_TYPE",
                    "Node parameter has incompatible type: " + name, node_id);
            }
        }

        std::set<std::string> mapped_parameters;
        for (const auto& mapping : node.parameter_mappings) {
            const auto* parameter = definition->parameter(mapping.parameter);
            if (parameter == nullptr) {
                result.error("ORLGRAPH_UNKNOWN_PARAMETER",
                    "Parameter mapping references an undeclared parameter: "
                        + mapping.parameter, node_id);
                continue;
            }
            if (!mapped_parameters.insert(mapping.parameter).second) {
                result.error("ORLGRAPH_DUPLICATE_PARAMETER_MAPPING",
                    "Parameter has more than one mapping: " + mapping.parameter,
                    node_id);
            }
            if (mapping.source_kind == ParameterSourceKind::Constant) {
                if (!mapping.constant.has_value()) {
                    result.error("ORLGRAPH_MISSING_CONSTANT",
                        "Constant parameter mapping has no value: "
                            + mapping.parameter, node_id);
                } else if (mapping.constant->type != parameter->type) {
                    result.error("ORLGRAPH_PARAMETER_TYPE",
                        "Constant parameter mapping has an incompatible type: "
                            + mapping.parameter, node_id);
                }
                continue;
            }
            if (mapping.source_kind == ParameterSourceKind::GraphInput
                || mapping.source_kind == ParameterSourceKind::NodeOutput)
            {
                const auto source = endpoint_info(module, registry, mapping.source, true);
                if (!source.valid) {
                    result.error("ORLGRAPH_INVALID_PARAMETER_SOURCE",
                        "Parameter mapping source does not resolve: "
                            + mapping.parameter, node_id);
                } else if (source.type != parameter->type
                    && mapping.conversion.empty())
                {
                    result.error("ORLGRAPH_PARAMETER_TYPE",
                        "Parameter mapping requires an explicit conversion: "
                            + mapping.parameter, node_id);
                }
                continue;
            }
            if (mapping.source_kind == ParameterSourceKind::ResourceAttribute) {
                if (mapping.source.owner.empty()
                    || module.resource(mapping.source.owner) == nullptr
                    || mapping.resource_attribute.empty())
                {
                    result.error("ORLGRAPH_INVALID_RESOURCE_MAPPING",
                        "Resource attribute mapping is incomplete: "
                            + mapping.parameter, node_id);
                }
            }
        }
    }

    for (const auto& [id, input] : module.inputs()) {
        if (input.direction != PortDirection::Input) {
            result.error("ORLGRAPH_INTERFACE_DIRECTION",
                "Graph input has invalid direction", {}, id);
        }
    }
    for (const auto& [id, output] : module.outputs()) {
        if (output.direction != PortDirection::Output) {
            result.error("ORLGRAPH_INTERFACE_DIRECTION",
                "Graph output has invalid direction", {}, id);
        }
    }

    for (const auto& connection : module.connections()) {
        const auto source = endpoint_info(module, registry, connection.source, true);
        const auto destination = endpoint_info(module, registry, connection.destination, false);
        if (!source.valid) {
            result.error("ORLGRAPH_INVALID_SOURCE",
                "Connection source does not resolve");
            continue;
        }
        if (!destination.valid) {
            result.error("ORLGRAPH_INVALID_DESTINATION",
                "Connection destination does not resolve");
            continue;
        }

        if (connection.destination.kind == EndpointKind::NodePort) {
            const auto key = connection.destination.owner;
            if (!connected_inputs[key].insert(connection.destination.port).second) {
                result.error("ORLGRAPH_MULTIPLE_INPUTS",
                    "Input port has more than one connection",
                    connection.destination.owner, connection.destination.port);
            }
        }
        if (connection.destination.kind == EndpointKind::GraphOutput) {
            ++output_connections[connection.destination.owner];
        }

        if (connection.feedback) {
            const auto* source_node = connection.source.kind == EndpointKind::NodePort
                ? module.node(connection.source.owner) : nullptr;
            const auto* destination_node = connection.destination.kind == EndpointKind::NodePort
                ? module.node(connection.destination.owner) : nullptr;
            const auto* source_definition = source_node == nullptr
                ? nullptr : registry.find(source_node->definition);
            const auto* destination_definition = destination_node == nullptr
                ? nullptr : registry.find(destination_node->definition);
            const auto feedback_capable = [](const NodeDefinition* definition) {
                return definition != nullptr
                    && (definition->stateful
                        || definition->operation == "feedback"
                        || definition->operation == "solver");
            };
            if (!feedback_capable(source_definition)
                && !feedback_capable(destination_definition))
            {
                result.error("ORLGRAPH_INVALID_FEEDBACK",
                    "Feedback edges must connect to a stateful or solver node");
            }
        }

        if (source.type != destination.type && connection.conversion.empty()) {
            result.error("ORLGRAPH_TYPE_MISMATCH",
                "Connection requires an explicit conversion");
        }
        if (!shape_compatible(source.shape, destination.shape)
            && connection.conversion.empty())
        {
            result.error("ORLGRAPH_SHAPE_MISMATCH",
                "Connection shapes are incompatible");
        }
        if (source.domain != destination.domain
            && source.domain.kind != DomainKind::Constant
            && destination.domain.kind != DomainKind::Constant
            && connection.conversion.empty())
        {
            result.error("ORLGRAPH_DOMAIN_MISMATCH",
                "Connection domains are incompatible");
        }
    }

    for (const auto& [id, output] : module.outputs()) {
        if (output_connections[id] == 0 && output.required) {
            result.error("ORLGRAPH_UNCONNECTED_OUTPUT",
                "Required graph output has no connection", {}, id);
        } else if (output_connections[id] > 1) {
            result.error("ORLGRAPH_MULTIPLE_OUTPUTS",
                "Graph output has more than one connection", {}, id);
        }
    }

    for (const auto& [node_id, node] : module.nodes()) {
        const auto* definition = registry.find(node.definition);
        if (definition == nullptr) {
            continue;
        }
        for (const auto& port : definition->inputs) {
            const bool has_default = port.default_value.has_value();
            if (port.required && !has_default
                && !connected_inputs[node_id].contains(port.id))
            {
                result.error("ORLGRAPH_UNCONNECTED_INPUT",
                    "Required node input has no connection: " + port.name,
                    node_id, port.id);
            }
        }
    }

    result.schedule = topological_schedule(module);
    for (const auto& error : result.schedule.errors) {
        result.error("ORLGRAPH_SCHEDULE", error);
    }
    return result;
}

} // namespace orlgraph
