#include "graph_ir.hpp"

#include <algorithm>
#include <functional>
#include <utility>

namespace orlgraph
{

namespace
{

bool set_error(std::string* error, std::string message) {
    if (error != nullptr) {
        *error = std::move(message);
    }
    return false;
}

} // namespace

bool Port::compatible_value(const Port& source) const {
    if (direction == PortDirection::Output || source.direction == PortDirection::Input) {
        return false;
    }
    if (type != source.type) {
        return false;
    }
    if (cardinality != source.cardinality
        && !(cardinality == PortCardinality::Buffer
             && source.cardinality == PortCardinality::Array))
    {
        return false;
    }
    return domain == source.domain || domain.kind == DomainKind::Constant
        || source.domain.kind == DomainKind::Constant;
}

const Port* NodeDefinition::input(std::string_view port_name) const {
    for (const auto& port : inputs) {
        if (port.name == port_name || port.id.value == port_name) {
            return &port;
        }
    }
    return nullptr;
}

const Port* NodeDefinition::output(std::string_view port_name) const {
    for (const auto& port : outputs) {
        if (port.name == port_name || port.id.value == port_name) {
            return &port;
        }
    }
    return nullptr;
}

const ParameterSpec* NodeDefinition::parameter(std::string_view parameter_name) const {
    for (const auto& parameter : parameters) {
        if (parameter.name == parameter_name || parameter.id.value == parameter_name) {
            return &parameter;
        }
    }
    return nullptr;
}

bool NodeRegistry::register_definition(NodeDefinition definition, std::string* error) {
    if (definition.id.empty()) {
        return set_error(error, "Node definition ID is empty");
    }
    if (definition.qualified_name.empty()) {
        return set_error(error, "Node definition name is empty");
    }
    if (values_.contains(definition.id)) {
        return set_error(error, "Duplicate node definition: " + definition.id.value);
    }
    values_.emplace(definition.id, std::move(definition));
    return true;
}

const NodeDefinition* NodeRegistry::find(const StableId& id) const {
    const auto found = values_.find(id);
    return found == values_.end() ? nullptr : &found->second;
}

const NodeDefinition* NodeRegistry::find(std::string_view qualified_name) const {
    for (const auto& [_, definition] : values_) {
        if (definition.qualified_name == qualified_name) {
            return &definition;
        }
    }
    return nullptr;
}

bool NodeRegistry::register_subgraph(StableId id, GraphModule graph,
    std::string* error)
{
    if (id.empty()) {
        return set_error(error, "Subgraph ID is empty");
    }
    if (subgraphs_.contains(id)) {
        return set_error(error, "Duplicate subgraph: " + id.value);
    }
    subgraphs_.emplace(std::move(id),
        std::make_shared<GraphModule>(std::move(graph)));
    return true;
}

const GraphModule* NodeRegistry::find_subgraph(const StableId& id) const {
    const auto found = subgraphs_.find(id);
    return found == subgraphs_.end() ? nullptr : found->second.get();
}

Endpoint Endpoint::node_port(StableId node, StableId port) {
    return Endpoint{EndpointKind::NodePort, std::move(node), std::move(port)};
}

Endpoint Endpoint::graph_input(StableId input) {
    return Endpoint{EndpointKind::GraphInput, std::move(input), {}};
}

Endpoint Endpoint::graph_output(StableId output) {
    return Endpoint{EndpointKind::GraphOutput, std::move(output), {}};
}

bool GraphModule::add_node(NodeInstance node_value, std::string* error) {
    if (node_value.id.empty()) {
        return set_error(error, "Graph node ID is empty");
    }
    if (node_value.definition.empty()) {
        return set_error(error, "Graph node has no definition: " + node_value.id.value);
    }
    if (nodes_.contains(node_value.id)) {
        return set_error(error, "Duplicate graph node: " + node_value.id.value);
    }
    nodes_.emplace(node_value.id, std::move(node_value));
    return true;
}

bool GraphModule::add_connection(Connection connection, std::string* error) {
    if (connection.source.owner.empty() || connection.destination.owner.empty()) {
        return set_error(error, "Graph connection endpoint is empty");
    }
    connections_.push_back(std::move(connection));
    return true;
}

bool GraphModule::add_input(InterfacePort input_value, std::string* error) {
    if (input_value.id.empty()) {
        return set_error(error, "Graph input ID is empty");
    }
    if (input_value.direction != PortDirection::Input) {
        return set_error(error, "Graph input must have input direction");
    }
    if (inputs_.contains(input_value.id)) {
        return set_error(error, "Duplicate graph input: " + input_value.id.value);
    }
    inputs_.emplace(input_value.id, std::move(input_value));
    return true;
}

bool GraphModule::add_output(InterfacePort output_value, std::string* error) {
    if (output_value.id.empty()) {
        return set_error(error, "Graph output ID is empty");
    }
    if (output_value.direction != PortDirection::Output) {
        return set_error(error, "Graph output must have output direction");
    }
    if (outputs_.contains(output_value.id)) {
        return set_error(error, "Duplicate graph output: " + output_value.id.value);
    }
    outputs_.emplace(output_value.id, std::move(output_value));
    return true;
}

bool GraphModule::add_resource(Resource resource_value, std::string* error) {
    if (resource_value.id.empty()) {
        return set_error(error, "Graph resource ID is empty");
    }
    if (resources_.contains(resource_value.id)) {
        return set_error(error, "Duplicate graph resource: " + resource_value.id.value);
    }
    resources_.emplace(resource_value.id, std::move(resource_value));
    return true;
}

bool GraphModule::remove_input(const StableId& id, std::string* error) {
    if (!inputs_.contains(id)) {
        return set_error(error, "Unknown graph input: " + id.value);
    }
    connections_.erase(std::remove_if(connections_.begin(), connections_.end(),
        [&id](const Connection& connection) {
            return connection.source.kind == EndpointKind::GraphInput
                && connection.source.owner == id;
        }), connections_.end());
    for (auto& [_, node] : nodes_) {
        node.parameter_mappings.erase(
            std::remove_if(node.parameter_mappings.begin(),
                node.parameter_mappings.end(),
                [&id](const ParameterMapping& mapping) {
                    return mapping.source_kind == ParameterSourceKind::GraphInput
                        && mapping.source.owner == id;
                }),
            node.parameter_mappings.end());
    }
    inputs_.erase(id);
    return true;
}

bool GraphModule::remove_output(const StableId& id, std::string* error) {
    if (!outputs_.contains(id)) {
        return set_error(error, "Unknown graph output: " + id.value);
    }
    connections_.erase(std::remove_if(connections_.begin(), connections_.end(),
        [&id](const Connection& connection) {
            return connection.destination.kind == EndpointKind::GraphOutput
                && connection.destination.owner == id;
        }), connections_.end());
    outputs_.erase(id);
    return true;
}

const NodeInstance* GraphModule::node(const StableId& id) const {
    const auto found = nodes_.find(id);
    return found == nodes_.end() ? nullptr : &found->second;
}

const InterfacePort* GraphModule::input(const StableId& id) const {
    const auto found = inputs_.find(id);
    return found == inputs_.end() ? nullptr : &found->second;
}

const InterfacePort* GraphModule::output(const StableId& id) const {
    const auto found = outputs_.find(id);
    return found == outputs_.end() ? nullptr : &found->second;
}

const Resource* GraphModule::resource(const StableId& id) const {
    const auto found = resources_.find(id);
    return found == resources_.end() ? nullptr : &found->second;
}

} // namespace orlgraph
