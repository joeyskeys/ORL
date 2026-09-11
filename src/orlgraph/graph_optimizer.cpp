#include "graph_optimizer.hpp"

#include <algorithm>
#include <map>
#include <set>
#include <sstream>
#include <tuple>
#include <unordered_map>
#include <utility>

namespace orlgraph
{

namespace
{

void add_warning(std::vector<Diagnostic>& diagnostics,
    std::string code, std::string message)
{
    diagnostics.push_back(Diagnostic{
        DiagnosticSeverity::Warning,
        std::move(code),
        std::move(message),
        {},
        {},
        {},
    });
}

std::string value_key(const ConstantValue& value) {
    std::ostringstream stream;
    stream << value.type.canonical_name() << "=";
    std::visit([&stream](const auto& item) {
        using T = std::decay_t<decltype(item)>;
        if constexpr (std::is_same_v<T, std::monostate>) {
            stream << "null";
        } else if constexpr (std::is_same_v<T, std::vector<std::int64_t>>
            || std::is_same_v<T, std::vector<double>>)
        {
            stream << "[";
            for (const auto& element : item) {
                stream << element << ",";
            }
            stream << "]";
        } else {
            stream << item;
        }
    }, value.value);
    return stream.str();
}

std::string endpoint_key(const Endpoint& endpoint) {
    return std::to_string(static_cast<int>(endpoint.kind))
        + ":" + endpoint.owner.value + ":" + endpoint.port.value;
}

bool has_write_effect(const NodeDefinition& definition) {
    if (!definition.pure || definition.stateful) {
        return true;
    }
    return std::any_of(definition.effects.begin(), definition.effects.end(),
        [](const ResourceEffect& effect) {
            return effect.observable
                || effect.access == AccessMode::Write
                || effect.access == AccessMode::ReadWrite;
        });
}

void remove_duplicate_connections(GraphModule& module) {
    std::set<std::string> seen;
    auto& connections = module.mutable_connections();
    connections.erase(std::remove_if(connections.begin(), connections.end(),
        [&seen](const Connection& connection) {
            const std::string key = endpoint_key(connection.source) + "->"
                + endpoint_key(connection.destination) + ":" + connection.conversion
                + ":" + (connection.feedback ? "feedback" : "normal");
            return !seen.insert(key).second;
        }), connections.end());
}

void redirect_node(GraphModule& module, const StableId& from, const StableId& to) {
    for (auto& connection : module.mutable_connections()) {
        if (connection.source.kind == EndpointKind::NodePort
            && connection.source.owner == from)
        {
            connection.source.owner = to;
        }
        if (connection.destination.kind == EndpointKind::NodePort
            && connection.destination.owner == from)
        {
            connection.destination.owner = to;
        }
    }
    remove_duplicate_connections(module);
}

void specialize_constants(GraphModule& module) {
    for (auto& [_, node] : module.mutable_nodes()) {
        for (const auto& mapping : node.parameter_mappings) {
            if (mapping.source_kind == ParameterSourceKind::Constant
                && mapping.constant.has_value())
            {
                node.parameter_values[mapping.parameter] = *mapping.constant;
            }
        }
        node.parameter_mappings.erase(
            std::remove_if(node.parameter_mappings.begin(), node.parameter_mappings.end(),
                [](const ParameterMapping& mapping) {
                    return mapping.source_kind == ParameterSourceKind::Constant
                        && mapping.constant.has_value();
                }),
            node.parameter_mappings.end());
    }
}

void simplify_identity(GraphModule& module, const NodeRegistry& registry,
    std::vector<StableId>* removed)
{
    std::vector<StableId> identities;
    for (const auto& [id, node] : module.nodes()) {
        const auto* definition = registry.find(node.definition);
        if (definition != nullptr && definition->operation == "identity") {
            identities.push_back(id);
        }
    }

    for (const auto& identity : identities) {
        const auto node_it = module.nodes().find(identity);
        if (node_it == module.nodes().end()) {
            continue;
        }
        Endpoint source;
        bool found = false;
        for (const auto& connection : module.connections()) {
            if (connection.destination.kind == EndpointKind::NodePort
                && connection.destination.owner == identity)
            {
                source = connection.source;
                found = true;
                break;
            }
        }
        if (!found) {
            continue;
        }
        for (auto& connection : module.mutable_connections()) {
            if (connection.source.kind == EndpointKind::NodePort
                && connection.source.owner == identity)
            {
                connection.source = source;
            }
        }
        auto& connections = module.mutable_connections();
        connections.erase(std::remove_if(connections.begin(), connections.end(),
            [&identity](const Connection& connection) {
                return (connection.source.kind == EndpointKind::NodePort
                        && connection.source.owner == identity)
                    || (connection.destination.kind == EndpointKind::NodePort
                        && connection.destination.owner == identity);
            }), connections.end());
        module.mutable_nodes().erase(identity);
        removed->push_back(identity);
        remove_duplicate_connections(module);
    }
}

void eliminate_common_nodes(GraphModule& module, const NodeRegistry& registry,
    std::vector<StableId>* merged)
{
    std::map<std::string, StableId> canonical;
    std::vector<std::pair<StableId, StableId>> replacements;

    for (const auto& [id, node] : module.nodes()) {
        const auto* definition = registry.find(node.definition);
        if (definition == nullptr || !definition->pure || has_write_effect(*definition)) {
            continue;
        }
        std::ostringstream signature;
        signature << node.definition.value << "|";
        for (const auto& [name, value] : node.parameter_values) {
            signature << name << "=" << value_key(value) << ";";
        }
        for (const auto& connection : module.connections()) {
            if (connection.destination.kind == EndpointKind::NodePort
                && connection.destination.owner == id)
            {
                signature << connection.destination.port.value << "<-"
                    << endpoint_key(connection.source)
                    << (connection.feedback ? ":feedback" : ":normal") << ";";
            }
        }
        const auto [found, inserted] = canonical.emplace(signature.str(), id);
        if (!inserted) {
            replacements.emplace_back(id, found->second);
        }
    }

    for (const auto& [duplicate, original] : replacements) {
        redirect_node(module, duplicate, original);
        module.mutable_nodes().erase(duplicate);
        merged->push_back(duplicate);
    }
}

void eliminate_dead_nodes(GraphModule& module, const NodeRegistry& registry,
    std::vector<StableId>* removed)
{
    std::set<StableId> live;
    std::vector<StableId> work;

    for (const auto& connection : module.connections()) {
        if (connection.destination.kind == EndpointKind::GraphOutput
            && connection.source.kind == EndpointKind::NodePort)
        {
            if (live.insert(connection.source.owner).second) {
                work.push_back(connection.source.owner);
            }
        }
    }
    for (const auto& [id, node] : module.nodes()) {
        const auto* definition = registry.find(node.definition);
        if (definition != nullptr && has_write_effect(*definition)) {
            if (live.insert(id).second) {
                work.push_back(id);
            }
        }
    }

    while (!work.empty()) {
        const StableId current = work.back();
        work.pop_back();
        for (const auto& connection : module.connections()) {
            if (connection.destination.kind == EndpointKind::NodePort
                && connection.destination.owner == current
                && connection.source.kind == EndpointKind::NodePort
                && live.insert(connection.source.owner).second)
            {
                work.push_back(connection.source.owner);
            }
        }
    }

    std::vector<StableId> dead;
    for (const auto& [id, _] : module.nodes()) {
        if (!live.contains(id)) {
            dead.push_back(id);
        }
    }
    for (const auto& id : dead) {
        module.mutable_nodes().erase(id);
        removed->push_back(id);
    }
    if (!dead.empty()) {
        auto& connections = module.mutable_connections();
        connections.erase(std::remove_if(connections.begin(), connections.end(),
            [&module](const Connection& connection) {
                if (connection.source.kind == EndpointKind::NodePort
                    && module.node(connection.source.owner) == nullptr)
                {
                    return true;
                }
                return connection.destination.kind == EndpointKind::NodePort
                    && module.node(connection.destination.owner) == nullptr;
            }), connections.end());
    }
}

const InterfacePort* matching_interface(
    const std::map<StableId, InterfacePort>& ports, const Port& definition_port)
{
    for (const auto& [_, port] : ports) {
        if (port.id == definition_port.id || port.name == definition_port.name) {
            return &port;
        }
    }
    return nullptr;
}

const InterfacePort* matching_interface(
    const std::map<StableId, InterfacePort>& ports, const StableId& id)
{
    const auto found = ports.find(id);
    return found == ports.end() ? nullptr : &found->second;
}

bool flatten_subgraph_node(GraphModule& module, const NodeRegistry& registry,
    const NodeInstance& instance, const NodeDefinition& definition,
    std::vector<StableId>* removed, std::vector<Diagnostic>* diagnostics)
{
    const auto* nested = registry.find_subgraph(definition.implementation.subgraph);
    if (nested == nullptr) {
        add_warning(*diagnostics, "ORLGRAPH_MISSING_SUBGRAPH",
            "Subgraph definition has no registered graph: "
                + definition.implementation.subgraph.value);
        return false;
    }

    std::map<StableId, Endpoint> input_sources;
    std::map<StableId, std::vector<Endpoint>> output_targets;
    std::vector<Connection> retained;
    for (const auto& connection : module.connections()) {
        const bool touches_destination =
            connection.destination.kind == EndpointKind::NodePort
            && connection.destination.owner == instance.id;
        const bool touches_source =
            connection.source.kind == EndpointKind::NodePort
            && connection.source.owner == instance.id;
        if (touches_destination) {
            input_sources[connection.destination.port] = connection.source;
        }
        if (touches_source) {
            output_targets[connection.source.port].push_back(connection.destination);
        }
        if (!touches_destination && !touches_source) {
            retained.push_back(connection);
        }
    }

    const std::string prefix = instance.id.value + "/";
    std::vector<NodeInstance> added_nodes;
    for (const auto& [nested_id, nested_node] : nested->nodes()) {
        NodeInstance clone = nested_node;
        clone.id = StableId{prefix + nested_id.value};
        clone.provenance.source_nodes.push_back(instance.id);
        added_nodes.push_back(std::move(clone));
    }

    std::vector<Connection> added_connections;
    for (const auto& nested_connection : nested->connections()) {
        std::vector<Endpoint> sources;
        const Endpoint& nested_source = nested_connection.source;
        if (nested_source.kind == EndpointKind::NodePort) {
            sources.push_back(Endpoint::node_port(
                StableId{prefix + nested_source.owner.value},
                nested_source.port));
        } else if (nested_source.kind == EndpointKind::GraphInput) {
            const auto* nested_input = matching_interface(
                nested->inputs(), nested_source.owner);
            if (nested_input == nullptr) {
                continue;
            }
            const auto* outer_port = definition.input(nested_input->name);
            if (outer_port == nullptr) {
                outer_port = definition.input(nested_input->id.value);
            }
            if (outer_port == nullptr) {
                continue;
            }
            const auto source = input_sources.find(outer_port->id);
            if (source == input_sources.end()) {
                continue;
            }
            sources.push_back(source->second);
        } else {
            continue;
        }

        std::vector<Endpoint> destinations;
        const Endpoint& nested_destination = nested_connection.destination;
        if (nested_destination.kind == EndpointKind::NodePort) {
            destinations.push_back(Endpoint::node_port(
                StableId{prefix + nested_destination.owner.value},
                nested_destination.port));
        } else if (nested_destination.kind == EndpointKind::GraphOutput) {
            const auto* nested_output = matching_interface(
                nested->outputs(), nested_destination.owner);
            if (nested_output == nullptr) {
                continue;
            }
            const auto* outer_port = definition.output(nested_output->name);
            if (outer_port == nullptr) {
                outer_port = definition.output(nested_output->id.value);
            }
            if (outer_port == nullptr) {
                continue;
            }
            const auto targets = output_targets.find(outer_port->id);
            if (targets == output_targets.end()) {
                continue;
            }
            destinations = targets->second;
        } else {
            continue;
        }

        for (const auto& source : sources) {
            for (const auto& destination : destinations) {
                Connection connection = nested_connection;
                connection.source = source;
                connection.destination = destination;
                connection.provenance.source_nodes.push_back(instance.id);
                added_connections.push_back(std::move(connection));
            }
        }
    }

    module.mutable_connections() = std::move(retained);
    module.mutable_nodes().erase(instance.id);
    for (auto& node : added_nodes) {
        std::string error;
        if (!module.add_node(std::move(node), &error)) {
            diagnostics->push_back({
                DiagnosticSeverity::Error,
                "ORLGRAPH_FLATTEN_NODE",
                std::move(error),
                instance.id,
                {},
                instance.provenance,
            });
            return false;
        }
    }
    for (auto& connection : added_connections) {
        module.add_connection(std::move(connection));
    }
    removed->push_back(instance.id);
    remove_duplicate_connections(module);
    return true;
}

void flatten_subgraphs(GraphModule& module, const NodeRegistry& registry,
    std::vector<StableId>* removed, std::vector<Diagnostic>* diagnostics)
{
    bool changed = true;
    std::size_t iterations = 0;
    while (changed && iterations++ < 64) {
        changed = false;
        std::vector<std::tuple<NodeInstance, NodeDefinition>> candidates;
        for (const auto& [_, node] : module.nodes()) {
            const auto* definition = registry.find(node.definition);
            if (definition != nullptr
                && definition->implementation.kind == ImplementationKind::Subgraph)
            {
                candidates.emplace_back(node, *definition);
            }
        }
        for (const auto& [node, definition] : candidates) {
            if (module.node(node.id) != nullptr
                && flatten_subgraph_node(module, registry, node, definition,
                    removed, diagnostics))
            {
                changed = true;
            }
        }
    }
    if (iterations >= 64) {
        add_warning(*diagnostics, "ORLGRAPH_FLATTEN_LIMIT",
            "Subgraph flattening stopped at the nesting limit");
    }
}

} // namespace

GraphOptimizer::GraphOptimizer(OptimizationOptions options)
    : options_(options)
{
}

OptimizationResult GraphOptimizer::optimize(GraphModule& module,
    const NodeRegistry& registry) const
{
    OptimizationResult result;
    const ValidationResult initial = validate(module, registry);
    result.diagnostics = initial.diagnostics;
    if (!initial.ok()) {
        result.schedule = initial.schedule;
        result.ok = false;
        return result;
    }

    if (options_.specialize_constants) {
        specialize_constants(module);
    }
    flatten_subgraphs(module, registry, &result.removed_nodes, &result.diagnostics);
    if (options_.simplify_identity) {
        simplify_identity(module, registry, &result.removed_nodes);
    }
    if (options_.eliminate_common_subgraphs) {
        eliminate_common_nodes(module, registry, &result.merged_nodes);
    }
    if (options_.eliminate_dead_nodes) {
        eliminate_dead_nodes(module, registry, &result.removed_nodes);
    }

    bool has_subgraph = false;
    for (const auto& [_, node] : module.nodes()) {
        const auto* definition = registry.find(node.definition);
        has_subgraph = has_subgraph || (definition != nullptr
            && definition->implementation.kind == ImplementationKind::Subgraph);
    }
    if (has_subgraph) {
        add_warning(result.diagnostics, "ORLGRAPH_SUBGRAPH_OPAQUE",
            "Subgraph definitions remain opaque until a subgraph registry is provided");
    }

    const ValidationResult final = validate(module, registry);
    result.diagnostics.insert(result.diagnostics.end(),
        final.diagnostics.begin(), final.diagnostics.end());
    result.schedule = final.schedule;
    result.ok = final.ok();
    return result;
}

} // namespace orlgraph
