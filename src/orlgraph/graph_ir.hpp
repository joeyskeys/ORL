#pragma once

#include "graph_ids.hpp"
#include "graph_types.hpp"

#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace orlgraph
{

enum class PortDirection : std::uint8_t {
    Input,
    Output,
    InOut,
};

enum class PortCardinality : std::uint8_t {
    Scalar,
    Array,
    Buffer,
};

struct SourceLocation {
    std::string file;
    std::uint32_t line = 0;
    std::uint32_t column = 0;
    std::uint32_t end_line = 0;
    std::uint32_t end_column = 0;

    friend bool operator==(const SourceLocation&, const SourceLocation&) = default;
};

struct Provenance {
    std::vector<SourceLocation> locations;
    std::vector<StableId> source_nodes;
    std::string description;
};

enum class AccessMode : std::uint8_t {
    Read,
    Write,
    ReadWrite,
};

struct ResourceEffect {
    StableId resource;
    AccessMode access = AccessMode::Read;
    bool observable = false;

    friend bool operator==(const ResourceEffect&, const ResourceEffect&) = default;
};

enum class InlinePolicy : std::uint8_t {
    Default,
    Never,
    Always,
};

enum class ImplementationKind : std::uint8_t {
    OrlFunction,
    Subgraph,
    Builtin,
    Runtime,
};

struct ImplementationRef {
    ImplementationKind kind = ImplementationKind::OrlFunction;
    std::string module;
    std::string function;
    StableId subgraph;
    std::string runtime_name;
};

struct Port {
    StableId id;
    std::string name;
    PortDirection direction = PortDirection::Input;
    PortCardinality cardinality = PortCardinality::Scalar;
    LogicalType type;
    Domain domain = Domain::constant();
    Shape shape = Shape::scalar();
    bool required = true;
    std::optional<ConstantValue> default_value;
    std::string semantic;
    std::string coordinate_space;

    bool compatible_value(const Port& source) const;
};

struct ParameterSpec {
    StableId id;
    std::string name;
    LogicalType type;
    bool compile_time = false;
    std::optional<ConstantValue> default_value;
    std::string semantic;
};

struct NodeDefinition {
    StableId id;
    std::string qualified_name;
    Version version;
    ImplementationRef implementation;
    std::vector<Port> inputs;
    std::vector<Port> outputs;
    std::vector<ParameterSpec> parameters;
    std::vector<ResourceEffect> effects;
    std::vector<std::string> capabilities;
    InlinePolicy inline_policy = InlinePolicy::Default;
    std::string operation;
    bool pure = true;
    bool stateful = false;
    Provenance provenance;

    const Port* input(std::string_view name) const;
    const Port* output(std::string_view name) const;
    const ParameterSpec* parameter(std::string_view name) const;
};

class GraphModule;

class NodeRegistry {
public:
    bool register_definition(NodeDefinition definition, std::string* error = nullptr);
    bool register_subgraph(StableId id, GraphModule graph,
        std::string* error = nullptr);
    const NodeDefinition* find(const StableId& id) const;
    const NodeDefinition* find(std::string_view qualified_name) const;
    const GraphModule* find_subgraph(const StableId& id) const;
    const std::map<StableId, NodeDefinition>& definitions() const { return values_; }

private:
    std::map<StableId, NodeDefinition> values_;
    std::map<StableId, std::shared_ptr<GraphModule>> subgraphs_;
};

struct NodeInstance {
    StableId id;
    StableId definition;
    std::string name;
    std::map<std::string, ConstantValue> parameter_values;
    Provenance provenance;
    InlinePolicy inline_policy = InlinePolicy::Default;
    std::vector<struct ParameterMapping> parameter_mappings;
};

enum class EndpointKind : std::uint8_t {
    NodePort,
    GraphInput,
    GraphOutput,
};

struct Endpoint {
    EndpointKind kind = EndpointKind::NodePort;
    StableId owner;
    StableId port;

    static Endpoint node_port(StableId node, StableId port);
    static Endpoint graph_input(StableId input);
    static Endpoint graph_output(StableId output);

    friend bool operator==(const Endpoint&, const Endpoint&) = default;
};

struct Connection {
    Endpoint source;
    Endpoint destination;
    std::string conversion;
    Shape shape = Shape::scalar();
    Provenance provenance;
    bool feedback = false;
};

enum class ParameterSourceKind : std::uint8_t {
    Constant,
    GraphInput,
    NodeOutput,
    ResourceAttribute,
    Derived,
    State,
};

struct ParameterMapping {
    std::string parameter;
    ParameterSourceKind source_kind = ParameterSourceKind::Constant;
    Endpoint source;
    std::optional<ConstantValue> constant;
    std::string resource_attribute;
    std::string conversion;
    bool compile_time = false;
    Provenance provenance;
};

struct InterfacePort {
    StableId id;
    std::string name;
    PortDirection direction = PortDirection::Input;
    LogicalType type;
    Domain domain = Domain::constant();
    Shape shape = Shape::scalar();
    bool required = true;
    std::optional<ConstantValue> default_value;
    bool compile_time = false;
    std::string binding;
    std::string semantic;
    std::string coordinate_space;
};

struct Resource {
    StableId id;
    std::string name;
    LogicalType type;
    Domain domain = Domain::buffer();
    Shape shape = Shape::scalar();
    AccessMode access = AccessMode::Read;
    bool observable = false;
    std::string binding;
};

class GraphModule {
public:
    std::string module_id;
    Version version;
    std::string language_version = "orl-0";
    std::string logical_abi_version = "orlgraph-0";

    bool add_node(NodeInstance node, std::string* error = nullptr);
    bool add_connection(Connection connection, std::string* error = nullptr);
    bool add_input(InterfacePort input, std::string* error = nullptr);
    bool add_output(InterfacePort output, std::string* error = nullptr);
    bool add_resource(Resource resource, std::string* error = nullptr);
    bool remove_input(const StableId& id, std::string* error = nullptr);
    bool remove_output(const StableId& id, std::string* error = nullptr);

    const NodeInstance* node(const StableId& id) const;
    const InterfacePort* input(const StableId& id) const;
    const InterfacePort* output(const StableId& id) const;
    const Resource* resource(const StableId& id) const;

    const std::map<StableId, NodeInstance>& nodes() const { return nodes_; }
    const std::vector<Connection>& connections() const { return connections_; }
    const std::map<StableId, InterfacePort>& inputs() const { return inputs_; }
    const std::map<StableId, InterfacePort>& outputs() const { return outputs_; }
    const std::map<StableId, Resource>& resources() const { return resources_; }
    std::map<StableId, NodeInstance>& mutable_nodes() { return nodes_; }
    std::vector<Connection>& mutable_connections() { return connections_; }

private:
    std::map<StableId, NodeInstance> nodes_;
    std::vector<Connection> connections_;
    std::map<StableId, InterfacePort> inputs_;
    std::map<StableId, InterfacePort> outputs_;
    std::map<StableId, Resource> resources_;
};

} // namespace orlgraph
