#include "graph_resources.hpp"

#include <utility>

namespace orlrig
{

using orlgraph::StableId;

namespace
{

orlgraph::Port buffer_port(std::string id, std::string name,
    orlgraph::LogicalType element, orlgraph::PortDirection direction,
    orlgraph::Domain domain = orlgraph::Domain::buffer(), bool required = true)
{
    orlgraph::Port port;
    port.id = orlgraph::StableId{std::move(id)};
    port.name = std::move(name);
    port.direction = direction;
    port.cardinality = orlgraph::PortCardinality::Buffer;
    port.type = orlgraph::LogicalType::buffer(std::move(element));
    port.domain = domain;
    port.shape = orlgraph::Shape::one("vertex_count");
    port.required = required;
    return port;
}

orlgraph::NodeDefinition make_capture_definition(
    const RigGraphResourceIds& resources)
{
    orlgraph::NodeDefinition definition;
    definition.id = orlgraph::StableId{"orlrig.deformer.lbs.capture_bind"};
    definition.qualified_name = "orlrig.deformer.lbs.capture_bind";
    definition.implementation.kind = orlgraph::ImplementationKind::Runtime;
    definition.implementation.runtime_name = definition.qualified_name;
    definition.inputs.push_back(buffer_port(
        "joints", "joints", orlgraph::LogicalType::struct_type("Joint"),
        orlgraph::PortDirection::Input));
    definition.outputs.push_back(buffer_port(
        "inverse_binds", "inverse_binds", orlgraph::LogicalType::matrix(),
        orlgraph::PortDirection::Output, orlgraph::Domain::buffer(), false));
    definition.inputs.front().shape = orlgraph::Shape::one("joint_count");
    definition.outputs.front().shape = orlgraph::Shape::one("joint_count");
    definition.effects = {
        {resources.joints, orlgraph::AccessMode::Read, false},
        {resources.inverse_binds, orlgraph::AccessMode::Write, true},
    };
    definition.pure = false;
    definition.inline_policy = orlgraph::InlinePolicy::Never;
    return definition;
}

orlgraph::NodeDefinition make_deform_definition(
    const RigGraphResourceIds& resources)
{
    orlgraph::NodeDefinition definition;
    definition.id = orlgraph::StableId{"orlrig.deformer.lbs.evaluate"};
    definition.qualified_name = "orlrig.deformer.lbs.evaluate";
    definition.implementation.kind = orlgraph::ImplementationKind::Runtime;
    definition.implementation.runtime_name = definition.qualified_name;
    definition.inputs = {
        buffer_port("bind_positions", "bind_positions",
            orlgraph::LogicalType::point(), orlgraph::PortDirection::Input),
        buffer_port("joints", "joints", orlgraph::LogicalType::struct_type("Joint"),
            orlgraph::PortDirection::Input),
        buffer_port("inverse_binds", "inverse_binds",
            orlgraph::LogicalType::matrix(), orlgraph::PortDirection::Input),
        buffer_port("weights", "weights", orlgraph::LogicalType::struct_type("Weight"),
            orlgraph::PortDirection::Input),
    };
    definition.outputs.push_back(buffer_port(
        "posed_positions", "posed_positions", orlgraph::LogicalType::point(),
        orlgraph::PortDirection::Output, orlgraph::Domain::buffer(), false));
    definition.inputs[1].shape = orlgraph::Shape::one("joint_count");
    definition.inputs[2].shape = orlgraph::Shape::one("joint_count");
    definition.inputs[3].shape = orlgraph::Shape::one("weight_count");
    definition.effects = {
        {resources.bind_positions, orlgraph::AccessMode::Read, false},
        {resources.joints, orlgraph::AccessMode::Read, false},
        {resources.inverse_binds, orlgraph::AccessMode::Read, false},
        {resources.weights, orlgraph::AccessMode::Read, false},
        {resources.posed_positions, orlgraph::AccessMode::Write, true},
    };
    definition.pure = false;
    definition.inline_policy = orlgraph::InlinePolicy::Never;
    return definition;
}

void add_interface_input(orlgraph::GraphModule& module,
    const orlgraph::StableId& id, std::string name,
    orlgraph::LogicalType type, std::string shape_symbol)
{
    module.add_input(orlgraph::InterfacePort{
        id, std::move(name), orlgraph::PortDirection::Input,
        std::move(type), orlgraph::Domain::buffer(),
        orlgraph::Shape::one(std::move(shape_symbol)), true, std::nullopt,
        false, id.value, {}, {}});
}

} // namespace

orlgraph::ValidationResult RigGraph::validate() const {
    return orlgraph::validate(module, registry);
}

RigGraphResourceIds add_lbs_resources(orlgraph::GraphModule& module) {
    RigGraphResourceIds resources{
        StableId{"bind_positions"},
        StableId{"posed_positions"},
        StableId{"joints"},
        StableId{"inverse_binds"},
        StableId{"weights"},
    };
    module.add_resource(orlgraph::Resource{
        resources.bind_positions, "bind_positions",
        orlgraph::LogicalType::buffer(orlgraph::LogicalType::point()),
        orlgraph::Domain::buffer(), orlgraph::Shape::one("vertex_count"),
        orlgraph::AccessMode::Read, false, "bind_positions"});
    module.add_resource(orlgraph::Resource{
        resources.posed_positions, "posed_positions",
        orlgraph::LogicalType::buffer(orlgraph::LogicalType::point()),
        orlgraph::Domain::vertex(), orlgraph::Shape::one("vertex_count"),
        orlgraph::AccessMode::Write, true, "posed_positions"});
    module.add_resource(orlgraph::Resource{
        resources.joints, "joints",
        orlgraph::LogicalType::buffer(orlgraph::LogicalType::struct_type("Joint")),
        orlgraph::Domain::joint(), orlgraph::Shape::one("joint_count"),
        orlgraph::AccessMode::Read, false, "joints"});
    module.add_resource(orlgraph::Resource{
        resources.inverse_binds, "inverse_binds",
        orlgraph::LogicalType::buffer(orlgraph::LogicalType::matrix()),
        orlgraph::Domain::joint(), orlgraph::Shape::one("joint_count"),
        orlgraph::AccessMode::ReadWrite, false, "inverse_binds"});
    module.add_resource(orlgraph::Resource{
        resources.weights, "weights",
        orlgraph::LogicalType::buffer(orlgraph::LogicalType::struct_type("Weight")),
        orlgraph::Domain::vertex(), orlgraph::Shape::one("weight_count"),
        orlgraph::AccessMode::Read, false, "weights"});
    return resources;
}

bool register_rig_node_definitions(orlgraph::NodeRegistry& registry,
    std::string* error)
{
    const RigGraphResourceIds resources{
        StableId{"bind_positions"},
        StableId{"posed_positions"},
        StableId{"joints"},
        StableId{"inverse_binds"},
        StableId{"weights"},
    };
    return registry.register_definition(make_capture_definition(resources), error)
        && registry.register_definition(make_deform_definition(resources), error);
}

RigGraph make_lbs_graph() {
    RigGraph graph;
    graph.module.module_id = "orlrig.lbs";
    graph.resources = add_lbs_resources(graph.module);
    add_interface_input(graph.module, graph.resources.bind_positions,
        "bind_positions", orlgraph::LogicalType::buffer(orlgraph::LogicalType::point()),
        "vertex_count");
    add_interface_input(graph.module, graph.resources.joints,
        "joints", orlgraph::LogicalType::buffer(orlgraph::LogicalType::struct_type("Joint")),
        "joint_count");
    add_interface_input(graph.module, graph.resources.weights,
        "weights", orlgraph::LogicalType::buffer(orlgraph::LogicalType::struct_type("Weight")),
        "weight_count");
    graph.module.add_output(orlgraph::InterfacePort{
        graph.resources.posed_positions, "posed_positions",
        orlgraph::PortDirection::Output,
        orlgraph::LogicalType::buffer(orlgraph::LogicalType::point()),
        orlgraph::Domain::buffer(), orlgraph::Shape::one("vertex_count"),
        true, std::nullopt, false, "posed_positions", {}, {}});

    register_rig_node_definitions(graph.registry);
    graph.module.add_node(orlgraph::NodeInstance{
        StableId{"capture_bind"}, StableId{"orlrig.deformer.lbs.capture_bind"},
        "capture_bind", {}, {}, orlgraph::InlinePolicy::Never});
    graph.module.add_node(orlgraph::NodeInstance{
        StableId{"deform"}, StableId{"orlrig.deformer.lbs.evaluate"},
        "deform", {}, {}, orlgraph::InlinePolicy::Never});

    graph.module.add_connection(orlgraph::Connection{
        orlgraph::Endpoint::graph_input(graph.resources.joints),
        orlgraph::Endpoint::node_port(
            StableId{"capture_bind"}, StableId{"joints"})});
    graph.module.add_connection(orlgraph::Connection{
        orlgraph::Endpoint::graph_input(graph.resources.bind_positions),
        orlgraph::Endpoint::node_port(
            StableId{"deform"}, StableId{"bind_positions"})});
    graph.module.add_connection(orlgraph::Connection{
        orlgraph::Endpoint::graph_input(graph.resources.joints),
        orlgraph::Endpoint::node_port(
            StableId{"deform"}, StableId{"joints"})});
    graph.module.add_connection(orlgraph::Connection{
        orlgraph::Endpoint::graph_input(graph.resources.weights),
        orlgraph::Endpoint::node_port(
            StableId{"deform"}, StableId{"weights"})});
    graph.module.add_connection(orlgraph::Connection{
        orlgraph::Endpoint::node_port(
            StableId{"capture_bind"}, StableId{"inverse_binds"}),
        orlgraph::Endpoint::node_port(
            StableId{"deform"}, StableId{"inverse_binds"})});
    graph.module.add_connection(orlgraph::Connection{
        orlgraph::Endpoint::node_port(
            StableId{"deform"}, StableId{"posed_positions"}),
        orlgraph::Endpoint::graph_output(graph.resources.posed_positions)});
    return graph;
}

} // namespace orlrig
