#if __has_include(<catch2/catch_all.hpp>)

#include <catch2/catch_all.hpp>

#include <orlgraph/orlgraph.hpp>

#include <algorithm>
#include <utility>

#if defined(ORLGRAPH_HAS_IO)
#include "graph_serialization.hpp"
#endif

using namespace orlgraph;

namespace
{

NodeDefinition make_add_definition() {
    NodeDefinition definition;
    definition.id = StableId{"builtin.add"};
    definition.qualified_name = "builtin.add";
    definition.inputs = {
        Port{StableId{"left"}, "left", PortDirection::Input,
            PortCardinality::Scalar, LogicalType::int64(), Domain::constant(),
            Shape::scalar(), true, std::nullopt, {}, {}},
        Port{StableId{"right"}, "right", PortDirection::Input,
            PortCardinality::Scalar, LogicalType::int64(), Domain::constant(),
            Shape::scalar(), true, std::nullopt, {}, {}},
    };
    definition.outputs = {
        Port{StableId{"result"}, "result", PortDirection::Output,
            PortCardinality::Scalar, LogicalType::int64(), Domain::constant(),
            Shape::scalar(), true, std::nullopt, {}, {}},
    };
    return definition;
}

NodeDefinition make_passthrough_definition() {
    NodeDefinition definition;
    definition.id = StableId{"builtin.pass"};
    definition.qualified_name = "builtin.pass";
    definition.inputs = {
        Port{StableId{"value"}, "value", PortDirection::Input,
            PortCardinality::Scalar, LogicalType::int64(), Domain::constant(),
            Shape::scalar(), true, std::nullopt, {}, {}},
    };
    definition.outputs = {
        Port{StableId{"result"}, "result", PortDirection::Output,
            PortCardinality::Scalar, LogicalType::int64(), Domain::constant(),
            Shape::scalar(), true, std::nullopt, {}, {}},
    };
    return definition;
}

} // namespace

TEST_CASE("graph module validates typed connections", "[orlgraph][validation]") {
    NodeRegistry registry;
    REQUIRE(registry.register_definition(make_add_definition()));

    GraphModule module;
    module.module_id = "test.add";
    REQUIRE(module.add_input(InterfacePort{
        StableId{"input.left"}, "input.left", PortDirection::Input,
        LogicalType::int64(), Domain::constant(), Shape::scalar(), true,
        std::nullopt, false, "left", {}}));
    REQUIRE(module.add_input(InterfacePort{
        StableId{"input.right"}, "input.right", PortDirection::Input,
        LogicalType::int64(), Domain::constant(), Shape::scalar(), true,
        std::nullopt, false, "right", {}}));
    REQUIRE(module.add_output(InterfacePort{
        StableId{"output.result"}, "output.result", PortDirection::Output,
        LogicalType::int64(), Domain::constant(), Shape::scalar(), true,
        std::nullopt, false, "result", {}}));
    REQUIRE(module.add_node(NodeInstance{
        StableId{"add"}, StableId{"builtin.add"}, "add", {}, {}, InlinePolicy::Default}));

    REQUIRE(module.add_connection(Connection{
        Endpoint::graph_input(StableId{"input.left"}),
        Endpoint::node_port(StableId{"add"}, StableId{"left"})}));
    REQUIRE(module.add_connection(Connection{
        Endpoint::graph_input(StableId{"input.right"}),
        Endpoint::node_port(StableId{"add"}, StableId{"right"})}));
    REQUIRE(module.add_connection(Connection{
        Endpoint::node_port(StableId{"add"}, StableId{"result"}),
        Endpoint::graph_output(StableId{"output.result"})}));

    const ValidationResult result = validate(module, registry);
    REQUIRE(result.ok());
    REQUIRE(result.schedule.order == std::vector<StableId>{StableId{"add"}});
}

TEST_CASE("graph scheduling is deterministic and rejects cycles", "[orlgraph][schedule]") {
    NodeRegistry registry;
    REQUIRE(registry.register_definition(make_passthrough_definition()));

    GraphModule module;
    REQUIRE(module.add_node(NodeInstance{
        StableId{"b"}, StableId{"builtin.pass"}, "b", {}, {}, InlinePolicy::Default}));
    REQUIRE(module.add_node(NodeInstance{
        StableId{"a"}, StableId{"builtin.pass"}, "a", {}, {}, InlinePolicy::Default}));
    REQUIRE(module.add_output(InterfacePort{
        StableId{"out"}, "out", PortDirection::Output,
        LogicalType::int64(), Domain::constant(), Shape::scalar(), false,
        std::nullopt, false, {}, {}}));
    REQUIRE(module.add_connection(Connection{
        Endpoint::node_port(StableId{"a"}, StableId{"result"}),
        Endpoint::node_port(StableId{"b"}, StableId{"value"})}));
    REQUIRE(module.add_connection(Connection{
        Endpoint::node_port(StableId{"b"}, StableId{"result"}),
        Endpoint::graph_output(StableId{"out"})}));

    const auto schedule = topological_schedule(module);
    REQUIRE(schedule.ok);
    REQUIRE(schedule.order == std::vector<StableId>{
        StableId{"a"}, StableId{"b"}});

    GraphModule cyclic;
    REQUIRE(cyclic.add_node(NodeInstance{
        StableId{"a"}, StableId{"builtin.pass"}, "a", {}, {}, InlinePolicy::Default}));
    REQUIRE(cyclic.add_node(NodeInstance{
        StableId{"b"}, StableId{"builtin.pass"}, "b", {}, {}, InlinePolicy::Default}));
    REQUIRE(cyclic.add_connection(Connection{
        Endpoint::node_port(StableId{"a"}, StableId{"result"}),
        Endpoint::node_port(StableId{"b"}, StableId{"value"})}));
    REQUIRE(cyclic.add_connection(Connection{
        Endpoint::node_port(StableId{"b"}, StableId{"result"}),
        Endpoint::node_port(StableId{"a"}, StableId{"value"})}));
    REQUIRE_FALSE(topological_schedule(cyclic).ok);
}

TEST_CASE("graph optimizer removes dead pure nodes", "[orlgraph][optimizer]") {
    NodeRegistry registry;
    auto pass = make_passthrough_definition();
    REQUIRE(registry.register_definition(std::move(pass)));

    auto add = make_add_definition();
    add.pure = true;
    REQUIRE(registry.register_definition(std::move(add)));

    GraphModule module;
    REQUIRE(module.add_input(InterfacePort{
        StableId{"input"}, "input", PortDirection::Input,
        LogicalType::int64(), Domain::constant(), Shape::scalar(), true,
        std::nullopt, false, "input", {}}));
    REQUIRE(module.add_output(InterfacePort{
        StableId{"output"}, "output", PortDirection::Output,
        LogicalType::int64(), Domain::constant(), Shape::scalar(), true,
        std::nullopt, false, "output", {}}));
    REQUIRE(module.add_node(NodeInstance{
        StableId{"live"}, StableId{"builtin.pass"}, "live", {}, {}, InlinePolicy::Default}));
    REQUIRE(module.add_node(NodeInstance{
        StableId{"dead"}, StableId{"builtin.add"}, "dead", {}, {}, InlinePolicy::Default}));
    REQUIRE(module.add_connection(Connection{
        Endpoint::graph_input(StableId{"input"}),
        Endpoint::node_port(StableId{"live"}, StableId{"value"})}));
    REQUIRE(module.add_connection(Connection{
        Endpoint::node_port(StableId{"live"}, StableId{"result"}),
        Endpoint::graph_output(StableId{"output"})}));
    REQUIRE(module.add_connection(Connection{
        Endpoint::graph_input(StableId{"input"}),
        Endpoint::node_port(StableId{"dead"}, StableId{"left"})}));
    REQUIRE(module.add_connection(Connection{
        Endpoint::graph_input(StableId{"input"}),
        Endpoint::node_port(StableId{"dead"}, StableId{"right"})}));

    GraphOptimizer optimizer;
    const auto optimized = optimizer.optimize(module, registry);
    REQUIRE(optimized.ok);
    REQUIRE(module.node(StableId{"live"}) != nullptr);
    REQUIRE(module.node(StableId{"dead"}) == nullptr);
    REQUIRE(std::find(optimized.removed_nodes.begin(), optimized.removed_nodes.end(),
        StableId{"dead"}) != optimized.removed_nodes.end());
}

TEST_CASE("graph optimizer flattens registered subgraphs", "[orlgraph][optimizer][flatten]") {
    NodeRegistry registry;
    REQUIRE(registry.register_definition(make_passthrough_definition()));

    GraphModule nested;
    REQUIRE(nested.add_input(InterfacePort{
        StableId{"in"}, "in", PortDirection::Input,
        LogicalType::int64(), Domain::constant(), Shape::scalar(), true,
        std::nullopt, false, {}, {}}));
    REQUIRE(nested.add_output(InterfacePort{
        StableId{"out"}, "out", PortDirection::Output,
        LogicalType::int64(), Domain::constant(), Shape::scalar(), true,
        std::nullopt, false, {}, {}}));
    REQUIRE(nested.add_node(NodeInstance{
        StableId{"pass"}, StableId{"builtin.pass"}, "pass", {}, {}, InlinePolicy::Default}));
    REQUIRE(nested.add_connection(Connection{
        Endpoint::graph_input(StableId{"in"}),
        Endpoint::node_port(StableId{"pass"}, StableId{"value"})}));
    REQUIRE(nested.add_connection(Connection{
        Endpoint::node_port(StableId{"pass"}, StableId{"result"}),
        Endpoint::graph_output(StableId{"out"})}));
    REQUIRE(registry.register_subgraph(StableId{"subgraph.pass"}, nested));

    NodeDefinition wrapper;
    wrapper.id = StableId{"builtin.wrapper"};
    wrapper.qualified_name = "builtin.wrapper";
    wrapper.implementation.kind = ImplementationKind::Subgraph;
    wrapper.implementation.subgraph = StableId{"subgraph.pass"};
    wrapper.inputs = make_passthrough_definition().inputs;
    wrapper.inputs.front().id = StableId{"in"};
    wrapper.inputs.front().name = "in";
    wrapper.outputs = make_passthrough_definition().outputs;
    wrapper.outputs.front().id = StableId{"out"};
    wrapper.outputs.front().name = "out";
    REQUIRE(registry.register_definition(std::move(wrapper)));

    GraphModule module;
    REQUIRE(module.add_input(InterfacePort{
        StableId{"input"}, "input", PortDirection::Input,
        LogicalType::int64(), Domain::constant(), Shape::scalar(), true,
        std::nullopt, false, {}, {}}));
    REQUIRE(module.add_output(InterfacePort{
        StableId{"output"}, "output", PortDirection::Output,
        LogicalType::int64(), Domain::constant(), Shape::scalar(), true,
        std::nullopt, false, {}, {}}));
    REQUIRE(module.add_node(NodeInstance{
        StableId{"wrapper"}, StableId{"builtin.wrapper"}, "wrapper", {},
        {}, InlinePolicy::Default}));
    REQUIRE(module.add_connection(Connection{
        Endpoint::graph_input(StableId{"input"}),
        Endpoint::node_port(StableId{"wrapper"}, StableId{"in"})}));
    REQUIRE(module.add_connection(Connection{
        Endpoint::node_port(StableId{"wrapper"}, StableId{"out"}),
        Endpoint::graph_output(StableId{"output"})}));

    const auto optimized = GraphOptimizer{}.optimize(module, registry);
    REQUIRE(optimized.ok);
    REQUIRE(module.node(StableId{"wrapper"}) == nullptr);
    REQUIRE(module.node(StableId{"wrapper/pass"}) != nullptr);
}

#if defined(ORLGRAPH_HAS_IO)
TEST_CASE("oro serialization is deterministic and round trips", "[orlgraph][oro]") {
    NodeRegistry registry;
    auto definition = make_passthrough_definition();
    definition.inputs.front().required = false;
    REQUIRE(registry.register_definition(std::move(definition)));

    GraphModule module;
    module.module_id = "oro.test";
    REQUIRE(module.add_node(NodeInstance{
        StableId{"node"}, StableId{"builtin.pass"}, "node", {}, {}, InlinePolicy::Default}));

    const auto first = serialize_oro(module, registry);
    const auto second = serialize_oro(module, registry);
    REQUIRE(first.ok);
    REQUIRE(first.text == second.text);
    REQUIRE(first.content_hash == second.content_hash);

    const auto loaded = deserialize_oro(first.text);
    REQUIRE(loaded.ok);
    REQUIRE(loaded.module.module_id == "oro.test");
    REQUIRE(loaded.module.nodes().size() == 1);
    REQUIRE(loaded.registry.find(StableId{"builtin.pass"}) != nullptr);
}
#endif

#endif
