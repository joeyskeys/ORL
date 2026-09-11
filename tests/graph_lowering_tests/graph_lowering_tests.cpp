#if __has_include(<catch2/catch_all.hpp>)

#include <catch2/catch_all.hpp>

#include <orlgraph/orlgraph.hpp>

#include "orl_graph_exec.hpp"

using namespace orlgraph;
using namespace ORL::exec;

namespace
{

NodeDefinition make_add_definition() {
    NodeDefinition definition;
    definition.id = StableId{"test.add"};
    definition.qualified_name = "test.add";
    definition.implementation.kind = ImplementationKind::OrlFunction;
    definition.implementation.function = "add";
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
            Shape::scalar(), false, std::nullopt, {}, {}},
    };
    return definition;
}

GraphModule make_graph() {
    GraphModule module;
    module.module_id = "lowering.add";
    module.add_input(InterfacePort{
        StableId{"left"}, "left", PortDirection::Input,
        LogicalType::int64(), Domain::constant(), Shape::scalar(), true,
        std::nullopt, false, "left", {}});
    module.add_input(InterfacePort{
        StableId{"right"}, "right", PortDirection::Input,
        LogicalType::int64(), Domain::constant(), Shape::scalar(), true,
        std::nullopt, false, "right", {}});
    module.add_output(InterfacePort{
        StableId{"result"}, "result", PortDirection::Output,
        LogicalType::int64(), Domain::constant(), Shape::scalar(), true,
        std::nullopt, false, "result", {}});
    module.add_node(NodeInstance{
        StableId{"add_node"}, StableId{"test.add"}, "add_node", {},
        {}, InlinePolicy::Default});
    module.add_connection(Connection{
        Endpoint::graph_input(StableId{"left"}),
        Endpoint::node_port(StableId{"add_node"}, StableId{"left"})});
    module.add_connection(Connection{
        Endpoint::graph_input(StableId{"right"}),
        Endpoint::node_port(StableId{"add_node"}, StableId{"right"})});
    module.add_connection(Connection{
        Endpoint::node_port(StableId{"add_node"}, StableId{"result"}),
        Endpoint::graph_output(StableId{"result"})});
    return module;
}

} // namespace

TEST_CASE("optimized graph lowers to reusable CPU ORL execution", "[orlgraph][lowering][cpu]") {
    NodeRegistry registry;
    REQUIRE(registry.register_definition(make_add_definition()));

    const auto program = OrlGraphProgram::Compile(
        make_graph(), registry,
        {
            .entry_function = "graph_compute",
            .source_preamble = "int add(int left, int right) { return left + right; }",
            .emit_module_uses = false,
        });
    REQUIRE(program.valid());
    REQUIRE(program.source().find("int graph_compute") != std::string::npos);

    auto execution = OrlGraphExecution::Create(program, Backend::Cpu);
    REQUIRE(execution.valid());
    REQUIRE(execution.bind_int("left", 7));
    REQUIRE(execution.bind_int("right", 11));
    const auto result = execution.evaluate();
    REQUIRE(result.has_value());
    REQUIRE(*result == 18);
}

TEST_CASE("CUDA graph execution matches CPU when a device is available",
    "[orlgraph][lowering][cuda]")
{
    NodeRegistry registry;
    REQUIRE(registry.register_definition(make_add_definition()));
    const auto program = OrlGraphProgram::Compile(
        make_graph(), registry,
        {
            .entry_function = "graph_compute",
            .source_preamble = "int add(int left, int right) { return left + right; }",
            .emit_module_uses = false,
        });
    REQUIRE(program.valid());

    auto execution = OrlGraphExecution::Create(program, Backend::Cuda);
    if (!execution.valid()) {
        SKIP("CUDA backend/device unavailable");
    }
    REQUIRE(execution.bind_int("left", 7));
    REQUIRE(execution.bind_int("right", 11));
    const auto result = execution.evaluate();
    REQUIRE(result.has_value());
    REQUIRE(*result == 18);
}

#endif
