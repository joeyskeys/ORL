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

TEST_CASE("graph validation enforces definition stage masks",
    "[orlgraph][validation][stage]")
{
    NodeRegistry registry;
    NodeDefinition definition;
    definition.id = StableId{"stage.solver_only"};
    definition.qualified_name = "stage.solver_only";
    definition.allowed_stages = GraphStageMask::Solver;
    REQUIRE(registry.register_definition(std::move(definition)));

    GraphModule module;
    REQUIRE(module.add_node(NodeInstance{
        StableId{"node"}, StableId{"stage.solver_only"}, "node",
        {}, {}, InlinePolicy::Default}));

    REQUIRE(validate(module, registry, GraphStage::Solver).ok());
    const auto deformer = validate(module, registry, GraphStage::Deformer);
    REQUIRE_FALSE(deformer.ok());
    REQUIRE(std::any_of(deformer.diagnostics.begin(),
        deformer.diagnostics.end(),
        [](const Diagnostic& diagnostic) {
            return diagnostic.code == "ORLGRAPH_STAGE_MISMATCH";
        }));
}

TEST_CASE("graph interface removal cleans its connections",
    "[orlgraph][editing]")
{
    NodeRegistry registry;
    REQUIRE(registry.register_definition(make_passthrough_definition()));

    GraphModule module;
    REQUIRE(module.add_input(InterfacePort{
        StableId{"input"}, "input", PortDirection::Input,
        LogicalType::int64(), Domain::constant(), Shape::scalar(), true,
        std::nullopt, false, "scene.test.input", {}, {}}));
    REQUIRE(module.add_output(InterfacePort{
        StableId{"output"}, "output", PortDirection::Output,
        LogicalType::int64(), Domain::constant(), Shape::scalar(), true,
        std::nullopt, false, "scene.test.output", {}, {}}));
    REQUIRE(module.add_node(NodeInstance{
        StableId{"pass"}, StableId{"builtin.pass"}, "pass", {}, {},
        InlinePolicy::Default}));
    REQUIRE(module.add_connection(Connection{
        Endpoint::graph_input(StableId{"input"}),
        Endpoint::node_port(StableId{"pass"}, StableId{"value"})}));
    REQUIRE(module.add_connection(Connection{
        Endpoint::node_port(StableId{"pass"}, StableId{"result"}),
        Endpoint::graph_output(StableId{"output"})}));

    REQUIRE(module.remove_input(StableId{"input"}));
    REQUIRE(module.input(StableId{"input"}) == nullptr);
    REQUIRE(module.connections().size() == 1);
    REQUIRE(module.remove_output(StableId{"output"}));
    REQUIRE(module.connections().empty());
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

TEST_CASE("graph reflection preserves port semantics",
    "[orlgraph][reflection]")
{
    NodeRegistry registry;
    auto definition = make_add_definition();
    definition.outputs.front().semantic = "scene.array_index";
    REQUIRE(registry.register_definition(std::move(definition)));

    GraphModule module;
    REQUIRE(module.add_node(NodeInstance{
        StableId{"add"}, StableId{"builtin.add"}, "add", {}, {},
        InlinePolicy::Default}));
    const auto reflection = reflect(module, registry);
    REQUIRE(reflection.nodes.size() == 1);
    const auto found = std::find_if(reflection.nodes.front().ports.begin(),
        reflection.nodes.front().ports.end(),
        [](const ReflectedPort& port) {
            return port.id == StableId{"result"};
        });
    REQUIRE(found != reflection.nodes.front().ports.end());
    REQUIRE(found->semantic == "scene.array_index");
}

#if defined(ORLGRAPH_HAS_IO)
TEST_CASE("oro serialization is deterministic and round trips", "[orlgraph][oro]") {
    NodeRegistry registry;
    auto definition = make_passthrough_definition();
    definition.inputs.front().required = false;
    definition.parameters.push_back(ParameterSpec{
        StableId{"name"}, "name", LogicalType::string(), true,
        std::nullopt, "scene.element_name"});
    definition.outputs.push_back(Port{
        StableId{"adapted"}, "adapted", PortDirection::Output,
        PortCardinality::Scalar, LogicalType::int64(), Domain::constant(),
        Shape::scalar(), false, std::nullopt, "scene.joint.xform", "world",
        Port::OutputAdapter{
            StableId{"test.convert.identity"}, StableId{"result"},
            StableId{"test.convert.writeback"},
            StableId{"result"}}});
    REQUIRE(registry.register_definition(std::move(definition)));
    ConversionDefinition conversion;
    conversion.id = StableId{"test.convert.identity"};
    conversion.qualified_name = "test.convert.identity";
    conversion.source = Port{
        StableId{"source"}, "source", PortDirection::Output,
        PortCardinality::Scalar, LogicalType::int64(), Domain::constant(),
        Shape::scalar(), false, std::nullopt, {}, {}};
    conversion.output = Port{
        StableId{"output"}, "output", PortDirection::Output,
        PortCardinality::Scalar, LogicalType::int64(), Domain::constant(),
        Shape::scalar(), false, std::nullopt, {}, {}};
    conversion.implementation.kind = ImplementationKind::OrlFunction;
    conversion.implementation.function = "identity";
    REQUIRE(registry.register_conversion(std::move(conversion)));
    ConversionDefinition writeback;
    writeback.id = StableId{"test.convert.writeback"};
    writeback.qualified_name = "test.convert.writeback";
    writeback.source = Port{
        StableId{"source"}, "source", PortDirection::Input,
        PortCardinality::Scalar, LogicalType::int64(), Domain::constant(),
        Shape::scalar(), true, std::nullopt, {}, {}};
    writeback.output = Port{
        StableId{"target"}, "target", PortDirection::Input,
        PortCardinality::Buffer, LogicalType::buffer(LogicalType::int64()),
        Domain::buffer(), Shape::one("count"), true, std::nullopt, {}, {}};
    writeback.selector = Port{
        StableId{"selector"}, "selector", PortDirection::Output,
        PortCardinality::Scalar, LogicalType::int64(), Domain::constant(),
        Shape::scalar(), true, std::nullopt, {}, {}};
    writeback.implementation.kind = ImplementationKind::OrlFunction;
    writeback.implementation.function = "writeback";
    writeback.emitter = ConversionEmitterKind::OrlFunctionWriteback;
    writeback.pure = false;
    REQUIRE(registry.register_conversion(std::move(writeback)));

    GraphModule module;
    module.module_id = "oro.test";
    REQUIRE(module.add_input(InterfacePort{
        StableId{"scene_input"}, "Scene Input", PortDirection::Input,
        LogicalType::buffer(LogicalType::point()), Domain::vertex(),
        Shape::one("vertex_count"), true, std::nullopt, false,
        "scene.mesh.body.positions", "mesh.positions", "world"}));
    REQUIRE(module.add_input(InterfacePort{
        StableId{"locator_input"}, "Locator Input", PortDirection::Input,
        LogicalType::buffer(LogicalType::struct_type("Locator")),
        Domain::rig(), Shape::one("locator_count"), true, std::nullopt, false,
        "scene.rig.locators", "locators", "world"}));
    REQUIRE(module.add_node(NodeInstance{
        StableId{"node"}, StableId{"builtin.pass"}, "node",
        {{"name", ConstantValue{
            LogicalType::string(), std::string{"second"}}}},
        {}, InlinePolicy::Default}));

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
    REQUIRE(loaded.registry.find_conversion(
        StableId{"test.convert.identity"}) != nullptr);
    const auto* loaded_definition =
        loaded.registry.find(StableId{"builtin.pass"});
    REQUIRE(loaded_definition != nullptr);
    REQUIRE(loaded_definition->output("adapted") != nullptr);
    REQUIRE(loaded_definition->output("adapted")->output_adapter.has_value());
    REQUIRE(loaded_definition->output("adapted")
        ->output_adapter->source_port == StableId{"result"});
    REQUIRE(loaded_definition->output("adapted")
        ->output_adapter->writeback_conversion
        == StableId{"test.convert.writeback"});
    REQUIRE(loaded_definition->output("adapted")
        ->output_adapter->writeback_source_port
        == StableId{"result"});
    REQUIRE(loaded_definition->output("adapted")->semantic
        == "scene.joint.xform");
    REQUIRE(loaded_definition->output("adapted")->coordinate_space
        == "world");
    const auto* loaded_writeback = loaded.registry.find_conversion(
        StableId{"test.convert.writeback"});
    REQUIRE(loaded_writeback != nullptr);
    REQUIRE(loaded_writeback->emitter
        == ConversionEmitterKind::OrlFunctionWriteback);
    REQUIRE(loaded_writeback->selector.has_value());
    const auto* input = loaded.module.input(StableId{"scene_input"});
    REQUIRE(input != nullptr);
    REQUIRE(input->binding == "scene.mesh.body.positions");
    REQUIRE(input->semantic == "mesh.positions");
    REQUIRE(input->coordinate_space == "world");
    const auto* locator_input =
        loaded.module.input(StableId{"locator_input"});
    REQUIRE(locator_input != nullptr);
    REQUIRE(locator_input->type
        == LogicalType::buffer(LogicalType::struct_type("Locator")));
    REQUIRE(locator_input->binding == "scene.rig.locators");
    const auto* lookup = loaded.module.node(StableId{"node"});
    REQUIRE(lookup != nullptr);
    REQUIRE(lookup->parameter_values.contains("name"));
    REQUIRE_FALSE(lookup->parameter_values.contains("index"));
}

TEST_CASE("oro serialization preserves partial evaluation footprints",
    "[orlgraph][oro][partial]")
{
    NodeRegistry registry;
    auto definition = make_passthrough_definition();
    PartialEvaluationFootprint footprint;
    footprint.declared = true;
    footprint.supports_sparse_dispatch = true;
    footprint.propagation = PartialPropagation::AncestorsAndDescendants;
    footprint.read_joint_ports = {"parent"};
    footprint.write_joints = {StableId{"joint.output"}};
    footprint.read_resources = {StableId{"scene.joints"}};
    definition.partial_footprint = footprint;
    REQUIRE(registry.register_definition(std::move(definition)));

    GraphModule module;
    module.module_id = "oro.partial";
    const auto serialized = serialize_oro(module, registry);
    REQUIRE(serialized.ok);
    const auto loaded = deserialize_oro(serialized.text);
    REQUIRE(loaded.ok);
    const auto* result = loaded.registry.find(StableId{"builtin.pass"});
    REQUIRE(result != nullptr);
    REQUIRE(result->partial_footprint.has_value());
    REQUIRE(*result->partial_footprint == footprint);
}

TEST_CASE("editable graph JSON is deterministic and round trips",
    "[orlgraph][graph-json]")
{
    GraphModule module;
    module.module_id = "character.pose";
    module.version = Version{0, 1, 0};
    REQUIRE(module.add_input(InterfacePort{
        StableId{"scene_input"}, "Scene Input", PortDirection::Input,
        LogicalType::buffer(LogicalType::point()), Domain::vertex(),
        Shape::one("vertex_count"), true, std::nullopt, false,
        "scene.mesh.body.positions", "mesh.positions", "world"}));
    REQUIRE(module.add_output(InterfacePort{
        StableId{"result"}, "Result", PortDirection::Output,
        LogicalType::int64(), Domain::constant(), Shape::scalar(), false,
        std::nullopt, false, "result", "result", {}}));

    const auto first = serialize_graph_json(module);
    const auto second = serialize_graph_json(module);
    REQUIRE(first.ok);
    REQUIRE(first.text == second.text);
    REQUIRE(first.content_hash == second.content_hash);

    const auto loaded = deserialize_graph_json(first.text);
    REQUIRE(loaded.ok);
    REQUIRE(loaded.module.module_id == "character.pose");
    REQUIRE(loaded.module.version == Version{0, 1, 0});
    const auto* input = loaded.module.input(StableId{"scene_input"});
    REQUIRE(input != nullptr);
    REQUIRE(input->binding == "scene.mesh.body.positions");
    REQUIRE(input->semantic == "mesh.positions");
    REQUIRE(input->coordinate_space == "world");
}

TEST_CASE("staged graph JSON preserves solver and deformer graphs",
    "[orlgraph][graph-stages][graph-json]")
{
    GraphModule solver;
    solver.module_id = "character.solver";

    GraphModule deformer;
    deformer.module_id = "character.deformer";
    REQUIRE(deformer.add_node(NodeInstance{
        StableId{"computed_joints"},
        StableId{"orlrig.stage.computed_joints"},
        "computed_joints", {}, {}, InlinePolicy::Never}));

    const auto first = serialize_graph_stages_json(solver, deformer);
    const auto second = serialize_graph_stages_json(solver, deformer);
    REQUIRE(first.ok);
    REQUIRE(first.text == second.text);
    REQUIRE(first.content_hash == second.content_hash);

    const auto loaded = deserialize_graph_stages_json(first.text);
    REQUIRE(loaded.ok);
    REQUIRE(loaded.solver.module_id == "character.solver");
    REQUIRE(loaded.deformer.module_id == "character.deformer");
    REQUIRE(loaded.solver.node(StableId{"computed_joints"}) == nullptr);
    REQUIRE(loaded.deformer.node(StableId{"computed_joints"}) != nullptr);
}
#endif

#endif
