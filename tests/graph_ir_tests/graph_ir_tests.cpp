#if __has_include(<catch2/catch_all.hpp>)

#include <catch2/catch_all.hpp>

#include <orlgraph/orlgraph.hpp>

#include <algorithm>
#include <filesystem>
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

TEST_CASE("graph types preserve exact nominal handle identity",
    "[orlgraph][handle][types]")
{
    const auto joint = LogicalType::handle("orlrig::joint_handle");
    const auto same_joint = LogicalType::handle("orlrig::joint_handle");
    const auto locator = LogicalType::handle("orlrig::locator_handle");

    REQUIRE(joint.is_handle());
    REQUIRE(joint.is_scalar());
    REQUIRE(joint.canonical_name() == "handle:orlrig::joint_handle");
    REQUIRE(joint == same_joint);
    REQUIRE(joint != locator);
    REQUIRE(is_assignable(joint, same_joint));
    REQUIRE_FALSE(is_assignable(joint, locator));
    REQUIRE_FALSE(is_assignable(joint, LogicalType::int64()));
    REQUIRE(is_valid_handle_name(joint.name));
    REQUIRE_FALSE(is_valid_handle_name("joint_handle"));
}

TEST_CASE("graph types support normalized handle union assignability",
    "[orlgraph][handle][union]")
{
    const auto joint = LogicalType::handle("orlrig::joint_handle");
    const auto locator = LogicalType::handle("orlrig::locator_handle");
    const auto union_type = LogicalType::handle_union(
        "orlrig::component_handle",
        {"orlrig::locator_handle", "orlrig::joint_handle"});
    const auto universal = LogicalType::handle_union(
        "handle", {}, true);
    REQUIRE(is_assignable(joint, union_type));
    REQUIRE(is_assignable(locator, union_type));
    REQUIRE(is_assignable(joint, universal));
    REQUIRE(is_assignable(locator, universal));
    REQUIRE_FALSE(is_assignable(union_type, joint));
    REQUIRE_FALSE(is_assignable(universal, joint));
    REQUIRE(union_type.canonical_name()
        == "handle_union:orlrig::component_handle"
            "<orlrig::joint_handle|orlrig::locator_handle>");
}

TEST_CASE("graph validation rejects incompatible exact handle connections",
    "[orlgraph][validation][handle]")
{
    const auto make_definition = [](
        std::string id, LogicalType output_type, LogicalType input_type) {
        NodeDefinition definition;
        definition.id = StableId{std::move(id)};
        definition.qualified_name = definition.id.value;
        definition.outputs.push_back(Port{
            StableId{"out"}, "out", PortDirection::Output,
            PortCardinality::Scalar, std::move(output_type),
            Domain::constant(), Shape::scalar(), false,
            std::nullopt, {}, {}});
        definition.inputs.push_back(Port{
            StableId{"in"}, "in", PortDirection::Input,
            PortCardinality::Scalar, std::move(input_type),
            Domain::constant(), Shape::scalar(), false,
            std::nullopt, {}, {}});
        return definition;
    };

    NodeRegistry registry;
    REQUIRE(registry.register_definition(make_definition(
        "test.joint_source",
        LogicalType::handle("orlrig::joint_handle"),
        LogicalType::int64())));
    REQUIRE(registry.register_definition(make_definition(
        "test.locator_sink",
        LogicalType::int64(),
        LogicalType::handle("orlrig::locator_handle"))));
    REQUIRE(registry.register_definition(make_definition(
        "test.joint_sink",
        LogicalType::int64(),
        LogicalType::handle("orlrig::joint_handle"))));

    const auto validate_connection = [&registry](
        StableId sink_definition) {
        GraphModule module;
        REQUIRE(module.add_node(NodeInstance{
            StableId{"source"}, StableId{"test.joint_source"}, "source",
            {}, {}, InlinePolicy::Default}));
        REQUIRE(module.add_node(NodeInstance{
            StableId{"sink"}, std::move(sink_definition), "sink",
            {}, {}, InlinePolicy::Default}));
        REQUIRE(module.add_connection(Connection{
            Endpoint::node_port(StableId{"source"}, StableId{"out"}),
            Endpoint::node_port(StableId{"sink"}, StableId{"in"})}));
        return validate(module, registry);
    };

    const auto exact = validate_connection(
        StableId{"test.joint_sink"});
    REQUIRE(exact.ok());

    const auto locator = validate_connection(
        StableId{"test.locator_sink"});
    REQUIRE_FALSE(locator.ok());
    REQUIRE(std::any_of(locator.diagnostics.begin(),
        locator.diagnostics.end(),
        [](const Diagnostic& diagnostic) {
            return diagnostic.code == "ORLGRAPH_TYPE_MISMATCH"
                && diagnostic.message.find(
                    "handle:orlrig::joint_handle") != std::string::npos
                && diagnostic.message.find(
                    "handle:orlrig::locator_handle") != std::string::npos;
        }));
}

TEST_CASE("graph reflection retains handle port types",
    "[orlgraph][reflection][handle]")
{
    NodeRegistry registry;
    NodeDefinition definition;
    definition.id = StableId{"test.handle_node"};
    definition.qualified_name = "test.handle_node";
    definition.outputs.push_back(Port{
        StableId{"handle"}, "handle", PortDirection::Output,
        PortCardinality::Scalar,
        LogicalType::handle("orlrig::joint_handle"),
        Domain::rig(), Shape::scalar(), false,
        std::nullopt, {}, {}});
    REQUIRE(registry.register_definition(definition));

    GraphModule module;
    REQUIRE(module.add_node(NodeInstance{
        StableId{"node"}, StableId{"test.handle_node"}, "node",
        {}, {}, InlinePolicy::Default}));

    const auto reflected = reflect(module, registry);
    REQUIRE(reflected.nodes.size() == 1);
    REQUIRE(reflected.nodes.front().ports.size() == 1);
    REQUIRE(reflected.nodes.front().ports.front().type
        == LogicalType::handle("orlrig::joint_handle"));
}

TEST_CASE("graph validation rejects handle constants",
    "[orlgraph][validation][handle][constant]")
{
    NodeRegistry registry;
    NodeDefinition definition;
    definition.id = StableId{"test.handle_parameter"};
    definition.qualified_name = "test.handle_parameter";
    definition.parameters.push_back(ParameterSpec{
        StableId{"value"}, "value",
        LogicalType::handle("orlrig::joint_handle"), true,
        std::nullopt, {}});
    REQUIRE(registry.register_definition(std::move(definition)));

    GraphModule module;
    REQUIRE(module.add_node(NodeInstance{
        StableId{"node"}, StableId{"test.handle_parameter"}, "node",
        {{"value", ConstantValue{
            LogicalType::handle("orlrig::joint_handle"),
            std::int64_t{1}}}},
        {}, InlinePolicy::Default}));

    const auto result = validate(module, registry);
    REQUIRE_FALSE(result.ok());
    REQUIRE(std::any_of(result.diagnostics.begin(),
        result.diagnostics.end(),
        [](const Diagnostic& diagnostic) {
            return diagnostic.code == "ORLGRAPH_HANDLE_CONSTANT";
        }));
}

TEST_CASE("graph validation rejects nested handle collections",
    "[orlgraph][validation][handle][collection]")
{
    NodeRegistry registry;
    NodeDefinition definition;
    definition.id = StableId{"test.handle_collection"};
    definition.qualified_name = "test.handle_collection";
    definition.outputs.push_back(Port{
        StableId{"handles"}, "handles", PortDirection::Output,
        PortCardinality::Buffer,
        LogicalType::buffer(LogicalType::array(
            LogicalType::handle("orlrig::joint_handle"), 2)),
        Domain::buffer(), Shape::one("count"), false,
        std::nullopt, {}, {}});
    REQUIRE(registry.register_definition(std::move(definition)));

    GraphModule module;
    REQUIRE(module.add_node(NodeInstance{
        StableId{"node"}, StableId{"test.handle_collection"}, "node",
        {}, {}, InlinePolicy::Default}));

    const auto result = validate(module, registry);
    REQUIRE_FALSE(result.ok());
    REQUIRE(std::any_of(result.diagnostics.begin(),
        result.diagnostics.end(),
        [](const Diagnostic& diagnostic) {
            return diagnostic.code == "ORLGRAPH_HANDLE_CARDINALITY";
        }));
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
    footprint.handle_effects.push_back({
        StableId{"joint"},
        "orlrig::joint_handle",
        "Joint",
        "rotation",
        AccessMode::ReadWrite,
    });
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

TEST_CASE("graph IR cache round trips a content-addressed oro document",
    "[orlgraph][oro][cache]")
{
    NodeRegistry registry;
    REQUIRE(registry.register_definition(make_passthrough_definition()));

    GraphModule module;
    module.module_id = "oro.cache";

    const auto cache_directory =
        std::filesystem::temp_directory_path() / "orl_graph_ir_cache_test";
    std::error_code error;
    std::filesystem::remove_all(cache_directory, error);

    std::string content_hash;
    OrlGraphIrCache cache({cache_directory, false});
    REQUIRE(cache.save(module, registry, &content_hash));
    REQUIRE_FALSE(content_hash.empty());
    REQUIRE(std::filesystem::exists(cache.path(content_hash)));

    const auto loaded = cache.load(content_hash);
    for (const auto& diagnostic : loaded.diagnostics) {
        INFO(diagnostic.code << ": " << diagnostic.message);
    }
    REQUIRE(loaded.ok);
    REQUIRE(loaded.module.module_id == module.module_id);
    REQUIRE(loaded.module.nodes().size() == module.nodes().size());
    REQUIRE(loaded.registry.find(StableId{"builtin.pass"}) != nullptr);

    OrlGraphIrCache forced({cache_directory, true});
    REQUIRE_FALSE(forced.load(content_hash).ok);

    std::filesystem::remove_all(cache_directory, error);
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

TEST_CASE("exact handle graph JSON preserves identity and rejects malformed types",
    "[orlgraph][graph-json][handle]")
{
    GraphModule module;
    module.module_id = "character.handles";
    const auto handle = LogicalType::handle("orlrig::joint_handle");
    REQUIRE(module.add_input(InterfacePort{
        StableId{"joint"}, "Joint", PortDirection::Input,
        handle, Domain::rig(), Shape::scalar(), true,
        std::nullopt, false, "scene.rig.joint", "joint", {}}));

    const auto first = serialize_graph_json(module);
    const auto second = serialize_graph_json(module);
    REQUIRE(first.ok);
    REQUIRE(first.text == second.text);
    REQUIRE(first.content_hash == second.content_hash);
    REQUIRE(first.text.find("orlrig::joint_handle") != std::string::npos);

    const auto loaded = deserialize_graph_json(first.text);
    REQUIRE(loaded.ok);
    REQUIRE(loaded.module.input(StableId{"joint"})->type == handle);

    auto malformed_name = first.text;
    const auto name_position = malformed_name.find(
        "orlrig::joint_handle");
    REQUIRE(name_position != std::string::npos);
    malformed_name.replace(name_position,
        std::string{"orlrig::joint_handle"}.size(), "");
    const auto malformed = deserialize_graph_json(malformed_name);
    REQUIRE_FALSE(malformed.ok);
    REQUIRE(std::any_of(malformed.diagnostics.begin(),
        malformed.diagnostics.end(),
        [](const Diagnostic& diagnostic) {
            return diagnostic.code == "ORLGRAPH_INVALID_HANDLE_TYPE";
        }));

    auto unknown_kind = first.text;
    const auto kind_position = unknown_kind.find("\"kind\":15");
    REQUIRE(kind_position != std::string::npos);
    unknown_kind.replace(kind_position, std::string{"\"kind\":15"}.size(),
        "\"kind\":99");
    const auto unknown = deserialize_graph_json(unknown_kind);
    REQUIRE_FALSE(unknown.ok);
    REQUIRE(std::any_of(unknown.diagnostics.begin(),
        unknown.diagnostics.end(),
        [](const Diagnostic& diagnostic) {
            return diagnostic.code == "ORLGRAPH_INVALID_TYPE";
        }));

    auto nested_handle = first.text;
    const auto nested_kind_position = nested_handle.find("\"kind\":15");
    REQUIRE(nested_kind_position != std::string::npos);
    nested_handle.replace(nested_kind_position,
        std::string{"\"kind\":15"}.size(), "\"kind\":13");
    const auto element_position = nested_handle.find("\"element\":null");
    REQUIRE(element_position != std::string::npos);
    nested_handle.replace(element_position,
        std::string{"\"element\":null"}.size(),
        "\"element\":{\"kind\":12,\"name\":\"\",\"lanes\":0,"
        "\"extent\":2,\"element\":{\"kind\":15,\"name\":"
        "\"orlrig::joint_handle\",\"lanes\":0,\"extent\":0,"
        "\"element\":null}}");
    const auto nested = deserialize_graph_json(nested_handle);
    REQUIRE_FALSE(nested.ok);
    REQUIRE(std::any_of(nested.diagnostics.begin(),
        nested.diagnostics.end(),
        [](const Diagnostic& diagnostic) {
            return diagnostic.code == "ORLGRAPH_INVALID_HANDLE_TYPE";
        }));

    auto old_schema = first.text;
    const auto abi_position = old_schema.find("orlgraph-1");
    REQUIRE(abi_position != std::string::npos);
    old_schema.replace(abi_position, std::string{"orlgraph-1"}.size(),
        "orlgraph-0");
    const auto old_document = deserialize_graph_json(old_schema);
    REQUIRE_FALSE(old_document.ok);
    REQUIRE(std::any_of(old_document.diagnostics.begin(),
        old_document.diagnostics.end(),
        [](const Diagnostic& diagnostic) {
            return diagnostic.code == "ORLGRAPH_ABI_MISMATCH";
        }));
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
