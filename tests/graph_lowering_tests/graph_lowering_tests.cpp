#if __has_include(<catch2/catch_all.hpp>)

#include <catch2/catch_all.hpp>

#include <orlgraph/orlgraph.hpp>

#include "orl_graph_exec.hpp"
#include "orlrig/abi.hpp"

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

NodeDefinition make_buffer_sum_definition() {
    NodeDefinition definition;
    definition.id = StableId{"test.buffer_sum"};
    definition.qualified_name = "test.buffer_sum";
    definition.implementation.kind = ImplementationKind::OrlFunction;
    definition.implementation.function = "sum_values";
    definition.inputs = {
        Port{StableId{"values"}, "values", PortDirection::Input,
            PortCardinality::Buffer,
            LogicalType::buffer(LogicalType::int64()), Domain::buffer(),
            Shape::one("value_count"), true, std::nullopt, {}, {}},
        Port{StableId{"count"}, "count", PortDirection::Input,
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

GraphModule make_buffer_graph() {
    GraphModule module;
    module.module_id = "lowering.buffer_sum";
    module.add_input(InterfacePort{
        StableId{"values"}, "values", PortDirection::Input,
        LogicalType::buffer(LogicalType::int64()), Domain::buffer(),
        Shape::one("value_count"), true, std::nullopt, false,
        "scene.test.values", "values", {}});
    module.add_input(InterfacePort{
        StableId{"count"}, "count", PortDirection::Input,
        LogicalType::int64(), Domain::constant(), Shape::scalar(), true,
        std::nullopt, false, "scene.test.count", "count", {}});
    module.add_output(InterfacePort{
        StableId{"result"}, "result", PortDirection::Output,
        LogicalType::int64(), Domain::constant(), Shape::scalar(), true,
        std::nullopt, false, "result", {}, {}});
    module.add_node(NodeInstance{
        StableId{"sum_node"}, StableId{"test.buffer_sum"}, "sum_node", {},
        {}, InlinePolicy::Default});
    module.add_connection(Connection{
        Endpoint::graph_input(StableId{"values"}),
        Endpoint::node_port(StableId{"sum_node"}, StableId{"values"})});
    module.add_connection(Connection{
        Endpoint::graph_input(StableId{"count"}),
        Endpoint::node_port(StableId{"sum_node"}, StableId{"count"})});
    module.add_connection(Connection{
        Endpoint::node_port(StableId{"sum_node"}, StableId{"result"}),
        Endpoint::graph_output(StableId{"result"})});
    return module;
}

NodeDefinition make_scene_probe_definition() {
    NodeDefinition definition;
    definition.id = StableId{"test.scene_probe"};
    definition.qualified_name = "test.scene_probe";
    definition.implementation.kind = ImplementationKind::OrlFunction;
    definition.implementation.function = "scene_probe";
    definition.inputs = {
        Port{StableId{"positions"}, "positions", PortDirection::Input,
            PortCardinality::Buffer,
            LogicalType::buffer(LogicalType::point()), Domain::vertex(),
            Shape::one("vertex_count"), true, std::nullopt, {}, {}},
        Port{StableId{"joints"}, "joints", PortDirection::Input,
            PortCardinality::Buffer,
            LogicalType::buffer(LogicalType::struct_type("Joint")),
            Domain::joint(), Shape::one("joint_count"), true,
            std::nullopt, {}, {}},
        Port{StableId{"weights"}, "weights", PortDirection::Input,
            PortCardinality::Buffer,
            LogicalType::buffer(LogicalType::struct_type("Weight")),
            Domain::vertex(), Shape::one("weight_count"), true,
            std::nullopt, {}, {}},
        Port{StableId{"vertex_count"}, "vertex_count", PortDirection::Input,
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

GraphModule make_scene_binding_graph() {
    GraphModule module;
    module.module_id = "lowering.scene_inputs";
    module.add_input(InterfacePort{
        StableId{"scene.mesh.body.positions"}, "positions",
        PortDirection::Input, LogicalType::buffer(LogicalType::point()),
        Domain::vertex(), Shape::one("vertex_count"), true, std::nullopt,
        false, "scene.mesh.body.positions", "mesh.positions", "world"});
    module.add_input(InterfacePort{
        StableId{"scene.rig.joints"}, "joints", PortDirection::Input,
        LogicalType::buffer(LogicalType::struct_type("Joint")),
        Domain::joint(), Shape::one("joint_count"), true, std::nullopt,
        false, "scene.rig.joints", "joints", "world"});
    module.add_input(InterfacePort{
        StableId{"scene.rig.weights.body.buffer"}, "weights",
        PortDirection::Input,
        LogicalType::buffer(LogicalType::struct_type("Weight")),
        Domain::vertex(), Shape::one("weight_count"), true, std::nullopt,
        false, "scene.rig.weights.body.buffer", "weights", "world"});
    module.add_input(InterfacePort{
        StableId{"scene.mesh.body.vertex_count"}, "vertex_count",
        PortDirection::Input, LogicalType::int64(), Domain::constant(),
        Shape::scalar(), true, std::nullopt, false,
        "scene.mesh.body.vertex_count", "vertex_count", {}});
    module.add_output(InterfacePort{
        StableId{"result"}, "result", PortDirection::Output,
        LogicalType::int64(), Domain::constant(), Shape::scalar(), false,
        std::nullopt, false, "result", {}});
    module.add_node(NodeInstance{
        StableId{"probe"}, StableId{"test.scene_probe"}, "probe",
        {}, {}, InlinePolicy::Default});
    module.add_connection(Connection{
        Endpoint::graph_input(StableId{"scene.mesh.body.positions"}),
        Endpoint::node_port(StableId{"probe"}, StableId{"positions"})});
    module.add_connection(Connection{
        Endpoint::graph_input(StableId{"scene.rig.joints"}),
        Endpoint::node_port(StableId{"probe"}, StableId{"joints"})});
    module.add_connection(Connection{
        Endpoint::graph_input(StableId{"scene.rig.weights.body.buffer"}),
        Endpoint::node_port(StableId{"probe"}, StableId{"weights"})});
    module.add_connection(Connection{
        Endpoint::graph_input(StableId{"scene.mesh.body.vertex_count"}),
        Endpoint::node_port(StableId{"probe"}, StableId{"vertex_count"})});
    module.add_connection(Connection{
        Endpoint::node_port(StableId{"probe"}, StableId{"result"}),
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

TEST_CASE("graph input resolver binds scene buffers and scalars",
    "[orlgraph][lowering][binding][cpu]")
{
    NodeRegistry registry;
    REQUIRE(registry.register_definition(make_buffer_sum_definition()));
    const auto program = OrlGraphProgram::Compile(
        make_buffer_graph(), registry,
        {
            .entry_function = "graph_buffer_compute",
            .source_preamble =
                "int sum_values(int values[], int count) { "
                "return values[0] + count; }",
            .emit_module_uses = false,
        });
    REQUIRE(program.valid());

    auto execution = OrlGraphExecution::Create(program, Backend::Cpu);
    REQUIRE(execution.valid());

    OrlBuffer values("int", sizeof(std::int64_t));
    REQUIRE(values.resize(1));
    REQUIRE(values.write<std::int64_t>(0, 7));
    const GraphInputResolver resolver =
        [&values](const InterfacePort& input, GraphInputBinding& binding,
            std::string& error) {
        if (input.binding == "scene.test.values") {
            binding.kind = ParameterKind::Buffer;
            binding.buffer = &values;
            binding.element_count = values.count();
            return true;
        }
        if (input.binding == "scene.test.count") {
            binding.kind = ParameterKind::Int64;
            binding.int_value = 5;
            return true;
        }
        error = "unexpected graph input binding";
        return false;
    };
    const auto graph = make_buffer_graph();
    REQUIRE(execution.bind_graph_inputs(graph, resolver));
    const auto result = execution.evaluate();
    REQUIRE(result.has_value());
    REQUIRE(*result == 12);
}

TEST_CASE("scene input bindings reach an LBS-compatible graph node",
    "[orlgraph][lowering][binding][scene][lbs]")
{
    NodeRegistry registry;
    REQUIRE(registry.register_definition(make_scene_probe_definition()));
    const auto graph = make_scene_binding_graph();
    const auto program = OrlGraphProgram::Compile(
        graph, registry,
        {
            .entry_function = "scene_binding_compute",
            .source_preamble =
                "use joint;\n"
                "use weight;\n"
                "int scene_probe(point positions[], Joint joints[], "
                "Weight weights[], int vertex_count) { "
                "return vertex_count; }",
            .emit_module_uses = false,
        });
    REQUIRE(program.valid());

    auto execution = OrlGraphExecution::Create(program, Backend::Cpu);
    REQUIRE(execution.valid());

    OrlBuffer positions(orlrig::kPointOrlType, orlrig::kPointStride);
    OrlBuffer joints(orlrig::kJointOrlType, orlrig::kJointStride);
    OrlBuffer weights(orlrig::kWeightOrlType, orlrig::kWeightStride);
    REQUIRE(positions.resize(2));
    REQUIRE(joints.resize(1));
    REQUIRE(weights.resize(2));

    const GraphInputResolver resolver =
        [&positions, &joints, &weights](const InterfacePort& input,
            GraphInputBinding& binding, std::string& error) {
        binding.kind = ParameterKind::Buffer;
        if (input.binding == "scene.mesh.body.positions") {
            binding.buffer = &positions;
        } else if (input.binding == "scene.rig.joints") {
            binding.buffer = &joints;
        } else if (input.binding == "scene.rig.weights.body.buffer") {
            binding.buffer = &weights;
        } else if (input.binding == "scene.mesh.body.vertex_count") {
            binding.kind = ParameterKind::Int64;
            binding.int_value = 2;
            return true;
        } else {
            error = "unexpected scene input binding";
            return false;
        }
        binding.element_count = binding.buffer->count();
        return true;
    };
    REQUIRE(execution.bind_graph_inputs(graph, resolver));
    const auto result = execution.evaluate();
    REQUIRE(result.has_value());
    REQUIRE(*result == 2);
}

TEST_CASE("graph input binding rejects missing and incompatible scene data",
    "[orlgraph][lowering][binding][validation]")
{
    NodeRegistry registry;
    REQUIRE(registry.register_definition(make_buffer_sum_definition()));
    const auto graph = make_buffer_graph();
    const auto program = OrlGraphProgram::Compile(
        graph, registry,
        {
            .entry_function = "graph_binding_validation",
            .source_preamble =
                "int sum_values(int values[], int count) { "
                "return values[0] + count; }",
            .emit_module_uses = false,
        });
    REQUIRE(program.valid());
    auto execution = OrlGraphExecution::Create(program, Backend::Cpu);
    REQUIRE(execution.valid());

    const GraphInputResolver missing =
        [](const InterfacePort&, GraphInputBinding&, std::string& error) {
        error = "scene input is unavailable";
        return false;
    };
    REQUIRE_FALSE(execution.bind_graph_inputs(graph, missing));
    REQUIRE_FALSE(execution.errors().empty());
    REQUIRE(execution.errors().front().find("unavailable")
        != std::string::npos);

    OrlBuffer wrong_type("float", sizeof(double));
    REQUIRE(wrong_type.resize(1));
    const GraphInputResolver incompatible =
        [&wrong_type](const InterfacePort& input,
            GraphInputBinding& binding, std::string&) {
        if (input.binding == "scene.test.values") {
            binding.kind = ParameterKind::Buffer;
            binding.buffer = &wrong_type;
            binding.element_count = 1;
        } else {
            binding.kind = ParameterKind::Int64;
            binding.int_value = 1;
        }
        return true;
    };
    REQUIRE_FALSE(execution.bind_graph_inputs(graph, incompatible));
    REQUIRE_FALSE(execution.errors().empty());
    REQUIRE(execution.errors().front().find("ABI") != std::string::npos);
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
