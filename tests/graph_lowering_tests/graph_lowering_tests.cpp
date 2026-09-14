#if __has_include(<catch2/catch_all.hpp>)

#include <catch2/catch_all.hpp>

#include <orlgraph/orlgraph.hpp>

#include "orl_graph_exec.hpp"
#include "orlrig/abi.hpp"
#include "orlrig/graph_resources.hpp"
#include "orlrig/joint.hpp"

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

NodeDefinition make_matrix_probe_definition() {
    NodeDefinition definition;
    definition.id = StableId{"test.matrix_probe"};
    definition.qualified_name = "test.matrix_probe";
    definition.implementation.kind = ImplementationKind::OrlFunction;
    definition.implementation.function = "matrix_probe";
    definition.inputs = {
        Port{StableId{"values"}, "values", PortDirection::Input,
            PortCardinality::Buffer,
            LogicalType::buffer(LogicalType::matrix()), Domain::buffer(),
            Shape::one("one"), true, std::nullopt, {}, {}},
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

GraphModule make_implicit_joint_matrix_graph() {
    GraphModule module;
    module.module_id = "lowering.implicit_joint_matrix";
    module.add_input(InterfacePort{
        StableId{"scene.rig.joints"}, "joints", PortDirection::Input,
        LogicalType::buffer(LogicalType::struct_type("Joint")),
        Domain::joint(), Shape::one("joint_count"), true, std::nullopt,
        false, "scene.rig.joints", "joints", "world"});
    module.add_input(InterfacePort{
        StableId{"count"}, "count", PortDirection::Input,
        LogicalType::int64(), Domain::constant(), Shape::scalar(), true,
        std::nullopt, false, "count", {}, {}});
    module.add_output(InterfacePort{
        StableId{"result"}, "result", PortDirection::Output,
        LogicalType::int64(), Domain::constant(), Shape::scalar(), true,
        std::nullopt, false, "result", {}, {}});
    module.add_node(NodeInstance{
        StableId{"find_joint"}, StableId{"orlrig.input.find_joint"},
        "find_joint",
        {{"name", ConstantValue{LogicalType::string(), std::string{"root"}}}},
        {}, InlinePolicy::Default});
    module.add_node(NodeInstance{
        StableId{"probe"}, StableId{"test.matrix_probe"}, "probe",
        {}, {}, InlinePolicy::Default});
    module.add_connection(Connection{
        Endpoint::node_port(StableId{"find_joint"}, StableId{"xform"}),
        Endpoint::node_port(StableId{"probe"}, StableId{"values"})});
    module.add_connection(Connection{
        Endpoint::graph_input(StableId{"count"}),
        Endpoint::node_port(StableId{"probe"}, StableId{"count"})});
    module.add_connection(Connection{
        Endpoint::node_port(StableId{"probe"}, StableId{"result"}),
        Endpoint::graph_output(StableId{"result"})});
    return module;
}

GraphModule make_controller_to_joint_copy_graph() {
    GraphModule module;
    module.module_id = "lowering.controller_to_joint_copy";
    module.add_input(InterfacePort{
        StableId{"scene.rig.joints"}, "joints", PortDirection::Input,
        LogicalType::buffer(LogicalType::struct_type("Joint")),
        Domain::joint(), Shape::one("joint_count"), true, std::nullopt,
        false, "scene.rig.joints", "joints", "world"});
    module.add_input(InterfacePort{
        StableId{"controller_xform"}, "controller_xform",
        PortDirection::Input, LogicalType::buffer(LogicalType::matrix()),
        Domain::buffer(), Shape::one("one"), true, std::nullopt, false,
        "scene.rig.controller.ctrl.xform", "controller.xform", "world"});
    module.add_input(InterfacePort{
        StableId{"controller_count"}, "controller_count",
        PortDirection::Input, LogicalType::int64(), Domain::constant(),
        Shape::scalar(), true, std::nullopt, false,
        "controller_count", {}, {}});
    module.add_input(InterfacePort{
        StableId{"joint_count"}, "joint_count", PortDirection::Input,
        LogicalType::int64(), Domain::constant(), Shape::scalar(), true,
        std::nullopt, false, "joint_count", {}, {}});
    module.add_output(InterfacePort{
        StableId{"result"}, "result", PortDirection::Output,
        LogicalType::int64(), Domain::constant(), Shape::scalar(), true,
        std::nullopt, false, "result", {}, {}});

    module.add_node(NodeInstance{
        StableId{"find_controller"},
        StableId{"orlrig.input.find_controller"},
        "find_controller",
        {{"name", ConstantValue{
            LogicalType::string(), std::string{"ctrl"}}}},
        {}, InlinePolicy::Default});
    module.add_node(NodeInstance{
        StableId{"find_joint"}, StableId{"orlrig.input.find_joint"},
        "find_joint",
        {{"name", ConstantValue{
            LogicalType::string(), std::string{"joint"}}}},
        {}, InlinePolicy::Default});
    module.add_node(NodeInstance{
        StableId{"copy"}, StableId{"orlrig.constraint.copy_xform"},
        "copy", {}, {}, InlinePolicy::Default});

    module.add_connection(Connection{
        Endpoint::node_port(StableId{"find_controller"},
            StableId{"xform"}),
        Endpoint::node_port(StableId{"copy"}, StableId{"source"})});
    module.add_connection(Connection{
        Endpoint::node_port(StableId{"find_joint"},
            StableId{"xform"}),
        Endpoint::node_port(StableId{"copy"}, StableId{"destination"})});
    module.add_connection(Connection{
        Endpoint::graph_input(StableId{"controller_count"}),
        Endpoint::node_port(StableId{"copy"},
            StableId{"source_count"})});
    module.add_connection(Connection{
        Endpoint::graph_input(StableId{"joint_count"}),
        Endpoint::node_port(StableId{"copy"},
            StableId{"destination_count"})});
    module.add_connection(Connection{
        Endpoint::node_port(StableId{"copy"}, StableId{"status"}),
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
    const auto typed = execution.evaluate_result();
    REQUIRE(typed.ok);
    REQUIRE(typed.status.has_value());
    REQUIRE(*typed.status == 18);
    REQUIRE(typed.output(StableId{"result"}) != nullptr);
}

TEST_CASE("graph lowering imports locator types used by graph signatures",
    "[orlgraph][lowering][locator]")
{
    NodeRegistry registry;
    NodeDefinition definition;
    definition.id = StableId{"test.locator_count"};
    definition.qualified_name = "test.locator_count";
    definition.implementation.kind = ImplementationKind::OrlFunction;
    definition.implementation.function = "locator_count";
    definition.inputs = {
        Port{StableId{"locators"}, "locators", PortDirection::Input,
            PortCardinality::Buffer,
            LogicalType::buffer(LogicalType::struct_type("Locator")),
            Domain::rig(), Shape::one("locator_count"), true,
            std::nullopt, "locators", "world"},
    };
    definition.outputs = {
        Port{StableId{"status"}, "status", PortDirection::Output,
            PortCardinality::Scalar, LogicalType::int64(), Domain::constant(),
            Shape::scalar(), false, std::nullopt, "status", {}, {}},
    };
    REQUIRE(registry.register_definition(std::move(definition)));

    GraphModule module;
    module.module_id = "locator.signature";
    REQUIRE(module.add_input(InterfacePort{
        StableId{"locators"}, "locators", PortDirection::Input,
        LogicalType::buffer(LogicalType::struct_type("Locator")),
        Domain::rig(), Shape::one("locator_count"), true, std::nullopt,
        false, "scene.rig.locators", "locators", "world"}));
    REQUIRE(module.add_output(InterfacePort{
        StableId{"status"}, "status", PortDirection::Output,
        LogicalType::int64(), Domain::constant(), Shape::scalar(), true,
        std::nullopt, false, "status", {}, {}}));
    REQUIRE(module.add_node(NodeInstance{
        StableId{"count"}, StableId{"test.locator_count"}, "count",
        {}, {}, InlinePolicy::Default}));
    REQUIRE(module.add_connection(Connection{
        Endpoint::graph_input(StableId{"locators"}),
        Endpoint::node_port(StableId{"count"}, StableId{"locators"})}));
    REQUIRE(module.add_connection(Connection{
        Endpoint::node_port(StableId{"count"}, StableId{"status"}),
        Endpoint::graph_output(StableId{"status"})}));

    const auto lowered = orlcomp::OrlGraphLowerer{}.lower(module, registry, {
        .entry_function = "locator_signature",
        .source_preamble =
            "int locator_count(Locator locators[], int count) { return count; }",
    });
    REQUIRE(lowered.ok);
    REQUIRE(lowered.source.find("use locator;") != std::string::npos);
    REQUIRE(lowered.source.find("Locator locators[]") != std::string::npos);
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

TEST_CASE("typed graph outputs expose scalar status and buffer aliases",
    "[orlgraph][lowering][outputs][cpu]")
{
    NodeRegistry registry;
    REQUIRE(registry.register_definition(make_buffer_sum_definition()));
    auto graph = make_buffer_graph();

    InterfacePort values_output;
    values_output.id = StableId{"values_output"};
    values_output.name = "values_output";
    values_output.direction = PortDirection::Output;
    values_output.type = LogicalType::buffer(LogicalType::int64());
    values_output.domain = Domain::buffer();
    values_output.shape = Shape::one("value_count");
    values_output.required = false;
    values_output.binding = "values_output";
    values_output.semantic = "values";
    REQUIRE(graph.add_output(values_output));
    REQUIRE(graph.add_connection(Connection{
        Endpoint::graph_input(StableId{"values"}),
        Endpoint::graph_output(StableId{"values_output"})}));
    InterfacePort exposure_input;
    exposure_input.id = StableId{"exposure"};
    exposure_input.name = "exposure";
    exposure_input.direction = PortDirection::Input;
    exposure_input.type = LogicalType::float64();
    exposure_input.domain = Domain::constant();
    exposure_input.shape = Shape::scalar();
    exposure_input.required = true;
    exposure_input.binding = "exposure";
    REQUIRE(graph.add_input(exposure_input));
    InterfacePort exposure_output;
    exposure_output.id = StableId{"exposure_output"};
    exposure_output.name = "exposure_output";
    exposure_output.direction = PortDirection::Output;
    exposure_output.type = LogicalType::float64();
    exposure_output.domain = Domain::constant();
    exposure_output.shape = Shape::scalar();
    exposure_output.required = false;
    exposure_output.binding = "exposure_output";
    REQUIRE(graph.add_output(exposure_output));
    REQUIRE(graph.add_connection(Connection{
        Endpoint::graph_input(StableId{"exposure"}),
        Endpoint::graph_output(StableId{"exposure_output"})}));

    const auto program = OrlGraphProgram::Compile(
        graph, registry,
        {
            .entry_function = "typed_output_compute",
            .source_preamble =
                "int sum_values(int values[], int count) { "
                "return values[0] + count; }",
            .emit_module_uses = false,
        });
    REQUIRE(program.valid());
    REQUIRE(program.outputs().size() == 3);

    auto execution = OrlGraphExecution::Create(program, Backend::Cpu);
    REQUIRE(execution.valid());
    OrlBuffer values("int", sizeof(std::int64_t));
    REQUIRE(values.resize(1));
    REQUIRE(values.write<std::int64_t>(0, 7));
    REQUIRE(execution.bind_int("count", 5));
    REQUIRE(execution.bind_float("exposure", 2.5));
    REQUIRE(execution.bind_buffer("values", values));

    const auto result = execution.evaluate_result();
    REQUIRE(result.ok);
    REQUIRE(result.status.has_value());
    REQUIRE(*result.status == 12);
    const auto* scalar = result.output(StableId{"result"});
    REQUIRE(scalar != nullptr);
    REQUIRE(scalar->int_value.has_value());
    REQUIRE(*scalar->int_value == 12);
    const auto* buffer = result.output(StableId{"values_output"});
    REQUIRE(buffer != nullptr);
    REQUIRE(buffer->buffer == &values);
    const auto* exposure = result.output(StableId{"exposure_output"});
    REQUIRE(exposure != nullptr);
    REQUIRE(exposure->float_value.has_value());
    REQUIRE(*exposure->float_value == Catch::Approx(2.5));
    REQUIRE(execution.output(StableId{"values_output"}) != nullptr);
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

TEST_CASE("implicit output adapter materializes a joint world matrix buffer",
    "[orlgraph][lowering][conversion]")
{
    auto rig = orlrig::make_lbs_graph();
    NodeRegistry registry = std::move(rig.registry);
    REQUIRE(registry.register_definition(make_matrix_probe_definition()));

    auto options = orlcomp::GraphLoweringOptions{};
    options.entry_function = "implicit_joint_matrix";
    options.scene_revision = 17;
    options.source_preamble =
        "int matrix_probe(matrix values[], int count) { return count; }";
    options.runtime_output_expression =
        [](const NodeInstance&, const NodeDefinition& definition,
            const Port& output) -> std::optional<std::string> {
        if (definition.qualified_name == "orlrig.input.find_joint"
            && output.name == "handle")
        {
            return std::string{"2001"};
        }
        if (definition.qualified_name == "orlrig.input.find_joint"
            && output.name == "index")
        {
            return std::string{"0"};
        }
        return std::nullopt;
    };

    const auto graph = make_implicit_joint_matrix_graph();
    const auto lowered = orlcomp::OrlGraphLowerer{}.lower(
        graph, registry, options);
    REQUIRE(lowered.ok);
    REQUIRE(lowered.scene_revision == 17);
    REQUIRE(lowered.source.find("matrix conversion_find_joint_xform[1];")
        != std::string::npos);
    REQUIRE(lowered.source.find(
        "conversion_find_joint_xform[0] = joint_world_matrix(joints, 0);")
        != std::string::npos);
    REQUIRE(lowered.source.find(
        "matrix_probe(conversion_find_joint_xform, count)")
        != std::string::npos);

    const auto program = OrlGraphProgram::Compile(graph, registry, options);
    REQUIRE(program.valid());
    REQUIRE(program.scene_revision() == 17);
    REQUIRE(program.scene_revision_matches(17));
    REQUIRE_FALSE(program.scene_revision_matches(18));
}

TEST_CASE("writable joint adapter commits a controller copy as local TRS",
    "[orlgraph][lowering][conversion][writeback]")
{
    auto rig = orlrig::make_lbs_graph();
    NodeRegistry registry = std::move(rig.registry);

    auto options = orlcomp::GraphLoweringOptions{};
    options.entry_function = "controller_to_joint_copy";
    options.runtime_output_expression =
        [](const NodeInstance&, const NodeDefinition& definition,
            const Port& output) -> std::optional<std::string> {
        if (definition.qualified_name == "orlrig.input.find_joint"
            && output.name == "handle")
        {
            return std::string{"2001"};
        }
        if (definition.qualified_name == "orlrig.input.find_joint"
            && output.name == "index")
        {
            return std::string{"1"};
        }
        if (definition.qualified_name == "orlrig.input.find_controller") {
            if (output.name == "handle") {
                return std::string{"1001"};
            }
            if (output.name == "index") {
                return std::string{"0"};
            }
            if (output.name == "xform") {
                return std::string{"controller_xform"};
            }
        }
        return std::nullopt;
    };
    options.runtime_handle_index_expression =
        [](const NodeInstance&, const NodeDefinition&,
            const Port&, std::string_view handle_expression)
            -> std::optional<std::string> {
        if (handle_expression == "2001") {
            return std::string{"1"};
        }
        return std::nullopt;
    };

    const auto graph = make_controller_to_joint_copy_graph();
    const auto program = OrlGraphProgram::Compile(graph, registry, options);
    REQUIRE(program.valid());
    REQUIRE(program.source().find(
        "joint_write_world_matrix(joints, 1, "
        "conversion_find_joint_xform);") != std::string::npos);

    auto execution = OrlGraphExecution::Create(program, Backend::Cpu);
    REQUIRE(execution.valid());

    OrlBuffer joints(orlrig::kJointOrlType, orlrig::kJointStride);
    REQUIRE(joints.resize(2));
    auto* joint = static_cast<orlrig::Joint*>(joints.data());
    joint[0] = orlrig::make_identity_joint();
    joint[0].translation[0] = 2.0;
    joint[1] = orlrig::make_identity_joint();
    joint[1].parent = 0;

    OrlBuffer controller(orlrig::kMatrixOrlType, orlrig::kMatrixStride);
    REQUIRE(controller.resize(1));
    auto* controller_matrix = static_cast<double*>(controller.data());
    controller_matrix[0] = 1.0;
    controller_matrix[5] = 1.0;
    controller_matrix[10] = 1.0;
    controller_matrix[15] = 1.0;
    controller_matrix[3] = 6.0;

    const GraphInputResolver resolver =
        [&joints, &controller](const InterfacePort& input,
            GraphInputBinding& binding, std::string& error) {
        if (input.binding == "scene.rig.joints") {
            binding.kind = ParameterKind::Buffer;
            binding.buffer = &joints;
            binding.element_count = joints.count();
            return true;
        }
        if (input.binding == "scene.rig.controller.ctrl.xform") {
            binding.kind = ParameterKind::Buffer;
            binding.buffer = &controller;
            binding.element_count = controller.count();
            return true;
        }
        if (input.binding == "controller_count"
            || input.binding == "joint_count")
        {
            binding.kind = ParameterKind::Int64;
            binding.int_value = 1;
            return true;
        }
        error = "unexpected writable graph input";
        return false;
    };
    REQUIRE(execution.bind_graph_inputs(graph, resolver));
    const auto result = execution.evaluate();
    REQUIRE(result.has_value());
    REQUIRE(*result == 1);

    REQUIRE(joint[1].translation[0] == Catch::Approx(4.0));
    REQUIRE(joint[1].translation[1] == Catch::Approx(0.0));
    REQUIRE(joint[1].translation[2] == Catch::Approx(0.0));
    REQUIRE(joint[1].rotation[3] == Catch::Approx(1.0));
    REQUIRE(joint[1].scale[0] == Catch::Approx(1.0));
    REQUIRE(joint[1].parent == 0);
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
