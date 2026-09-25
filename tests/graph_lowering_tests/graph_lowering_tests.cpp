#if __has_include(<catch2/catch_all.hpp>)

#include <catch2/catch_all.hpp>

#include <orlgraph/orlgraph.hpp>

#include "orl_graph_exec.hpp"
#include "orl_graph_lowering.h"
#include "orlrig/abi.hpp"
#include "orlrig/graph_resources.hpp"
#include "orlrig/handle_registry.hpp"
#include "orlrig/joint.hpp"
#include "orlrig/locator.hpp"

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

TEST_CASE("graph lowering preserves exact handle lane inputs and outputs",
    "[orl][graph][lowering][handle]")
{
    GraphModule module;
    module.module_id = "lowering.handle";
    const auto handle = LogicalType::handle("orlrig::joint_handle");
    REQUIRE(module.add_input(InterfacePort{
        StableId{"handle"}, "handle", PortDirection::Input,
        handle, Domain::rig(), Shape::scalar(), true,
        std::nullopt, false, "scene.rig.handle", {}, {}}));
    REQUIRE(module.add_output(InterfacePort{
        StableId{"result"}, "result", PortDirection::Output,
        handle, Domain::rig(), Shape::scalar(), true,
        std::nullopt, false, "result", {}, {}}));
    REQUIRE(module.add_connection(Connection{
        Endpoint::graph_input(StableId{"handle"}),
        Endpoint::graph_output(StableId{"result"})}));

    const auto lowered = orlcomp::OrlGraphLowerer{}.lower(module,
        NodeRegistry{}, orlcomp::GraphLoweringOptions{
            .entry_function = "handle_graph",
            .emit_module_uses = false,
        });
    REQUIRE(lowered.ok);
    REQUIRE(lowered.source.find(
        "handle __orl_handle_orlrig__joint_handle;")
        != std::string::npos);
    REQUIRE(lowered.source.find(
        "int handle_graph(__orl_handle_orlrig__joint_handle _handle)")
        != std::string::npos);
    REQUIRE(lowered.outputs.size() == 1);
    REQUIRE(lowered.outputs.front().type == handle);
    REQUIRE(lowered.outputs.front().source_parameter == "_handle");
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

TEST_CASE("typed scene input lowers as one nominal handle socket",
    "[orlgraph][lowering][handle]")
{
    auto rig = orlrig::make_lbs_graph();
    NodeRegistry registry = std::move(rig.registry);

    GraphModule graph;
    graph.module_id = "lowering.typed_scene_handle";
    const auto joint_handle =
        LogicalType::handle("orlrig::joint_handle");
    REQUIRE(graph.add_input(InterfacePort{
        StableId{"joint_input"}, "joint_input", PortDirection::Input,
        joint_handle, Domain::rig(), Shape::scalar(), true,
        std::nullopt, false, "scene.rig.joint.root.handle",
        "scene.joint.handle", {}}));
    REQUIRE(graph.add_output(InterfacePort{
        StableId{"result"}, "result", PortDirection::Output,
        joint_handle, Domain::rig(), Shape::scalar(), true,
        std::nullopt, false, "result", "scene.joint.handle", {}}));
    REQUIRE(graph.add_node(NodeInstance{
        StableId{"find_joint"}, StableId{"orlrig.input.find_joint"},
        "find_joint",
        {{"name", ConstantValue{
            LogicalType::string(), std::string{"root"}}}},
        {}, InlinePolicy::Default}));
    REQUIRE(graph.add_connection(Connection{
        Endpoint::node_port(StableId{"find_joint"},
            StableId{"handle"}),
        Endpoint::graph_output(StableId{"result"})}));

    orlcomp::GraphLoweringOptions options;
    options.entry_function = "typed_scene_handle";
    options.emit_module_uses = false;
    options.runtime_output_expression =
        [](const NodeInstance&, const NodeDefinition& definition,
            const Port& output) -> std::optional<std::string> {
        if (definition.qualified_name == "orlrig.input.find_joint"
            && output.name == "handle")
        {
            return std::string{"joint_input"};
        }
        return std::nullopt;
    };

    const auto lowered = orlcomp::OrlGraphLowerer{}.lower(
        graph, registry, options);
    REQUIRE(lowered.ok);
    REQUIRE(lowered.source.find(
        "handle __orl_handle_orlrig__joint_handle;")
        != std::string::npos);
    REQUIRE(lowered.source.find(
        "int typed_scene_handle("
        "__orl_handle_orlrig__joint_handle joint_input)")
        != std::string::npos);

    const auto program = OrlGraphProgram::Compile(graph, registry, options);
    REQUIRE(program.valid());
    REQUIRE(program.parameters().size() == 1);
    REQUIRE(program.parameters().front().kind == ParameterKind::Handle);
}

TEST_CASE("typed solver graph writes through handle view context",
    "[orlgraph][lowering][handle][solver]")
{
    NodeRegistry registry;
    NodeDefinition solver;
    solver.id = StableId{"test.typed_ik"};
    solver.qualified_name = "test.typed_ik";
    solver.allowed_stages = GraphStageMask::Solver;
    solver.implementation.kind = ImplementationKind::OrlFunction;
    solver.implementation.module = "solver/ik_two_bone";
    solver.implementation.function = "solver_ik_two_bone";
    const auto add_input = [&solver](
        std::string name, LogicalType type) {
        Port port;
        port.id = StableId{name};
        port.name = std::move(name);
        port.direction = PortDirection::Input;
        port.cardinality = PortCardinality::Scalar;
        port.type = std::move(type);
        port.domain = Domain::rig();
        port.shape = Shape::scalar();
        port.required = true;
        solver.inputs.push_back(std::move(port));
    };
    add_input("root", LogicalType::handle("orlrig::joint_handle"));
    add_input("mid", LogicalType::handle("orlrig::joint_handle"));
    add_input("end", LogicalType::handle("orlrig::joint_handle"));
    add_input("target", LogicalType::handle("orlrig::locator_handle"));
    add_input("pole", LogicalType::handle("orlrig::locator_handle"));
    solver.outputs.push_back(Port{
        StableId{"status"}, "status", PortDirection::Output,
        PortCardinality::Scalar, LogicalType::int64(),
        Domain::constant(), Shape::scalar(), false, std::nullopt,
        "status", {}});
    REQUIRE(registry.register_definition(std::move(solver)));

    GraphModule graph;
    graph.module_id = "lowering.typed_ik_execution";
    const auto add_graph_input = [&graph](
        std::string name, LogicalType type) {
        return graph.add_input(InterfacePort{
            StableId{name}, name, PortDirection::Input, std::move(type),
            Domain::rig(), Shape::scalar(), true, std::nullopt, false,
            "scene.rig." + name + ".handle", "scene." + name + ".handle",
            {}});
    };
    REQUIRE(add_graph_input(
        "root", LogicalType::handle("orlrig::joint_handle")));
    REQUIRE(add_graph_input(
        "mid", LogicalType::handle("orlrig::joint_handle")));
    REQUIRE(add_graph_input(
        "end", LogicalType::handle("orlrig::joint_handle")));
    REQUIRE(add_graph_input(
        "target", LogicalType::handle("orlrig::locator_handle")));
    REQUIRE(add_graph_input(
        "pole", LogicalType::handle("orlrig::locator_handle")));
    REQUIRE(graph.add_output(InterfacePort{
        StableId{"result"}, "result", PortDirection::Output,
        LogicalType::int64(), Domain::constant(), Shape::scalar(), true,
        std::nullopt, false, "result", {}, {}}));
    REQUIRE(graph.add_node(NodeInstance{
        StableId{"solver"}, StableId{"test.typed_ik"}, "solver",
        {}, {}, InlinePolicy::Default}));
    for (const auto& name : {"root", "mid", "end", "target", "pole"}) {
        REQUIRE(graph.add_connection(Connection{
            Endpoint::graph_input(StableId{name}),
            Endpoint::node_port(StableId{"solver"}, StableId{name})}));
    }
    REQUIRE(graph.add_connection(Connection{
        Endpoint::node_port(StableId{"solver"}, StableId{"status"}),
        Endpoint::graph_output(StableId{"result"})}));

    const auto program = OrlGraphProgram::Compile(graph, registry);
    REQUIRE(program.valid());
    auto execution = OrlGraphExecution::Create(program, Backend::Cpu);
    REQUIRE(execution.valid());

    OrlBuffer joints(orlrig::kJointOrlType, orlrig::kJointStride);
    REQUIRE(joints.resize(3));
    auto root = orlrig::make_identity_joint();
    auto mid = orlrig::make_identity_joint();
    mid.parent = 0;
    mid.translation[0] = 1.0;
    auto end = orlrig::make_identity_joint();
    end.parent = 1;
    end.translation[0] = 1.0;
    REQUIRE(joints.write(0, root));
    REQUIRE(joints.write(1, mid));
    REQUIRE(joints.write(2, end));
    OrlBuffer locators(orlrig::kLocatorOrlType, orlrig::kLocatorStride);
    REQUIRE(locators.resize(2));
    const double target_xform[16] = {
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 1.5,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0,
    };
    const double pole_xform[16] = {
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 1.0,
        0.0, 0.0, 0.0, 1.0,
    };
    REQUIRE(locators.write(0, target_xform, sizeof(target_xform)));
    REQUIRE(locators.write(1, pole_xform, sizeof(pole_xform)));

    const auto joint_type =
        orlcomp::HandleTypeIdFor("orlrig::joint_handle");
    const auto locator_type =
        orlcomp::HandleTypeIdFor("orlrig::locator_handle");
    const GraphInputResolver resolver =
        [joint_type, locator_type](
            const InterfacePort& input, GraphInputBinding& binding,
            std::string&) {
        binding.kind = ParameterKind::Handle;
        if (input.name == "target" || input.name == "pole") {
            binding.handle_value = {
                locator_type, input.name == "target" ? 0 : 1};
        } else {
            const std::int64_t slot = input.name == "root"
                ? 0 : input.name == "mid" ? 1 : 2;
            binding.handle_value = {joint_type, slot};
        }
        return true;
    };
    REQUIRE(execution.bind_graph_inputs(graph, resolver));
    orlrig::HandleViewContext context{
        {joints.data(), joints.count(), orlrig::kJointStride, true},
        {locators.data(), locators.count(), orlrig::kLocatorStride, false},
        1};
    REQUIRE(execution.bind_handle_view_context(context));
    const auto result = execution.evaluate();
    REQUIRE(result.has_value());
    REQUIRE(*result == 1);
    REQUIRE(static_cast<orlrig::Joint*>(joints.data())->rotation[3]
        != Catch::Approx(1.0));
}

TEST_CASE("graph specialization folds union handle is branch",
    "[orlgraph][lowering][handle][union][is]")
{
    NodeRegistry registry;
    NodeDefinition generic;
    generic.id = StableId{"test.generic_handle"};
    generic.qualified_name = "test.generic_handle";
    generic.implementation.kind = ImplementationKind::OrlFunction;
    generic.implementation.function = "generic_handle";
    generic.inputs.push_back(Port{
        StableId{"value"}, "value", PortDirection::Input,
        PortCardinality::Scalar,
        LogicalType::handle_union(
            "orl_graph_program::component_handle",
            {"orl_graph_program::joint_handle",
                "orl_graph_program::locator_handle"}),
        Domain::rig(), Shape::scalar(), true, std::nullopt, {}, {}});
    generic.outputs.push_back(Port{
        StableId{"result"}, "result", PortDirection::Output,
        PortCardinality::Scalar, LogicalType::int64(),
        Domain::constant(), Shape::scalar(), false, std::nullopt, {}, {}});
    REQUIRE(registry.register_definition(std::move(generic)));

    GraphModule graph;
    graph.module_id = "lowering.union_specialization";
    const auto joint_type =
        LogicalType::handle("orl_graph_program::joint_handle");
    REQUIRE(graph.add_input(InterfacePort{
        StableId{"value"}, "value", PortDirection::Input, joint_type,
        Domain::rig(), Shape::scalar(), true, std::nullopt, false,
        "scene.rig.joint.root.handle", "scene.joint.handle", {}}));
    REQUIRE(graph.add_output(InterfacePort{
        StableId{"result"}, "result", PortDirection::Output,
        LogicalType::int64(), Domain::constant(), Shape::scalar(), true,
        std::nullopt, false, "result", {}, {}}));
    REQUIRE(graph.add_node(NodeInstance{
        StableId{"generic"}, StableId{"test.generic_handle"},
        "generic", {}, {}, InlinePolicy::Default}));
    REQUIRE(graph.add_connection(Connection{
        Endpoint::graph_input(StableId{"value"}),
        Endpoint::node_port(StableId{"generic"}, StableId{"value"})}));
    REQUIRE(graph.add_connection(Connection{
        Endpoint::node_port(StableId{"generic"}, StableId{"result"}),
        Endpoint::graph_output(StableId{"result"})}));

    orlcomp::GraphLoweringOptions options;
    options.entry_function = "union_specialization";
    options.source_preamble = R"(
        handle joint_handle;
        handle locator_handle;
        handle component_handle = joint_handle | locator_handle;
        int generic_handle(component_handle value) {
            if (value is joint_handle) {
                return 1;
            }
            return 2;
        }
    )";
    const auto program = OrlGraphProgram::Compile(
        graph, registry, std::move(options));
    REQUIRE(program.valid());
    REQUIRE(program.parameters().size() == 1);

    auto execution = OrlGraphExecution::Create(program, Backend::Cpu);
    REQUIRE(execution.valid());
    const GraphInputResolver resolver =
        [](const InterfacePort&, GraphInputBinding& binding, std::string&) {
        binding.kind = ParameterKind::Handle;
        binding.handle_value = {
            orlcomp::HandleTypeIdFor(
                "orl_graph_program::joint_handle"), 0};
        return true;
    };
    REQUIRE(execution.bind_graph_inputs(graph, resolver));
    const auto result = execution.evaluate();
    REQUIRE(result.has_value());
    REQUIRE(*result == 1);
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
