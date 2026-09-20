#if __has_include(<catch2/catch_all.hpp>)

#include <algorithm>
#include <string>

#include <catch2/catch_all.hpp>

#include "orlrig/graph_resources.hpp"

using namespace orlrig;
using namespace orlgraph;

TEST_CASE("standard LBS rig graph exposes resources and dependencies",
    "[orlrig][graph]")
{
    const RigGraph graph = make_lbs_graph();
    const auto validation = graph.validate();
    REQUIRE(validation.ok());
    REQUIRE(validation.schedule.order.size() == 2);
    REQUIRE(validation.schedule.order[0] == StableId{"capture_bind"});
    REQUIRE(validation.schedule.order[1] == StableId{"deform"});

    REQUIRE(graph.module.resource(graph.resources.joints) != nullptr);
    REQUIRE(graph.module.resource(graph.resources.weights) != nullptr);
    REQUIRE(graph.module.resource(graph.resources.posed_positions) != nullptr);
    REQUIRE(graph.module.outputs().contains(graph.resources.posed_positions));
    REQUIRE(graph.module.input(graph.resources.bind_positions)->binding
        == "bind_positions");
    REQUIRE(graph.module.input(graph.resources.joints) == nullptr);
    REQUIRE(graph.module.input(graph.resources.weights)->binding == "weights");
}

TEST_CASE("scene input bindings use stable semantic names",
    "[orlrig][graph][scene]")
{
    REQUIRE(std::string{kSceneJointsBinding} == "scene.rig.joints");
    REQUIRE(std::string{kSceneJointCountBinding} == "scene.rig.joint_count");
    REQUIRE(scene_mesh_positions_binding("body")
        == "scene.mesh.body.positions");
    REQUIRE(scene_mesh_vertex_count_binding("body")
        == "scene.mesh.body.vertex_count");
    REQUIRE(scene_weight_buffer_binding("weights")
        == "scene.rig.weights.weights.buffer");
    REQUIRE(scene_weight_count_binding("weights")
        == "scene.rig.weights.weights.weight_count");
    REQUIRE(scene_inverse_bindings_binding("deformer")
        == "scene.rig.deformer.deformer.inverse_binds");
    REQUIRE(scene_controller_xform_binding("ctrl")
        == "scene.rig.controller.ctrl.xform");
    REQUIRE(scene_controller_count_binding("ctrl")
        == "scene.rig.controller.ctrl.count");
    REQUIRE(scene_locator_xform_binding("loc")
        == "scene.rig.locator.loc.xform");
    REQUIRE(scene_locator_count_binding("loc")
        == "scene.rig.locator.loc.count");
}

TEST_CASE("standard rig graph registers public stdlib nodes",
    "[orlrig][graph][stdlib]")
{
    const RigGraph graph = make_lbs_graph();
    const std::vector<std::string> definitions{
        "orlrig.auto_weight.closest_joint",
        "orlrig.auto_weight.closest_distance",
        "orlrig.auto_weight.closest_hierarchy",
        "orlrig.auto_weight.envelope",
        "orlrig.auto_weight.heat",
        "orlrig.auto_weight.geodesic",
        "orlrig.auto_weight.harmonic",
        "orlrig.auto_weight.bounded_biharmonic",
        "orlrig.solver.fk",
        "orlrig.solver.ik_two_bone",
        "orlrig.solver.hd_id",
        "orlrig.solver.spline_ik",
        "orlrig.solver.full_body_ik",
        "orlrig.constraint.aim",
        "orlrig.constraint.aim_locator",
        "orlrig.constraint.copy_xform",
        "orlrig.constraint.copy_translation",
        "orlrig.constraint.copy_rotation",
        "orlrig.constraint.copy_scale",
        "orlrig.input.find_joint",
        "orlrig.input.find_controller",
        "orlrig.input.find_locator",
        "orlrig.input.find_mesh",
        "orlrig.stage.computed_joints",
    };

    for (const auto& name : definitions) {
        const auto* definition = graph.registry.find(name);
        REQUIRE(definition != nullptr);
        const bool is_scene_input = name.rfind("orlrig.input.", 0) == 0;
        const bool is_stage_boundary = name.rfind("orlrig.stage.", 0) == 0;
        if (is_stage_boundary) {
            REQUIRE(definition->implementation.kind
                == ImplementationKind::Runtime);
            REQUIRE(definition->allowed_stages
                == GraphStageMask::Deformer);
            REQUIRE(definition->inputs.empty());
            REQUIRE(definition->outputs.size() == 1);
            REQUIRE(definition->output("joints") != nullptr);
            continue;
        }
        REQUIRE(definition->implementation.kind
            == (is_scene_input
                ? ImplementationKind::Runtime
                : ImplementationKind::OrlFunction));
        if (is_scene_input) {
            REQUIRE(definition->allowed_stages == GraphStageMask::All);
            const bool is_find_input =
                name.rfind("orlrig.input.find_", 0) == 0;
            if (!is_find_input) {
                REQUIRE(definition->outputs.size() == 1);
                continue;
            }
            const bool has_transform =
                name == "orlrig.input.find_joint"
                || name == "orlrig.input.find_controller"
                || name == "orlrig.input.find_locator";
            REQUIRE(definition->outputs.size() == (has_transform ? 3 : 1));
            REQUIRE(definition->output("handle") != nullptr);
            if (has_transform) {
                REQUIRE(definition->output("index") != nullptr);
                REQUIRE(definition->output("xform") != nullptr);
                REQUIRE(definition->output("index")->semantic
                    == std::string{kSceneArrayIndexSemantic});
            }
            REQUIRE(definition->parameter("name") != nullptr);
        } else {
            REQUIRE(definition->allowed_stages
                == (name.rfind("orlrig.solver.", 0) == 0
                    || name.rfind("orlrig.constraint.", 0) == 0
                    ? GraphStageMask::Solver
                    : GraphStageMask::Deformer));
            REQUIRE(definition->outputs.size() == 1);
            REQUIRE(definition->outputs.front().name == "status");
            if (name.rfind("orlrig.solver.", 0) == 0) {
                REQUIRE(definition->parameter("joint_count") == nullptr);
                REQUIRE(definition->input("joints") == nullptr);
            }
        }
    }

    const auto* conversion = graph.registry.find_conversion(
        kJointWorldMatrixConversion);
    REQUIRE(conversion != nullptr);
    REQUIRE(conversion->source.type == LogicalType::int64());
    REQUIRE(conversion->source.semantic
        == std::string{kSceneArrayIndexSemantic});
    REQUIRE(conversion->output.type
        == LogicalType::buffer(LogicalType::matrix()));
    REQUIRE(conversion->auxiliary_inputs.size() == 1);
    REQUIRE(conversion->auxiliary_inputs.front().semantic
        == std::string{kSceneJointsBinding});
    const auto* writeback = graph.registry.find_conversion(
        kJointWorldMatrixWritebackConversion);
    REQUIRE(writeback != nullptr);
    REQUIRE(writeback->source.type
        == LogicalType::buffer(LogicalType::matrix()));
    REQUIRE(writeback->output.type
        == LogicalType::buffer(LogicalType::struct_type("Joint")));
    REQUIRE(writeback->selector.has_value());
    REQUIRE(writeback->selector->type == LogicalType::int64());
    REQUIRE(writeback->selector->semantic
        == std::string{kSceneJointHandleSemantic});
    REQUIRE(writeback->emitter
        == ConversionEmitterKind::OrlFunctionWriteback);

    const auto* find_joint = graph.registry.find("orlrig.input.find_joint");
    REQUIRE(find_joint != nullptr);
    const auto* xform = find_joint->output("xform");
    REQUIRE(xform != nullptr);
    REQUIRE(xform->output_adapter.has_value());
    REQUIRE(xform->output_adapter->conversion
        == StableId{std::string{kJointWorldMatrixConversion}});
    REQUIRE(xform->output_adapter->source_port == StableId{"index"});
    REQUIRE(xform->output_adapter->writeback_conversion
        == StableId{std::string{kJointWorldMatrixWritebackConversion}});
    REQUIRE(xform->output_adapter->writeback_source_port
        == StableId{"handle"});
}

TEST_CASE("scene handle and packed index semantics are not interchangeable",
    "[orlrig][graph][semantics]")
{
    auto rig = make_lbs_graph();
    NodeDefinition sink;
    sink.id = StableId{"test.index_sink"};
    sink.qualified_name = "test.index_sink";
    sink.implementation.kind = ImplementationKind::Runtime;
    sink.implementation.runtime_name = "test.index_sink";
    Port input;
    input.id = StableId{"index"};
    input.name = "index";
    input.direction = PortDirection::Input;
    input.type = LogicalType::int64();
    input.semantic = std::string{kSceneArrayIndexSemantic};
    sink.inputs.push_back(std::move(input));
    REQUIRE(rig.registry.register_definition(std::move(sink)));

    const auto validate_connection =
        [&rig](StableId output) {
            GraphModule module;
            module.add_node(NodeInstance{
                StableId{"find"}, StableId{"orlrig.input.find_controller"},
                "find",
                {{"name", ConstantValue{
                    LogicalType::string(), std::string{"ctrl"}}}},
                {}, InlinePolicy::Default});
            module.add_node(NodeInstance{
                StableId{"sink"}, StableId{"test.index_sink"}, "sink",
                {}, {}, InlinePolicy::Default});
            REQUIRE(module.add_connection(Connection{
                Endpoint::node_port(StableId{"find"}, std::move(output)),
                Endpoint::node_port(StableId{"sink"}, StableId{"index"})}));
            return validate(module, rig.registry);
        };

    const auto handle_validation =
        validate_connection(StableId{"handle"});
    REQUIRE_FALSE(handle_validation.ok());
    REQUIRE(std::any_of(handle_validation.diagnostics.begin(),
        handle_validation.diagnostics.end(),
        [](const auto& diagnostic) {
            return diagnostic.code == "ORLGRAPH_SEMANTIC_MISMATCH";
        }));

    const auto index_validation =
        validate_connection(StableId{"index"});
    REQUIRE(index_validation.ok());
}

#endif
