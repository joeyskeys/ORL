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
        "orlrig.constraint.aim",
        "orlrig.constraint.aim_locator",
        "orlrig.constraint.copy_xform",
        "orlrig.constraint.copy_translation",
        "orlrig.constraint.copy_rotation",
        "orlrig.constraint.copy_scale",
        "orlrig.constraint.parent",
        "orlrig.input.find_joint",
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
            REQUIRE(definition->outputs.size() == 1);
            REQUIRE(definition->output("handle") != nullptr);
            REQUIRE(definition->output("index") == nullptr);
            REQUIRE(definition->output("xform") == nullptr);
            if (name == "orlrig.input.find_joint") {
                REQUIRE(definition->output("handle")->type
                    == LogicalType::handle("orlrig::joint_handle"));
            } else if (name == "orlrig.input.find_locator") {
                REQUIRE(definition->output("handle")->type
                    == LogicalType::handle("orlrig::locator_handle"));
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

    REQUIRE(graph.registry.find("orlrig.input.find_controller")
        == nullptr);
    REQUIRE(graph.registry.find("orlrig.input.legacy_find_joint")
        == nullptr);
    REQUIRE(graph.registry.find("orlrig.input.legacy_find_locator")
        == nullptr);
    REQUIRE(graph.registry.find("orlrig.solver.hd_id") == nullptr);
    REQUIRE(graph.registry.find("orlrig.solver.spline_ik") == nullptr);
    REQUIRE(graph.registry.find("orlrig.solver.full_body_ik") == nullptr);
}

TEST_CASE("two-bone solver exposes typed handles and inferred effects",
    "[orlrig][graph][handles][effects]")
{
    const auto graph = make_lbs_graph();
    const auto* definition =
        graph.registry.find("orlrig.solver.ik_two_bone");
    REQUIRE(definition != nullptr);
    REQUIRE(definition->inputs.size() == 5);
    REQUIRE(definition->input("root")->type
        == LogicalType::handle("orlrig::joint_handle"));
    REQUIRE(definition->input("mid")->type
        == LogicalType::handle("orlrig::joint_handle"));
    REQUIRE(definition->input("end")->type
        == LogicalType::handle("orlrig::joint_handle"));
    REQUIRE(definition->input("target")->type
        == LogicalType::handle("orlrig::locator_handle"));
    REQUIRE(definition->input("pole")->type
        == LogicalType::handle("orlrig::locator_handle"));

    REQUIRE(definition->metadata.find("partial_read_joint_ports")
        == definition->metadata.end());
    REQUIRE(definition->metadata.find("partial_write_joint_ports")
        == definition->metadata.end());
    REQUIRE(definition->metadata.find("partial_read_locator_ports")
        == definition->metadata.end());
    REQUIRE(definition->partial_footprint.has_value());
    const auto& footprint = *definition->partial_footprint;
    REQUIRE_FALSE(footprint.global);
    REQUIRE(footprint.handle_effects.size() >= 5);

    const auto access_for = [&footprint](std::string_view parameter) {
        bool reads = false;
        bool writes = false;
        for (const auto& effect : footprint.handle_effects) {
            if (effect.parameter.value != parameter) {
                continue;
            }
            reads = reads
                || effect.access == orlgraph::AccessMode::Read
                || effect.access == orlgraph::AccessMode::ReadWrite;
            writes = writes
                || effect.access == orlgraph::AccessMode::Write
                || effect.access == orlgraph::AccessMode::ReadWrite;
        }
        return reads && writes
            ? orlgraph::AccessMode::ReadWrite
            : writes ? orlgraph::AccessMode::Write
            : orlgraph::AccessMode::Read;
    };
    REQUIRE(access_for("root") == orlgraph::AccessMode::ReadWrite);
    REQUIRE(access_for("mid") == orlgraph::AccessMode::ReadWrite);
    REQUIRE(access_for("end") == orlgraph::AccessMode::Read);
    REQUIRE(access_for("target") == orlgraph::AccessMode::Read);
    REQUIRE(access_for("pole") == orlgraph::AccessMode::Read);
}

TEST_CASE("exact scene handles are not interchangeable",
    "[orlrig][graph][semantics][handle]")
{
    auto rig = make_lbs_graph();
    NodeDefinition sink;
    sink.id = StableId{"test.locator_sink"};
    sink.qualified_name = "test.locator_sink";
    sink.implementation.kind = ImplementationKind::Runtime;
    sink.implementation.runtime_name = "test.locator_sink";
    Port input;
    input.id = StableId{"value"};
    input.name = "value";
    input.direction = PortDirection::Input;
    input.type = LogicalType::handle("orlrig::locator_handle");
    sink.inputs.push_back(std::move(input));
    REQUIRE(rig.registry.register_definition(std::move(sink)));

    const auto validate_connection = [&rig](
        StableId definition, StableId output) {
            GraphModule module;
            module.add_node(NodeInstance{
                StableId{"find"}, std::move(definition),
                "find",
                {{"name", ConstantValue{LogicalType::string(),
                    std::string{"root"}}}},
                {}, InlinePolicy::Default});
            module.add_node(NodeInstance{
                StableId{"sink"}, StableId{"test.locator_sink"}, "sink",
                {}, {}, InlinePolicy::Default});
            REQUIRE(module.add_connection(Connection{
                Endpoint::node_port(StableId{"find"}, std::move(output)),
                Endpoint::node_port(StableId{"sink"}, StableId{"value"})}));
            return validate(module, rig.registry);
        };

    const auto joint_validation = validate_connection(
        StableId{"orlrig.input.find_joint"}, StableId{"handle"});
    REQUIRE_FALSE(joint_validation.ok());
    REQUIRE(std::any_of(joint_validation.diagnostics.begin(),
        joint_validation.diagnostics.end(),
        [](const auto& diagnostic) {
            return diagnostic.code == "ORLGRAPH_TYPE_MISMATCH";
        }));
}

#endif
