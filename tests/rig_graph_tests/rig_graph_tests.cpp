#if __has_include(<catch2/catch_all.hpp>)

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
    REQUIRE(graph.module.input(graph.resources.joints)->binding == "joints");
    REQUIRE(graph.module.input(graph.resources.weights)->binding == "weights");
}

TEST_CASE("scene input bindings use stable semantic names",
    "[orlrig][graph][scene]")
{
    REQUIRE(kSceneJointsBinding == "scene.rig.joints");
    REQUIRE(kSceneJointCountBinding == "scene.rig.joint_count");
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
        "orlrig.constraint.copy_xform",
        "orlrig.constraint.copy_translation",
        "orlrig.constraint.copy_rotation",
        "orlrig.constraint.copy_scale",
        "orlrig.input.find_joint",
        "orlrig.input.find_controller",
    };

    for (const auto& name : definitions) {
        const auto* definition = graph.registry.find(name);
        REQUIRE(definition != nullptr);
        const bool is_find_input = name.rfind("orlrig.input.find_", 0) == 0;
        REQUIRE(definition->implementation.kind
            == (is_find_input
                ? ImplementationKind::Runtime
                : ImplementationKind::OrlFunction));
        REQUIRE(definition->outputs.size() == 1);
        if (is_find_input) {
            REQUIRE(definition->outputs.front().name == "handle");
            REQUIRE(definition->parameter("name") != nullptr);
        } else {
            REQUIRE(definition->outputs.front().name == "result");
        }
    }
}

#endif
