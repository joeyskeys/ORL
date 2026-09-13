#include <algorithm>
#include <string>
#include <utility>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "component_manager.hpp"
#include "control_map.hpp"
#include "graph_scene_runtime.hpp"
#include "orlrig/graph_resources.hpp"
#include "runtime_config.hpp"
#include "scene_graph_context.hpp"

namespace
{

orlgraph::GraphModule make_controller_to_joint_graph()
{
    orlgraph::GraphModule graph;
    graph.module_id = "scene.controller_to_joint";
    graph.add_input(orlgraph::InterfacePort{
        orlgraph::StableId{"controller_count"}, "controller_count",
        orlgraph::PortDirection::Input, orlgraph::LogicalType::int64(),
        orlgraph::Domain::constant(), orlgraph::Shape::scalar(), true,
        std::nullopt, false,
        orlrig::scene_controller_count_binding("ctrl"),
        {}, {}});
    graph.add_input(orlgraph::InterfacePort{
        orlgraph::StableId{"joint_count"}, "joint_count",
        orlgraph::PortDirection::Input, orlgraph::LogicalType::int64(),
        orlgraph::Domain::constant(), orlgraph::Shape::scalar(), true,
        std::nullopt, false, std::string{orlrig::kSceneJointCountBinding},
        {}, {}});
    graph.add_output(orlgraph::InterfacePort{
        orlgraph::StableId{"status"}, "status",
        orlgraph::PortDirection::Output, orlgraph::LogicalType::int64(),
        orlgraph::Domain::constant(), orlgraph::Shape::scalar(), true,
        std::nullopt, false, "status", {}, {}});
    graph.add_node(orlgraph::NodeInstance{
        orlgraph::StableId{"find_controller"},
        orlgraph::StableId{"orlrig.input.find_controller"},
        "find_controller",
        {{"name", orlgraph::ConstantValue{
            orlgraph::LogicalType::string(), std::string{"ctrl"}}}},
        {}, orlgraph::InlinePolicy::Default});
    graph.add_node(orlgraph::NodeInstance{
        orlgraph::StableId{"find_joint"},
        orlgraph::StableId{"orlrig.input.find_joint"},
        "find_joint",
        {{"name", orlgraph::ConstantValue{
            orlgraph::LogicalType::string(), std::string{"root"}}}},
        {}, orlgraph::InlinePolicy::Default});
    graph.add_node(orlgraph::NodeInstance{
        orlgraph::StableId{"copy"},
        orlgraph::StableId{"orlrig.constraint.copy_xform"},
        "copy", {}, {}, orlgraph::InlinePolicy::Default});
    graph.add_connection(orlgraph::Connection{
        orlgraph::Endpoint::node_port(
            orlgraph::StableId{"find_controller"},
            orlgraph::StableId{"xform"}),
        orlgraph::Endpoint::node_port(
            orlgraph::StableId{"copy"},
            orlgraph::StableId{"source"})});
    graph.add_connection(orlgraph::Connection{
        orlgraph::Endpoint::node_port(
            orlgraph::StableId{"find_joint"},
            orlgraph::StableId{"xform"}),
        orlgraph::Endpoint::node_port(
            orlgraph::StableId{"copy"},
            orlgraph::StableId{"destination"})});
    graph.add_connection(orlgraph::Connection{
        orlgraph::Endpoint::graph_input(
            orlgraph::StableId{"controller_count"}),
        orlgraph::Endpoint::node_port(
            orlgraph::StableId{"copy"},
            orlgraph::StableId{"source_count"})});
    graph.add_connection(orlgraph::Connection{
        orlgraph::Endpoint::graph_input(
            orlgraph::StableId{"joint_count"}),
        orlgraph::Endpoint::node_port(
            orlgraph::StableId{"copy"},
            orlgraph::StableId{"destination_count"})});
    graph.add_connection(orlgraph::Connection{
        orlgraph::Endpoint::node_port(
            orlgraph::StableId{"copy"},
            orlgraph::StableId{"status"}),
        orlgraph::Endpoint::graph_output(
            orlgraph::StableId{"status"})});
    return graph;
}

orlgraph::GraphModule make_two_bone_solver_graph()
{
    orlgraph::GraphModule graph;
    graph.module_id = "scene.two_bone_solver";
    graph.add_input(orlgraph::InterfacePort{
        orlgraph::StableId{"joint_count"}, "joint_count",
        orlgraph::PortDirection::Input, orlgraph::LogicalType::int64(),
        orlgraph::Domain::constant(), orlgraph::Shape::scalar(), true,
        std::nullopt, false, std::string{orlrig::kSceneJointCountBinding},
        {}, {}});
    graph.add_output(orlgraph::InterfacePort{
        orlgraph::StableId{"status"}, "status",
        orlgraph::PortDirection::Output, orlgraph::LogicalType::int64(),
        orlgraph::Domain::constant(), orlgraph::Shape::scalar(), true,
        std::nullopt, false, "status", {}, {}});

    const auto add_find = [&graph](
        std::string id, std::string definition, std::string name) {
        graph.add_node(orlgraph::NodeInstance{
            orlgraph::StableId{id},
            orlgraph::StableId{std::move(definition)},
            id,
            {{"name", orlgraph::ConstantValue{
                orlgraph::LogicalType::string(), std::move(name)}}},
            {}, orlgraph::InlinePolicy::Default});
    };
    add_find("target", "orlrig.input.find_controller", "target");
    add_find("pole", "orlrig.input.find_controller", "pole");
    add_find("root", "orlrig.input.find_joint", "root");
    add_find("mid", "orlrig.input.find_joint", "mid");
    add_find("end", "orlrig.input.find_joint", "end");
    graph.add_node(orlgraph::NodeInstance{
        orlgraph::StableId{"input_joints"},
        orlgraph::StableId{"orlrig.input.joints"},
        "input_joints", {}, {}, orlgraph::InlinePolicy::Default});
    graph.add_node(orlgraph::NodeInstance{
        orlgraph::StableId{"solver"},
        orlgraph::StableId{"orlrig.solver.ik_two_bone"},
        "solver", {}, {}, orlgraph::InlinePolicy::Default});

    const auto connect = [&graph](
        std::string source_node, std::string source_port,
        std::string destination_port) {
        graph.add_connection(orlgraph::Connection{
            orlgraph::Endpoint::node_port(
                orlgraph::StableId{std::move(source_node)},
                orlgraph::StableId{std::move(source_port)}),
            orlgraph::Endpoint::node_port(
                orlgraph::StableId{"solver"},
                orlgraph::StableId{std::move(destination_port)})});
    };
    connect("input_joints", "joints", "joints");
    connect("target", "xform", "target");
    connect("pole", "xform", "pole");
    connect("root", "index", "root");
    connect("mid", "index", "mid");
    connect("end", "index", "end");
    graph.add_connection(orlgraph::Connection{
        orlgraph::Endpoint::graph_input(
            orlgraph::StableId{"joint_count"}),
        orlgraph::Endpoint::node_port(
            orlgraph::StableId{"solver"},
            orlgraph::StableId{"joint_count"})});
    graph.add_connection(orlgraph::Connection{
        orlgraph::Endpoint::node_port(
            orlgraph::StableId{"solver"},
            orlgraph::StableId{"status"}),
        orlgraph::Endpoint::graph_output(
            orlgraph::StableId{"status"})});
    return graph;
}

} // namespace

TEST_CASE("scene graph context owns the active LBS graph",
    "[scene-graph][context]")
{
    vkkk::Scene scene;
    ORL::ComponentManager components;
    const auto joint_id = components.create_joint("root");
    const auto controller_id = components.create_controller("ctrl");
    const auto weight_id = components.create_weight("weights");
    const auto deformer_id = components.create_deformer("deformer");
    REQUIRE(scene.add_object("body", "body_mesh"));

    ORL::SceneGraphContext context(scene, components);
    auto graph = orlrig::make_lbs_graph();
    context.set_graph(
        std::move(graph.module), std::move(graph.registry));

    REQUIRE(context.validate().ok());
    REQUIRE(context.has_runtime_node("orlrig.deformer.lbs.capture_bind"));
    REQUIRE(context.has_runtime_node("orlrig.deformer.lbs.evaluate"));
    REQUIRE(context.registry().find("orlrig.input.joints") != nullptr);
    REQUIRE(context.registry().find("orlrig.input.controllers") != nullptr);
    REQUIRE(context.registry().find("orlrig.input.find_mesh") != nullptr);
    const auto* find_controller =
        context.registry().find("orlrig.input.find_controller");
    const auto* find_joint =
        context.registry().find("orlrig.input.find_joint");
    REQUIRE(find_controller != nullptr);
    REQUIRE(find_joint != nullptr);
    REQUIRE(find_controller->output("handle") != nullptr);
    REQUIRE(find_controller->output("index") != nullptr);
    REQUIRE(find_controller->output("xform") != nullptr);
    REQUIRE(find_joint->output("handle") != nullptr);
    REQUIRE(find_joint->output("index") != nullptr);
    REQUIRE(find_joint->output("xform") != nullptr);
    REQUIRE(find_joint->output("handle")->semantic
        == std::string{orlrig::kSceneJointHandleSemantic});
    REQUIRE(find_joint->output("index")->semantic
        == std::string{orlrig::kSceneArrayIndexSemantic});
    REQUIRE(find_controller->output("xform")->type
        == orlgraph::LogicalType::buffer(orlgraph::LogicalType::matrix()));
    REQUIRE(find_joint->output("xform")->cardinality
        == orlgraph::PortCardinality::Buffer);

    const auto* copy_xform =
        context.registry().find("orlrig.constraint.copy_xform");
    const auto* auto_weight =
        context.registry().find("orlrig.auto_weight.closest_joint");
    REQUIRE(copy_xform != nullptr);
    REQUIRE(auto_weight != nullptr);
    REQUIRE(copy_xform->output("status") != nullptr);
    REQUIRE(copy_xform->output("result") == nullptr);
    REQUIRE(auto_weight->output("status") != nullptr);

    const auto validation = context.validate();
    const auto capture = std::find(
        validation.schedule.order.begin(), validation.schedule.order.end(),
        orlgraph::StableId{"capture_bind"});
    const auto deform = std::find(
        validation.schedule.order.begin(), validation.schedule.order.end(),
        orlgraph::StableId{"deform"});
    REQUIRE(capture != validation.schedule.order.end());
    REQUIRE(deform != validation.schedule.order.end());
    REQUIRE(capture < deform);

    context.refresh_scene_inputs();
    std::string error;
    REQUIRE(context.map_input_by_binding(
        "joints", std::string{orlrig::kSceneJointsBinding}, &error));
    const auto* joints = context.graph().input(orlgraph::StableId{"joints"});
    REQUIRE(joints != nullptr);
    ORL::exec::GraphInputBinding binding;
    REQUIRE(context.resolve_graph_input(*joints, binding, &error));
    REQUIRE(binding.kind == ORL::exec::ParameterKind::Buffer);
    REQUIRE(binding.element_count == 1);
    REQUIRE(context.scene_inputs().resolve_element_handle(
        ORL::SceneElementKind::Mesh, "body") == 0);
    REQUIRE(context.scene_inputs().resolve_element_handle(
        ORL::SceneElementKind::Controller, "ctrl").has_value());
    REQUIRE(context.scene_inputs().resolve_element_index(
        ORL::SceneElementKind::Joint, "root") == 0);
    REQUIRE(context.scene_inputs().resolve_element_index(
        ORL::SceneElementKind::Controller, "ctrl") == 0);

    ORL::exec::GraphInputBinding controllers;
    REQUIRE(context.scene_inputs().resolve_binding(
        orlrig::kSceneControllersBinding, controllers, &error));
    REQUIRE(controllers.kind == ORL::exec::ParameterKind::Buffer);
    REQUIRE(controllers.element_count == 1);

    REQUIRE_FALSE(context.map_input_by_binding(
        "missing", "scene.rig.missing", &error));

    ORL::Selection selection(components, scene);
    ORL::GraphSceneRuntime runtime(
        context, selection, deformer_id, weight_id);
    runtime.request_bind();
    REQUIRE(context.take_operation("bind"));
    REQUIRE_FALSE(context.take_operation("bind"));

    REQUIRE(components.find(joint_id) != nullptr);
    REQUIRE(components.find(controller_id) != nullptr);
}

TEST_CASE("control map selects the first matching operation overload",
    "[controls][polymorphism]")
{
    ORL::ControlMap controls;
    std::string selected;
    controls.bind_op_variant("contextual_bind",
        [](const ORL::InputEvent& event) {
            return event.key == vkkk::Key::A;
        },
        [&selected](const ORL::InputEvent&) {
            selected = "deformer";
        });
    controls.bind_op_variant("contextual_bind",
        [](const ORL::InputEvent& event) {
            return event.key == vkkk::Key::B;
        },
        [&selected](const ORL::InputEvent&) {
            selected = "constraint";
        });

    controls.map(ORL::InputSpec{
        ORL::InputSpec::Type::Key,
        static_cast<int>(vkkk::Key::A),
        static_cast<int>(vkkk::InputAction::Press),
        0,
    }, "contextual_bind");
    controls.map(ORL::InputSpec{
        ORL::InputSpec::Type::Key,
        static_cast<int>(vkkk::Key::B),
        static_cast<int>(vkkk::InputAction::Press),
        0,
    }, "contextual_bind");

    ORL::InputEvent event;
    event.kind = ORL::InputEvent::Kind::Key;
    event.action = vkkk::InputAction::Press;
    event.key = vkkk::Key::A;
    controls.dispatch_event(event);
    REQUIRE(selected == "deformer");

    event.key = vkkk::Key::B;
    controls.dispatch_event(event);
    REQUIRE(selected == "constraint");
}

TEST_CASE("graph runtime executes ORL constraints and commits joint output",
    "[scene-graph][runtime][constraint]")
{
    const auto previous_device = ORL::runtime_config.device;
    ORL::runtime_config.device = ORL::ComputeDevice::Cpu;

    vkkk::Scene scene;
    ORL::ComponentManager components;
    const auto joint_id = components.create_joint("root");
    const auto controller_id = components.create_controller(
        "ctrl", orlrig::make_controller(glm::vec3{4.0f, 0.0f, 0.0f}));

    ORL::SceneGraphContext context(scene, components);
    orlgraph::NodeRegistry registry;
    std::string registry_error;
    REQUIRE(orlrig::register_rig_node_definitions(
        registry, &registry_error));
    context.set_graph(
        make_controller_to_joint_graph(), std::move(registry));

    ORL::Selection selection(components, scene);
    ORL::GraphSceneRuntime runtime(context, selection, {}, {});
    vkkk::Context backend(false);
    runtime.request_bind();
    runtime.on_update(backend);

    REQUIRE(components.joint(joint_id)->translation[0]
        == Catch::Approx(4.0));
    REQUIRE(components.joint(joint_id)->translation[1]
        == Catch::Approx(0.0));
    REQUIRE(components.joint(joint_id)->translation[2]
        == Catch::Approx(0.0));
    REQUIRE(components.controller(controller_id) != nullptr);

    orlrig::set_controller_origin(
        *components.controller(controller_id), glm::vec3{6.0f, 0.0f, 0.0f});
    runtime.on_update(backend);
    REQUIRE(components.joint(joint_id)->translation[0]
        == Catch::Approx(6.0));

    ORL::runtime_config.device = previous_device;
}

TEST_CASE("graph runtime executes ORL solver mutations",
    "[scene-graph][runtime][solver]")
{
    const auto previous_device = ORL::runtime_config.device;
    ORL::runtime_config.device = ORL::ComputeDevice::Cpu;

    vkkk::Scene scene;
    ORL::ComponentManager components;
    auto root = orlviewer::make_identity_joint();
    auto mid = orlviewer::make_identity_joint();
    mid.parent = 0;
    mid.translation[0] = 1.0;
    auto end = orlviewer::make_identity_joint();
    end.parent = 1;
    end.translation[0] = 1.0;
    const auto root_id = components.create_joint("root", root);
    components.create_joint("mid", mid);
    components.create_joint("end", end);
    components.create_controller(
        "target", orlrig::make_controller(glm::vec3{0.0f, 1.5f, 0.0f}));
    components.create_controller(
        "pole", orlrig::make_controller(glm::vec3{0.0f, 0.0f, 1.0f}));

    ORL::SceneGraphContext context(scene, components);
    orlgraph::NodeRegistry registry;
    std::string registry_error;
    REQUIRE(orlrig::register_rig_node_definitions(
        registry, &registry_error));
    context.set_graph(make_two_bone_solver_graph(), std::move(registry));

    ORL::Selection selection(components, scene);
    ORL::GraphSceneRuntime runtime(context, selection, {}, {});
    vkkk::Context backend(false);
    runtime.request_bind();
    runtime.on_update(backend);

    REQUIRE(components.joint(root_id)->rotation[3]
        != Catch::Approx(1.0));
    ORL::runtime_config.device = previous_device;
}

TEST_CASE("scene graph commits host-readback joint writeback",
    "[scene-graph][writeback]")
{
    vkkk::Scene scene;
    ORL::ComponentManager components;
    const auto joint_id = components.create_joint("root");
    REQUIRE(scene.add_object("body", "body_mesh"));

    ORL::SceneGraphContext context(scene, components);
    auto graph = orlrig::make_lbs_graph();
    context.set_graph(
        std::move(graph.module), std::move(graph.registry));
    context.refresh_scene_inputs();

    std::string error;
    REQUIRE(context.map_input_by_binding(
        "joints", std::string{orlrig::kSceneJointsBinding}, &error));
    const auto* joints_input =
        context.graph().input(orlgraph::StableId{"joints"});
    REQUIRE(joints_input != nullptr);
    ORL::exec::GraphInputBinding binding;
    REQUIRE(context.resolve_graph_input(*joints_input, binding, &error));
    REQUIRE(binding.buffer != nullptr);
    auto* packed = static_cast<orlrig::Joint*>(binding.buffer->data());
    REQUIRE(packed != nullptr);
    packed[0].translation[0] = 3.0;

    REQUIRE_FALSE(context.commit_scene_writes(false, &error));
    REQUIRE(components.joint(joint_id)->translation[0]
        == 0.0);
    REQUIRE(context.commit_scene_writes(true, &error));
    REQUIRE(components.joint(joint_id)->translation[0]
        == 3.0);
}

TEST_CASE("stable scene handles survive packed index changes",
    "[scene-graph][handles]")
{
    vkkk::Scene scene;
    ORL::ComponentManager components;
    const auto first_joint = components.create_joint("first");
    const auto second_joint = components.create_joint("second");
    const auto first_controller = components.create_controller("z_ctrl");
    const auto second_controller = components.create_controller("a_ctrl");

    ORL::SceneGraphContext context(scene, components);
    const auto initial_revision = context.scene_input_revision();
    const auto second_handle =
        context.scene_inputs().resolve_element_handle(
            ORL::SceneElementKind::Joint, "second");
    REQUIRE(second_handle.has_value());
    REQUIRE(context.scene_inputs().resolve_element_index(
        ORL::SceneElementKind::Joint, "second") == 1);
    REQUIRE(context.scene_inputs().resolve_element_index(
        ORL::SceneElementKind::Controller, "z_ctrl") == 1);

    std::string error;
    ORL::exec::GraphInputBinding binding;
    REQUIRE(context.scene_inputs().resolve_binding(
        orlrig::kSceneJointsBinding, binding, &error));
    REQUIRE(binding.buffer != nullptr);
    auto* packed = static_cast<orlrig::Joint*>(binding.buffer->data());
    REQUIRE(packed != nullptr);
    packed[1].translation[0] = 9.0;

    REQUIRE(components.destroy(first_joint));
    REQUIRE(context.scene_inputs().resolve_element_handle(
        ORL::SceneElementKind::Joint, "second") == *second_handle);
    REQUIRE(context.scene_inputs().resolve_element_index(
        ORL::SceneElementKind::Joint, "second") == 0);

    // The buffer still carries the old stable-ID order, so writeback must
    // resolve its entries by ID rather than using the new packed positions.
    REQUIRE(context.commit_scene_writes(true, &error));
    REQUIRE(components.joint(second_joint)->translation[0] == 9.0);

    REQUIRE(components.rename(first_controller, "b_ctrl"));
    REQUIRE(components.rename(second_controller, "z_ctrl_2"));
    context.refresh_scene_inputs();
    REQUIRE(context.scene_input_revision() > initial_revision);
    REQUIRE(context.scene_inputs().resolve_element_index(
        ORL::SceneElementKind::Controller, "z_ctrl_2") == 1);
}
