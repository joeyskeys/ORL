#include <algorithm>
#include <cstdint>
#include <string>
#include <string_view>
#include <utility>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "component_manager.hpp"
#include "control_map.hpp"
#include "graph_scene_runtime.hpp"
#include "ops/create_joint_op.hpp"
#include "ops/toggle_controller_attachment_op.hpp"
#include "orlrig/graph_resources.hpp"
#include "runtime_config.hpp"
#include "scene_graph_context.hpp"

namespace
{

static_assert(requires(ORL::CreateJointOp& operation) {
    operation.enter();
});

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
    add_find("target", "orlrig.input.find_locator", "target");
    add_find("pole", "orlrig.input.find_locator", "pole");
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
    const auto locator_id = components.create_locator("loc");
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
    REQUIRE(context.registry().find("orlrig.input.locators") != nullptr);
    REQUIRE(context.registry().find("orlrig.input.find_mesh") != nullptr);
    const auto* find_controller =
        context.registry().find("orlrig.input.find_controller");
    const auto* find_joint =
        context.registry().find("orlrig.input.find_joint");
    const auto* find_locator =
        context.registry().find("orlrig.input.find_locator");
    REQUIRE(find_controller != nullptr);
    REQUIRE(find_joint != nullptr);
    REQUIRE(find_locator != nullptr);
    REQUIRE(find_controller->output("handle") != nullptr);
    REQUIRE(find_controller->output("index") != nullptr);
    REQUIRE(find_controller->output("xform") != nullptr);
    REQUIRE(find_joint->output("handle") != nullptr);
    REQUIRE(find_joint->output("index") != nullptr);
    REQUIRE(find_joint->output("xform") != nullptr);
    REQUIRE(find_locator->output("xform") != nullptr);
    REQUIRE(find_joint->output("handle")->semantic
        == std::string{orlrig::kSceneJointHandleSemantic});
    REQUIRE(find_joint->output("index")->semantic
        == std::string{orlrig::kSceneArrayIndexSemantic});
    REQUIRE(find_controller->output("xform")->type
        == orlgraph::LogicalType::buffer(orlgraph::LogicalType::matrix()));
    REQUIRE(find_locator->output("xform")->type
        == orlgraph::LogicalType::buffer(
            orlgraph::LogicalType::struct_type("Locator")));
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
    REQUIRE(context.scene_inputs().resolve_element_handle(
        ORL::SceneElementKind::Locator, "loc").has_value());
    REQUIRE(context.scene_inputs().resolve_element_index(
        ORL::SceneElementKind::Joint, "root") == 0);
    REQUIRE(context.scene_inputs().resolve_element_index(
        ORL::SceneElementKind::Controller, "ctrl") == 0);
    REQUIRE(context.scene_inputs().resolve_element_index(
        ORL::SceneElementKind::Locator, "loc") == 0);

    ORL::exec::GraphInputBinding controllers;
    REQUIRE(context.scene_inputs().resolve_binding(
        orlrig::kSceneControllersBinding, controllers, &error));
    REQUIRE(controllers.kind == ORL::exec::ParameterKind::Buffer);
    REQUIRE(controllers.element_count == 1);
    ORL::exec::GraphInputBinding locators;
    REQUIRE(context.scene_inputs().resolve_binding(
        orlrig::kSceneLocatorsBinding, locators, &error));
    REQUIRE(locators.kind == ORL::exec::ParameterKind::Buffer);
    REQUIRE(locators.element_count == 1);
    ORL::exec::GraphInputBinding locator_xform;
    REQUIRE(context.scene_inputs().resolve_binding(
        orlrig::scene_locator_xform_binding("loc"),
        locator_xform, &error));
    REQUIRE(locator_xform.kind == ORL::exec::ParameterKind::Buffer);
    REQUIRE(locator_xform.element_count == 1);
    REQUIRE(static_cast<const double*>(locator_xform.buffer->data())[3]
        == Catch::Approx(0.0));

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
    REQUIRE(components.find(locator_id) != nullptr);
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

TEST_CASE("control map scopes viewport edits",
    "[controls][evaluation]")
{
    ORL::ControlMap controls;
    int evaluations = 0;
    int scope_enters = 0;
    int scope_exits = 0;
    struct EditOperation {
        int* evaluations = nullptr;

        void eval(const ORL::InputEvent&) {
            ++*evaluations;
        }
    } operation{&evaluations};

    controls.set_operation_scope_handler(
        [&scope_enters, &scope_exits](std::string_view, bool entering) {
            if (entering) {
                ++scope_enters;
            } else {
                ++scope_exits;
            }
        });
    controls.bind_edit_op("edit", operation);
    controls.map(ORL::InputSpec{
        ORL::InputSpec::Type::Key,
        static_cast<int>(vkkk::Key::A),
        static_cast<int>(vkkk::InputAction::Press),
        0,
    }, "edit");

    ORL::InputEvent event;
    event.kind = ORL::InputEvent::Kind::Key;
    event.action = vkkk::InputAction::Press;
    event.key = vkkk::Key::A;
    controls.dispatch_event(event);

    REQUIRE(evaluations == 1);
    REQUIRE(scope_enters == 1);
    REQUIRE(scope_exits == 1);
}

TEST_CASE("controller attachments preserve target-local transforms",
    "[scene-graph][attachment]")
{
    vkkk::Scene scene;
    ORL::ComponentManager components;
    auto root = orlviewer::make_identity_joint();
    root.translation[0] = 2.0;
    const auto joint_id = components.create_joint("root", root);
    const auto locator_id = components.create_locator(
        "loc", orlrig::make_locator(glm::vec3{10.0f, 0.0f, 0.0f}));
    const auto controller_id = components.create_controller(
        "ctrl", orlrig::make_controller(glm::vec3{4.0f, 0.0f, 0.0f}));
    const auto second_controller = components.create_controller("ctrl2");

    std::string error;
    REQUIRE(components.attach_controller(controller_id, joint_id, &error));
    const auto* attachment =
        components.controller_attachment(controller_id);
    REQUIRE(attachment != nullptr);
    REQUIRE(attachment->target_kind == ORL::AttachmentTargetKind::Joint);
    REQUIRE(attachment->xform[3].x == Catch::Approx(2.0f));
    REQUIRE(components.controller(controller_id)->xform[3].x
        == Catch::Approx(4.0f));
    REQUIRE(components.controller(controller_id)->input_xform[3].x
        == Catch::Approx(0.0f));
    REQUIRE(components.controller_world_xform(controller_id)[3].x
        == Catch::Approx(4.0f));

    glm::mat4 desired{1.0f};
    desired[3] = glm::vec4{5.0f, 0.0f, 0.0f, 1.0f};
    REQUIRE(components.set_controller_world_xform(
        controller_id, desired, &error));
    REQUIRE(components.joint(joint_id)->translation[0]
        == Catch::Approx(2.0));
    REQUIRE(components.controller(controller_id)->xform[3].x
        == Catch::Approx(4.0f));
    REQUIRE(components.controller(controller_id)->input_xform[3].x
        == Catch::Approx(1.0f));
    REQUIRE(components.apply_controller_inputs(&error));
    REQUIRE(components.joint(joint_id)->translation[0]
        == Catch::Approx(3.0));
    REQUIRE(components.controller_world_xform(controller_id)[3].x
        == Catch::Approx(5.0f));

    REQUIRE_FALSE(components.attach_controller(
        second_controller, joint_id, &error));
    REQUIRE_FALSE(components.attach_controller(
        controller_id, second_controller, &error));
    REQUIRE(components.attach_controller(controller_id, locator_id, &error));
    REQUIRE(components.controller_attachment(
        controller_id)->target_kind == ORL::AttachmentTargetKind::Locator);
    REQUIRE(components.controller_world_xform(controller_id)[3].x
        == Catch::Approx(5.0f));
    REQUIRE(components.validate_controller_attachments(&error));

    REQUIRE(components.destroy(locator_id));
    REQUIRE(components.controller_attachment(controller_id) == nullptr);
    REQUIRE(components.controller(controller_id)->xform[3].x
        == Catch::Approx(5.0f));
    REQUIRE(components.attach_controller(controller_id, joint_id, &error));
    REQUIRE(components.detach_controller(controller_id));
    REQUIRE(components.validate_controller_attachments(&error));
}

TEST_CASE("attachment toggle accepts controller plus joint context",
    "[scene-graph][attachment][controls]")
{
    vkkk::Scene scene;
    ORL::ComponentManager components;
    const auto joint = components.create_joint("joint");
    const auto controller = components.create_controller("controller");
    ORL::Selection selection(components, scene);
    selection.add(ORL::SelectionRef::controller(controller));
    selection.add(ORL::SelectionRef::joint(joint));

    ORL::ToggleControllerAttachmentOp toggle(components, selection);
    ORL::InputEvent event;
    event.kind = ORL::InputEvent::Kind::Key;
    event.key = vkkk::Key::B;
    event.action = vkkk::InputAction::Press;
    toggle.eval(event);
    REQUIRE(components.controller_attachment(controller) != nullptr);
    toggle.eval(event);
    REQUIRE(components.controller_attachment(controller) == nullptr);
}

TEST_CASE("controller setup scale is not applied to attachment target",
    "[scene-graph][attachment][controller-input]")
{
    vkkk::Scene scene;
    ORL::ComponentManager components;
    const auto joint = components.create_joint("joint");
    orlrig::Controller controller = orlrig::make_controller(
        glm::vec3{4.0f, 0.0f, 0.0f});
    controller.xform[0][0] = 3.0f;
    controller.xform[1][1] = 3.0f;
    controller.xform[2][2] = 3.0f;
    const auto controller_id = components.create_controller(
        "controller", controller);

    std::string error;
    REQUIRE(components.attach_controller(controller_id, joint, &error));
    REQUIRE(components.joint(joint)->scale[0] == Catch::Approx(1.0));
    REQUIRE(components.joint(joint)->scale[1] == Catch::Approx(1.0));
    REQUIRE(components.joint(joint)->scale[2] == Catch::Approx(1.0));
    REQUIRE(components.controller(controller_id)->xform[0][0]
        == Catch::Approx(3.0f));
    REQUIRE(components.controller(controller_id)->input_xform[0][0]
        == Catch::Approx(1.0f));

    ORL::Selection selection(components, scene);
    selection.set(ORL::SelectionRef::controller(controller_id));
    selection.set_controller_input_mode(false);
    glm::mat4 setup_world =
        components.controller_world_xform(controller_id);
    setup_world[0][0] = 5.0f;
    setup_world[1][1] = 5.0f;
    setup_world[2][2] = 5.0f;
    auto setup_attr = selection.dest();
    REQUIRE(setup_attr.set_world_matrix(setup_world));
    REQUIRE(components.joint(joint)->scale[0] == Catch::Approx(1.0));
    REQUIRE(components.controller(controller_id)->input_xform[0][0]
        == Catch::Approx(1.0f));
}

TEST_CASE("C toggles controller attachments from joint or locator selection",
    "[scene-graph][attachment][controls]")
{
    vkkk::Scene scene;
    ORL::ComponentManager components;
    const auto joint = components.create_joint("joint");
    const auto locator = components.create_locator(
        "locator", orlrig::make_locator(glm::vec3{2.0f, 0.0f, 0.0f}));
    ORL::Selection selection(components, scene);
    ORL::ToggleControllerAttachmentOp toggle(components, selection);

    ORL::InputEvent event;
    event.kind = ORL::InputEvent::Kind::Key;
    event.key = vkkk::Key::C;
    event.action = vkkk::InputAction::Press;

    selection.add(ORL::SelectionRef::joint(joint));
    toggle.eval(event);
    const auto joint_controller = components.attached_controller(joint);
    REQUIRE(joint_controller);
    REQUIRE(components.controller_shape(joint_controller)
        == orlviewer::ControllerShape::Circle);
    REQUIRE(components.controller_attachment(joint_controller) != nullptr);
    toggle.eval(event);
    REQUIRE_FALSE(components.attached_controller(joint));
    REQUIRE(components.controller(joint_controller) != nullptr);

    selection.clear();
    selection.add(ORL::SelectionRef::locator(locator));
    toggle.eval(event);
    const auto locator_controller = components.attached_controller(locator);
    REQUIRE(locator_controller);
    REQUIRE(components.controller_attachment(locator_controller) != nullptr);
    toggle.eval(event);
    REQUIRE_FALSE(components.attached_controller(locator));
}

TEST_CASE("control map preserves contextual attachment and curve controls",
    "[controls][attachment]")
{
    ORL::ControlMap controls;
    controls.load_config("resource/config/control_map.json");
    const auto find_key = [&controls](
                              vkkk::Key key, std::uint32_t mods) {
        return std::find_if(controls.bindings().begin(),
            controls.bindings().end(), [key, mods](
                const ORL::ControlBinding& binding) {
            return binding.input.type == ORL::InputSpec::Type::Key
                && binding.input.code == static_cast<int>(key)
                && binding.input.action
                    == static_cast<int>(vkkk::InputAction::Press)
                && binding.input.mods == mods;
        });
    };

    const auto b = find_key(vkkk::Key::B, 0);
    REQUIRE(b != controls.bindings().end());
    REQUIRE(std::find(b->ops.begin(), b->ops.end(),
        "toggle_controller_attachment") != b->ops.end());

    const auto c = find_key(vkkk::Key::C, 0);
    REQUIRE(c != controls.bindings().end());
    REQUIRE(std::find(c->ops.begin(), c->ops.end(),
        "toggle_controller_attachment") != c->ops.end());
    REQUIRE(std::find(c->ops.begin(), c->ops.end(),
        "create_controller") == c->ops.end());

    const auto shift_c = find_key(vkkk::Key::C, vkkk::input_mod::shift);
    REQUIRE(shift_c != controls.bindings().end());
    REQUIRE(std::find(shift_c->ops.begin(), shift_c->ops.end(),
        "cycle_controller_curve") != shift_c->ops.end());

    const auto tab = find_key(vkkk::Key::Tab, 0);
    REQUIRE(tab != controls.bindings().end());
    REQUIRE(std::find(tab->ops.begin(), tab->ops.end(),
        "toggle_orl_evaluation") != tab->ops.end());

    const auto j = find_key(vkkk::Key::J, 0);
    REQUIRE(j != controls.bindings().end());
    REQUIRE(std::find(j->ops.begin(), j->ops.end(),
        "create_joint") != j->ops.end());
    for (const auto& binding : controls.bindings()) {
        if (binding.input.type == ORL::InputSpec::Type::MouseButton
            && binding.input.code
                == static_cast<int>(vkkk::MouseButton::Left))
        {
            REQUIRE(std::find(binding.ops.begin(), binding.ops.end(),
                "create_joint") == binding.ops.end());
        }
    }
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
    auto runtime = [&]() -> ORL::GraphSceneRuntime {
        ORL::GraphSceneRuntime temporary(context, selection, {}, {});
        return std::move(temporary);
    }();
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
    components.create_locator(
        "target", orlrig::make_locator(glm::vec3{0.0f, 1.5f, 0.0f}));
    components.create_locator(
        "pole", orlrig::make_locator(glm::vec3{0.0f, 0.0f, 1.0f}));

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

TEST_CASE("solver feature path evaluates the staged solver graph",
    "[scene-graph][runtime][stages][solver]")
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
    components.create_locator(
        "target", orlrig::make_locator(glm::vec3{0.0f, 1.5f, 0.0f}));
    components.create_locator(
        "pole", orlrig::make_locator(glm::vec3{0.0f, 0.0f, 1.0f}));

    ORL::SceneGraphContext context(scene, components);
    orlgraph::NodeRegistry registry;
    std::string registry_error;
    REQUIRE(orlrig::register_rig_node_definitions(
        registry, &registry_error));
    orlgraph::GraphModule deformer;
    deformer.module_id = "scene.empty_deformer";
    context.set_stage_graphs(
        make_two_bone_solver_graph(), std::move(deformer),
        std::move(registry));

    ORL::Selection selection(components, scene);
    ORL::GraphSceneRuntime runtime(
        context, selection, {}, {}, orlgraph::GraphStage::Solver);
    vkkk::Context backend(false);
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
    const auto* scene_joints_input =
        context.graph().input(orlgraph::StableId{"joints"});
    REQUIRE(scene_joints_input != nullptr);
    ORL::exec::GraphInputBinding binding;
    REQUIRE(context.resolve_graph_input(
        *scene_joints_input, binding, &error));
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

TEST_CASE("computed joints resolves the shared device buffer",
    "[scene-graph][stages][computed-joints]")
{
    vkkk::Scene scene;
    ORL::ComponentManager components;
    components.create_joint("root");

    ORL::SceneGraphContext context(scene, components);
    context.refresh_scene_inputs();
    const auto* descriptor = context.scene_inputs().find(
        orlgraph::StableId{std::string{orlrig::kComputedJointsBinding}});
    REQUIRE(descriptor != nullptr);

    ORL::exec::GraphInputBinding binding;
    std::string error;
    REQUIRE(context.scene_inputs().resolve(
        descriptor->port, binding, &error));
    REQUIRE(binding.buffer != nullptr);
    REQUIRE(binding.device_ptr == 0);

    context.set_computed_joints_device(
        ORL::exec::DeviceBufferView{
            0x1234, orlrig::kJointStride},
        1);
    REQUIRE(context.scene_inputs().resolve(
        descriptor->port, binding, &error));
    REQUIRE(binding.buffer == nullptr);
    REQUIRE(binding.device_ptr == 0x1234);
    REQUIRE(binding.bytes == orlrig::kJointStride);
    REQUIRE(binding.element_count == 1);

    context.clear_computed_joints_device();
    REQUIRE(context.scene_inputs().resolve(
        descriptor->port, binding, &error));
    REQUIRE(binding.buffer != nullptr);
    REQUIRE(binding.device_ptr == 0);
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
