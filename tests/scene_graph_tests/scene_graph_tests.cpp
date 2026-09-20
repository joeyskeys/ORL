#include <algorithm>
#include <cstdint>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include "component_manager.hpp"
#include "control_map.hpp"
#include "graph_scene_runtime.hpp"
#include "ops/create_joint_op.hpp"
#include "ops/delete_op.hpp"
#include "ops/mirror_op.hpp"
#include "ops/transform_mode.hpp"
#include "ops/toggle_controller_attachment_op.hpp"
#include "orlrig/abi.hpp"
#include "orlrig/graph_resources.hpp"
#include "runtime_config.hpp"
#include "scene_graph_context.hpp"

namespace
{

static_assert(requires(ORL::CreateJointOp& operation) {
    operation.enter();
});

class TestWindowBackend final : public vkkk::WindowBackend {
public:
    TestWindowBackend() {
        pointer_state.window_size = size;
        pointer_state.framebuffer_size = size;
    }

    std::vector<const char*> instance_extensions(bool) const override {
        return {};
    }
    VkSurfaceKHR create_surface(VkInstance) override {
        return VK_NULL_HANDLE;
    }
    VkExtent2D framebuffer_size() const override {
        return size;
    }
    VkExtent2D window_size() const override {
        return size;
    }
    void wait_until_visible() override {}
    bool should_close() const override {
        return false;
    }
    void poll_events() override {}
    void set_resize_flag(bool*) override {}
    void* native_handle() const override {
        return nullptr;
    }
    vkkk::InputPointer pointer() const override {
        return pointer_state;
    }
    bool mouse_down(vkkk::MouseButton) const override {
        return false;
    }
    bool key_down(vkkk::Key) const override {
        return false;
    }
    std::uint32_t modifiers() const override {
        return 0;
    }

    VkExtent2D size{100, 100};
    vkkk::InputPointer pointer_state{50.0, 50.0};
};

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
    connect("target", "index", "target_index");
    connect("pole", "index", "pole_index");
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
    REQUIRE(context.graph().input(orlgraph::StableId{"joints"}) == nullptr);
    ORL::exec::GraphInputBinding binding;
    REQUIRE(context.scene_inputs().resolve_binding(
        orlrig::kSceneJointsBinding, binding, &error));
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

TEST_CASE("transform axis keys cycle screen world and local spaces",
    "[controls][transform]")
{
    ORL::TransformConstraint constraint;
    ORL::InputEvent event;
    event.kind = ORL::InputEvent::Kind::Key;
    event.action = vkkk::InputAction::Press;

    event.key = vkkk::Key::X;
    REQUIRE(constraint.handle_axis_key(event));
    REQUIRE(constraint.space == ORL::TransformSpace::World);
    REQUIRE(constraint.axis == ORL::TransformAxis::X);

    REQUIRE(constraint.handle_axis_key(event));
    REQUIRE(constraint.space == ORL::TransformSpace::Local);
    REQUIRE(constraint.axis == ORL::TransformAxis::X);

    event.key = vkkk::Key::Y;
    REQUIRE(constraint.handle_axis_key(event));
    REQUIRE(constraint.space == ORL::TransformSpace::Local);
    REQUIRE(constraint.axis == ORL::TransformAxis::Y);

    REQUIRE(constraint.handle_axis_key(event));
    REQUIRE(constraint.space == ORL::TransformSpace::Screen);
    REQUIRE(constraint.axis == ORL::TransformAxis::None);
}

TEST_CASE("control map separates window and panel bindings",
    "[controls][scope]")
{
    ORL::ControlMap controls;
    int window_calls = 0;
    int graph_calls = 0;
    controls.bind_op("window_action",
        [&window_calls](const ORL::InputEvent&) { ++window_calls; });
    controls.bind_op("graph_action",
        [&graph_calls](const ORL::InputEvent&) { ++graph_calls; });

    const ORL::InputSpec input{
        ORL::InputSpec::Type::Key,
        static_cast<int>(vkkk::Key::A),
        static_cast<int>(vkkk::InputAction::Press),
        0,
    };
    controls.map(input, "window_action",
        ORL::BindingScope::Window);
    controls.map(input, "graph_action",
        ORL::BindingScope::Panel, "node_graph");

    ORL::InputEvent event;
    event.kind = ORL::InputEvent::Kind::Key;
    event.action = vkkk::InputAction::Press;
    event.key = vkkk::Key::A;

    controls.set_active_panel("properties");
    controls.dispatch_event(event);
    REQUIRE(window_calls == 1);
    REQUIRE(graph_calls == 0);

    controls.set_active_panel("node_graph");
    controls.dispatch_event(event);
    REQUIRE(window_calls == 2);
    REQUIRE(graph_calls == 1);

    controls.set_active_panel("viewport");
    controls.dispatch_event(event);
    REQUIRE(window_calls == 3);
    REQUIRE(graph_calls == 1);
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

    const auto delete_key = find_key(vkkk::Key::Delete, 0);
    REQUIRE(delete_key != controls.bindings().end());
    REQUIRE(std::find(delete_key->ops.begin(), delete_key->ops.end(),
        "delete_selection") != delete_key->ops.end());
    REQUIRE(delete_key->scope == ORL::BindingScope::Panel);
    REQUIRE(delete_key->panel == "viewport");
    const auto backspace = find_key(vkkk::Key::Backspace, 0);
    REQUIRE(backspace != controls.bindings().end());
    REQUIRE(std::find(backspace->ops.begin(), backspace->ops.end(),
        "delete_selection") != backspace->ops.end());
    const auto m = find_key(vkkk::Key::M, 0);
    REQUIRE(m != controls.bindings().end());
    REQUIRE(std::find(m->ops.begin(), m->ops.end(), "mirror")
        != m->ops.end());

    const auto j = find_key(vkkk::Key::J, 0);
    REQUIRE(j != controls.bindings().end());
    REQUIRE(std::find(j->ops.begin(), j->ops.end(),
        "create_joint") != j->ops.end());
    const auto e = find_key(vkkk::Key::E, 0);
    REQUIRE(e != controls.bindings().end());
    REQUIRE(std::find(e->ops.begin(), e->ops.end(),
        "extend_joint_chain") != e->ops.end());
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

TEST_CASE("joint chain extension requires exactly one existing joint",
    "[controls][joint-creation]")
{
    vkkk::Scene scene;
    ORL::ComponentManager components;
    const auto joint = components.create_joint("root");
    const auto controller = components.create_controller("controller");
    vkkk::Camera camera{};
    ORL::Selection selection(components, scene);
    ORL::CreateJointOp create_joint(
        components, camera, glm::vec3{}, nullptr, selection);
    ORL::ExtendJointChainOp extend_joint_chain(create_joint);

    selection.set(ORL::SelectionRef::joint(joint));
    extend_joint_chain.enter();
    REQUIRE(extend_joint_chain.active());
    extend_joint_chain.cancel();
    REQUIRE_FALSE(extend_joint_chain.active());

    selection.clear();
    extend_joint_chain.enter();
    REQUIRE_FALSE(extend_joint_chain.active());

    selection.set(ORL::SelectionRef::controller(controller));
    extend_joint_chain.enter();
    REQUIRE_FALSE(extend_joint_chain.active());

    selection.set(ORL::SelectionRef::joint(joint));
    selection.add(ORL::SelectionRef::controller(controller));
    extend_joint_chain.enter();
    REQUIRE_FALSE(extend_joint_chain.active());
}

TEST_CASE("joint creation exposes a cursor preview for J and E modes",
    "[controls][joint-creation]")
{
    vkkk::Scene scene;
    ORL::ComponentManager components;
    const auto root = components.create_joint("root");
    ORL::Selection selection(components, scene);
    vkkk::Camera camera{};
    camera.pos = {0.0f, 0.0f, 5.0f};
    camera.front = {0.0f, 0.0f, -1.0f};
    camera.up = {0.0f, 1.0f, 0.0f};
    camera.ubo_data.view = glm::lookAt(
        camera.pos, camera.pos + camera.front, camera.up);
    camera.ubo_data.proj = glm::perspective(
        glm::radians(60.0f), 1.0f, 0.1f, 100.0f);
    camera.ubo_data.proj[1][1] *= -1.0f;
    TestWindowBackend window;

    ORL::CreateJointOp create_joint(
        components, camera, glm::vec3{}, &window, selection);
    create_joint.enter();
    REQUIRE(create_joint.active());
    REQUIRE(create_joint.preview().visible);
    REQUIRE_FALSE(create_joint.preview().parent);

    ORL::InputEvent move;
    move.kind = ORL::InputEvent::Kind::MouseMove;
    move.x = 75.0;
    move.y = 50.0;
    create_joint.eval(move);
    REQUIRE(create_joint.preview().visible);

    create_joint.cancel();
    REQUIRE_FALSE(create_joint.preview().visible);

    ORL::ExtendJointChainOp extend_joint_chain(create_joint);
    selection.set(ORL::SelectionRef::joint(root));
    extend_joint_chain.enter();
    REQUIRE(extend_joint_chain.active());
    REQUIRE(extend_joint_chain.preview().visible);
    REQUIRE(extend_joint_chain.preview().parent == root);
    extend_joint_chain.cancel();
    REQUIRE_FALSE(extend_joint_chain.preview().visible);
}

TEST_CASE("recursive joint deletion preserves packed hierarchy and detaches controllers",
    "[components][delete]")
{
    ORL::ComponentManager components;

    const auto removed_root = components.create_joint("removed_root");
    auto removed_child_data = orlviewer::make_identity_joint();
    removed_child_data.parent = 0;
    const auto removed_child = components.create_joint(
        "removed_child", removed_child_data);
    auto removed_grandchild_data = orlviewer::make_identity_joint();
    removed_grandchild_data.parent = 1;
    const auto removed_grandchild = components.create_joint(
        "removed_grandchild", removed_grandchild_data);

    const auto surviving_root = components.create_joint("surviving_root");
    auto surviving_child_data = orlviewer::make_identity_joint();
    surviving_child_data.parent = 3;
    const auto surviving_child = components.create_joint(
        "surviving_child", surviving_child_data);

    const auto child_controller = components.create_controller(
        "child_controller");
    REQUIRE(components.attach_controller(child_controller, removed_child));
    REQUIRE(components.destroy_joint_recursive(removed_root));

    REQUIRE(components.find(removed_root) == nullptr);
    REQUIRE(components.find(removed_child) == nullptr);
    REQUIRE(components.find(removed_grandchild) == nullptr);
    REQUIRE(components.joint(surviving_root) != nullptr);
    REQUIRE(components.joint(surviving_child) != nullptr);
    REQUIRE(components.joint(surviving_root)->parent == -1);
    REQUIRE(components.joint(surviving_child)->parent
        == components.joint_index(surviving_root));
    REQUIRE(components.controller(child_controller) != nullptr);
    REQUIRE(components.controller_attachment(child_controller) == nullptr);
    REQUIRE(components.validate_controller_attachments());

    const auto locator = components.create_locator("locator");
    const auto locator_controller = components.create_controller(
        "locator_controller");
    REQUIRE(components.attach_controller(locator_controller, locator));
    REQUIRE(components.destroy(locator));
    REQUIRE(components.controller(locator_controller) != nullptr);
    REQUIRE(components.controller_attachment(locator_controller) == nullptr);
    REQUIRE_FALSE(components.attached_controller(locator));
    REQUIRE(components.validate_controller_attachments());
}

TEST_CASE("delete operation removes selected scene and component elements",
    "[controls][delete]")
{
    vkkk::Scene scene;
    REQUIRE(scene.add_object("mesh_instance", "mesh"));
    ORL::ComponentManager components;

    const auto root = components.create_joint("root");
    auto child_data = orlviewer::make_identity_joint();
    child_data.parent = 0;
    const auto child = components.create_joint("child", child_data);
    const auto locator = components.create_locator("locator");
    const auto controller = components.create_controller("controller");
    REQUIRE(components.attach_controller(controller, locator));

    ORL::Selection selection(components, scene);
    ORL::DeleteOp delete_op(components, scene, selection);
    ORL::InputEvent event;
    event.kind = ORL::InputEvent::Kind::Key;
    event.key = vkkk::Key::Delete;
    event.action = vkkk::InputAction::Press;

    selection.add(ORL::SelectionRef::scene_object("mesh_instance"));
    selection.add(ORL::SelectionRef::controller(controller));
    delete_op.eval(event);

    REQUIRE(scene.find_object("mesh_instance") == nullptr);
    REQUIRE(components.find(controller) == nullptr);
    REQUIRE(components.find(locator) != nullptr);
    REQUIRE_FALSE(components.attached_controller(locator));
    REQUIRE(components.validate_controller_attachments());

    selection.set(ORL::SelectionRef::locator(locator));
    delete_op.eval(event);
    REQUIRE(components.find(locator) == nullptr);

    selection.set(ORL::SelectionRef::joint(root));
    delete_op.eval(event);
    REQUIRE(components.find(root) == nullptr);
    REQUIRE(components.find(child) == nullptr);
    REQUIRE(selection.empty());
}

TEST_CASE("mirror operation copies the selected joint subtree across X",
    "[controls][mirror]")
{
    vkkk::Scene scene;
    ORL::ComponentManager components;

    auto root_data = orlviewer::make_identity_joint();
    root_data.translation[0] = 2.0;
    const auto root = components.create_joint("root", root_data);

    auto child_data = orlviewer::make_identity_joint();
    child_data.parent = 0;
    child_data.translation[0] = 1.0;
    const auto child = components.create_joint("child", child_data);

    ORL::Selection selection(components, scene);
    selection.set(ORL::SelectionRef::joint(root));
    ORL::MirrorOp mirror(components, selection);
    mirror.enter();
    REQUIRE(mirror.active());

    ORL::InputEvent event;
    event.kind = ORL::InputEvent::Kind::Key;
    event.action = vkkk::InputAction::Press;
    event.key = vkkk::Key::X;
    mirror.eval(event);

    REQUIRE_FALSE(mirror.active());
    const auto* mirrored_root = components.find("root_mirror");
    const auto* mirrored_child = components.find("child_mirror");
    REQUIRE(mirrored_root != nullptr);
    REQUIRE(mirrored_child != nullptr);
    REQUIRE(components.joint(mirrored_root->id)->parent == -1);
    REQUIRE(components.joint(mirrored_child->id)->parent
        == components.joint_index(mirrored_root->id));

    const auto packed = components.packed_joints();
    const auto root_world = orlviewer::joint_world_matrix(
        packed, components.joint_index(root));
    const auto child_world = orlviewer::joint_world_matrix(
        packed, components.joint_index(child));
    const auto mirrored_root_world = orlviewer::joint_world_matrix(
        packed, components.joint_index(mirrored_root->id));
    const auto mirrored_child_world = orlviewer::joint_world_matrix(
        packed, components.joint_index(mirrored_child->id));
    REQUIRE(glm::vec3{mirrored_root_world[3]}.x
        == Catch::Approx(-glm::vec3{root_world[3]}.x));
    REQUIRE(glm::vec3{mirrored_child_world[3]}.x
        == Catch::Approx(-glm::vec3{child_world[3]}.x));

    REQUIRE(selection.refs().size() == 2);
    REQUIRE(selection.focus() != nullptr);
    REQUIRE(selection.focus()->component == mirrored_root->id);
    for (const auto& ref : selection.refs()) {
        REQUIRE(ref.kind == ORL::SelectionRef::Kind::Joint);
        REQUIRE((ref.component == mirrored_root->id
            || ref.component == mirrored_child->id));
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
    const auto mid_id = components.create_joint("mid", mid);
    const auto end_id = components.create_joint("end", end);
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

    REQUIRE(context.hierarchy_plan().has_value());
    REQUIRE(context.hierarchy_plan()->joint_count == 3);
    REQUIRE(context.hierarchy_plan()->is_direct_parent(root_id, mid_id));
    REQUIRE(context.hierarchy_plan()->is_direct_parent(mid_id, end_id));
    REQUIRE(context.evaluation_plan() != nullptr);
    REQUIRE(context.evaluation_plan()->regions.size() == 1);
    REQUIRE_FALSE(context.evaluation_plan()->regions.front().global);
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
    auto* packed = static_cast<orlrig::Joint*>(
        context.scene_inputs().computed_joints_buffer().data());
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

TEST_CASE("CUDA scene inputs share one packed arena",
    "[scene-graph][inputs][cuda][packed]")
{
    vkkk::Scene scene;
    ORL::ComponentManager components;
    components.create_joint("root");
    components.create_joint("end");
    components.create_locator("target");
    components.create_locator("pole");
    components.create_controller("z_ctrl");
    components.create_controller("a_ctrl");

    ORL::SceneGraphContext context(scene, components);
    context.refresh_scene_inputs();
    REQUIRE(context.scene_inputs().set_cuda_evaluation(true));
    REQUIRE(context.scene_inputs().ensure_cuda_inputs());

    std::string error;
    ORL::exec::GraphInputBinding joints;
    ORL::exec::GraphInputBinding locators;
    ORL::exec::GraphInputBinding target;
    ORL::exec::GraphInputBinding controllers;
    REQUIRE(context.scene_inputs().resolve_binding(
        orlrig::kSceneJointsBinding, joints, &error));
    REQUIRE(context.scene_inputs().resolve_binding(
        orlrig::kSceneLocatorsBinding, locators, &error));
    REQUIRE(context.scene_inputs().resolve_binding(
        orlrig::scene_locator_xform_binding("target"),
        target, &error));
    REQUIRE(context.scene_inputs().resolve_binding(
        orlrig::kSceneControllersBinding, controllers, &error));

    REQUIRE(joints.packed.has_value());
    REQUIRE(locators.packed.has_value());
    REQUIRE(target.packed.has_value());
    REQUIRE(controllers.packed.has_value());
    REQUIRE(joints.packed->data == locators.packed->data);
    REQUIRE(joints.packed->data == target.packed->data);
    REQUIRE(joints.packed->data == controllers.packed->data);
    REQUIRE(joints.packed->storage_bytes
        == sizeof(orlrig::SolverContext)
            + 2 * orlrig::kJointStride
            + 2 * orlrig::kLocatorStride
            + 2 * orlrig::kMatrixStride);
    REQUIRE(joints.packed->bytes == 2 * orlrig::kJointStride);
    REQUIRE(locators.packed->bytes == 2 * orlrig::kLocatorStride);
    REQUIRE(controllers.packed->bytes == 2 * orlrig::kMatrixStride);
    REQUIRE(target.packed->bytes == orlrig::kLocatorStride);
    REQUIRE(target.packed->offset == locators.packed->offset);

    REQUIRE(context.scene_inputs().set_cuda_evaluation(false));
    REQUIRE(context.scene_inputs().resolve_binding(
        orlrig::kSceneJointsBinding, joints, &error));
    REQUIRE_FALSE(joints.packed.has_value());
    REQUIRE(joints.buffer != nullptr);
}

TEST_CASE("scene graph context packs the compiled hierarchy layout",
    "[scene-graph][hierarchy][abi]")
{
    vkkk::Scene scene;
    ORL::ComponentManager components;
    const auto root = components.create_joint("root");
    auto child_joint = orlviewer::make_identity_joint();
    child_joint.parent = 0;
    const auto child = components.create_joint("child", child_joint);
    REQUIRE(root);
    REQUIRE(child);

    ORL::SceneGraphContext context(scene, components);
    std::string error;
    REQUIRE(context.ensure_hierarchy_plan(&error));
    REQUIRE(context.hierarchy_context().joint_count == 2);
    REQUIRE(context.hierarchy_data().count() > 0);
    REQUIRE(context.hierarchy_context().data_count
        == static_cast<std::int64_t>(context.hierarchy_data().count()));

    const auto first_signature = context.hierarchy_plan()
        ->topology_revision;
    REQUIRE(context.ensure_hierarchy_plan(&error));
    REQUIRE(context.hierarchy_plan()->topology_revision == first_signature);

    auto replacement = orlviewer::make_identity_joint();
    replacement.parent = -1;
    REQUIRE(components.joint(child) != nullptr);
    *components.joint(child) = replacement;
    REQUIRE(context.ensure_hierarchy_plan(&error));
    REQUIRE(context.hierarchy_plan()->topology_revision != first_signature);
    REQUIRE(context.hierarchy_context().joint_count == 2);
}

TEST_CASE("scene graph change sets separate pose edits from topology edits",
    "[scene-graph][partial][changes]")
{
    vkkk::Scene scene;
    ORL::ComponentManager components;
    const auto root = components.create_joint("root");
    const auto locator = components.create_locator("target");
    ORL::SceneGraphContext context(scene, components);
    std::string error;
    REQUIRE(context.ensure_hierarchy_plan(&error));
    (void)context.take_change_set();

    const auto before = context.take_change_set().generation;
    REQUIRE(components.locator(locator) != nullptr);
    components.locator(locator)->xform[3].x = 2.0f;
    context.refresh_scene_inputs();
    const auto pose_change = context.take_change_set();
    REQUIRE(std::find(pose_change.locators.begin(),
        pose_change.locators.end(), locator) != pose_change.locators.end());
    REQUIRE_FALSE(pose_change.plan_invalidated);
    REQUIRE(pose_change.generation > before);

    auto child = orlviewer::make_identity_joint();
    child.parent = 0;
    REQUIRE(components.create_joint("child", child));
    context.refresh_scene_inputs();
    const auto topology_change = context.take_change_set();
    REQUIRE(topology_change.plan_invalidated);
    REQUIRE(topology_change.full_evaluation);
    REQUIRE(topology_change.topology_revision
        > pose_change.topology_revision);
    REQUIRE(root);
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
