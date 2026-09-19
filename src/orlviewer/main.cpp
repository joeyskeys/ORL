#include <algorithm>
#include <array>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <string_view>
#include <utility>

#include <glm/geometric.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include "ORL/frame.h"

#include "asset_mgr/scene.h"
#include "camera_navigator.hpp"
#include "component_manager.hpp"
#include "concepts/camera.h"
#include "control_map.hpp"
#include "selection.hpp"
#include "scene_graph_context.hpp"
#include "orlrig/graph_resources.hpp"
#if ORL_USE_QT6
#include "gui/qt_backend.hpp"
#include <QApplication>
#include <QKeySequence>
#include <QString>
#include <QShortcut>
#include "qt/node_graph_editor.hpp"
#include "qt/property_editor.hpp"
#else
#include "gui/glfw_backend.hpp"
#endif
#include "ops/camera_switch_op.hpp"
#include "ops/clear_scene_op.hpp"
#include "ops/create_ik_op.hpp"
#include "ops/create_joint_op.hpp"
#include "ops/create_locator_op.hpp"
#include "ops/cycle_controller_curve_op.hpp"
#include "ops/delete_op.hpp"
#include "ops/display_mode_switch.hpp"
#include "ops/load_model_op.hpp"
#include "ops/mirror_op.hpp"
#include "ops/move_op.hpp"
#include "ops/rotate_op.hpp"
#include "ops/scale_op.hpp"
#include "ops/select_op.hpp"
#include "ops/toggle_controller_attachment_op.hpp"
#include "runtime_config.hpp"
#include "vp/auto_weight_feature.hpp"
#include "vp/controller_feature.hpp"
#include "vp/deformer_feature.hpp"
#include "vp/solver_feature.hpp"
#include "vp/frame_axis.hpp"
#include "vp/grid.hpp"
#include "vp/joint_feature.hpp"
#include "vp/joint_picking_feature.hpp"
#include "vp/locator_feature.hpp"
#include "vp/mesh_csr_feature.hpp"
#include "vp/mesh_picking_feature.hpp"
#include "vp/ortho_grid_feature.hpp"
#include "vp/runtime_hud_feature.hpp"
#include "vp/scene_mesh_feature.hpp"
#include "vp/transform_guide_feature.hpp"
#include "vp/viewport.hpp"

namespace {

constexpr std::uint32_t kViewportWidth = 1200;
constexpr std::uint32_t kViewportHeight = 800;

// Semantic directions encoded by ORL::Frame, expressed in a shared world:
// +X right, +Y up, +Z in (toward the viewer), matching frame_gl / OpenGL / Maya.
glm::vec3 direction_from_frame_axis(std::uint32_t dir) {
    switch (dir) {
    case ORL::Frame::dir_right:
        return {1.0f, 0.0f, 0.0f};
    case ORL::Frame::dir_left:
        return {-1.0f, 0.0f, 0.0f};
    case ORL::Frame::dir_up:
        return {0.0f, 1.0f, 0.0f};
    case ORL::Frame::dir_down:
        return {0.0f, -1.0f, 0.0f};
    case ORL::Frame::dir_in:
        return {0.0f, 0.0f, 1.0f};
    case ORL::Frame::dir_out:
        return {0.0f, 0.0f, -1.0f};
    default:
        return {0.0f, 0.0f, 0.0f};
    }
}

struct ViewportFrame {
    std::array<glm::vec3, 3> axes{};
    glm::vec3 camera_pos{0.0f};
    glm::vec3 camera_front{0.0f, 0.0f, -1.0f};
    glm::vec3 camera_up{0.0f, 1.0f, 0.0f};
    bool right_handed = true;
};

ViewportFrame make_viewport_frame(const ORL::Frame& frame) {
    // Handedness is measured in the shared semantic space (right/up/in).
    // The viewport itself uses the Frame's local XYZ, so the gizmo axes stay
    // +X/+Y/+Z of that Frame and the camera sits in its +++ octant. Embedding
    // dir_out as (0,0,-1) *and* moving the camera to that flipped octant
    // cancels, so a left-handed Frame still looks right-handed on screen.
    const glm::vec3 semantic_x = direction_from_frame_axis(ORL::Frame::get_axis_x_index(frame.flag));
    const glm::vec3 semantic_y = direction_from_frame_axis(ORL::Frame::get_axis_y_index(frame.flag));
    const glm::vec3 semantic_z = direction_from_frame_axis(ORL::Frame::get_axis_z_index(frame.flag));

    ViewportFrame viewport_frame;
    viewport_frame.axes = {
        glm::vec3{1.0f, 0.0f, 0.0f},
        glm::vec3{0.0f, 1.0f, 0.0f},
        glm::vec3{0.0f, 0.0f, 1.0f},
    };
    viewport_frame.right_handed = glm::dot(glm::cross(semantic_x, semantic_y), semantic_z) > 0.0f;
    viewport_frame.camera_pos = glm::vec3{3.0f, 3.0f, 3.0f};
    viewport_frame.camera_front = glm::normalize(-viewport_frame.camera_pos);
    viewport_frame.camera_up = glm::vec3{0.0f, 1.0f, 0.0f};
    return viewport_frame;
}

vkkk::vp::CoordinateSystem make_coordinate_system(const ViewportFrame& viewport_frame) {
    return {
        viewport_frame.axes,
        {"x", "y", "z"},
    };
}

struct StartupOptions {
    bool force_recompile = false;
};

StartupOptions parse_startup_options(int argc, char** argv) {
    StartupOptions options;
    for (int index = 1; index < argc; ++index) {
        const std::string_view argument{argv[index]};
        if (argument == "--recompile"
            || argument == "--recompile-shaders")
        {
            options.force_recompile = true;
        }
    }
    return options;
}

} // namespace

int main(int argc, char** argv) {
    const auto startup = parse_startup_options(argc, argv);
    const auto working_directory = std::filesystem::current_path();
    const auto spirv_cache_directory =
        working_directory / ".orlviewer_spirv_cache";
    const auto pipeline_cache_path =
        working_directory / "orlviewer.pipeline.cache";
#if ORL_USE_QT6
    vkkk::QtBackend window_backend(kViewportWidth, kViewportHeight, "ORL Viewport");
#else
    vkkk::GlfwBackend window_backend(kViewportWidth, kViewportHeight, "ORL Viewport", true);
#endif
    vkkk::Context context;
    context.set_shader_cache(
        spirv_cache_directory, startup.force_recompile);
    context.set_pipeline_cache_path(
        pipeline_cache_path, startup.force_recompile);
    context.init(window_backend,
                 "ORL",
                 VK_MAKE_VERSION(0, 1, 0),
                 "ORL Viewport",
                 vk::ApiVersion13,
                 true,
                 {},
                 {});

    // Viewport world follows the provided ORL Frame. frame_dx is left-handed:
    // +X right, +Y up, +Z out (DirectX-style).
    const ORL::Frame world_frame = ORL::frame_dx;
    const ViewportFrame viewport_frame = make_viewport_frame(world_frame);

    vkkk::Camera camera{
        viewport_frame.camera_pos,
        viewport_frame.camera_front,
        viewport_frame.camera_up,
        45.0f,
        kViewportWidth / static_cast<float>(kViewportHeight),
        0.1f,
        100.0f,
    };

    vkkk::Scene scene;
    scene.camera = &camera;
    ORL::ComponentManager components;
    const auto weight_id = components.create_weight("weights");
    const auto deformer_id = components.create_deformer("deformer");
    ORL::SceneGraphContext scene_graph(scene, components);
    orlgraph::GraphModule empty_graph;
    empty_graph.module_id = "orlrig.scene";
    orlgraph::NodeRegistry node_registry;
    std::string registry_error;
    if (!orlrig::register_rig_node_definitions(
            node_registry, &registry_error))
    {
        std::cerr << "Failed to register rig graph nodes: "
            << registry_error << '\n';
        return 1;
    }
    scene_graph.set_graph(
        std::move(empty_graph), std::move(node_registry));
#if ORL_USE_QT6
    auto* node_graph_editor = new ORL::NodeGraphEditor();
    node_graph_editor->set_scene_graph_context(&scene_graph);
    node_graph_editor->set_project_context(
        &components, weight_id, deformer_id);
    if (window_backend.add_dock_panel(node_graph_editor, "Node Graph") < 0) {
        delete node_graph_editor;
        node_graph_editor = nullptr;
    }
#endif
    ORL::Selection selection(components, scene);
#if ORL_USE_QT6
    if (node_graph_editor != nullptr) {
        node_graph_editor->set_selection(&selection);
    }
    auto* open_project_shortcut = new QShortcut(
        QKeySequence(QStringLiteral("Ctrl+O")),
        window_backend.main_window());
    open_project_shortcut->setContext(Qt::ApplicationShortcut);
    QObject::connect(open_project_shortcut, &QShortcut::activated,
        [node_graph_editor] {
            if (node_graph_editor != nullptr) {
                node_graph_editor->load_project_file();
            }
        });
    auto* property_editor = new ORL::PropertyEditor(selection, components);
    if (window_backend.set_hud_panel(property_editor, "Properties") < 0) {
        delete property_editor;
        property_editor = nullptr;
    }
#endif

    using Viewport = vkkk::vp::Viewport<
        vkkk::vp::GridFeature,
        ORL::OrthoGridFeature,
        ORL::SceneMeshFeature,
        ORL::JointFeature,
        ORL::ControllerFeature,
        ORL::MeshPickingFeature,
        ORL::MeshCsrFeature,
        ORL::AutoWeightFeature,
        ORL::SolverFeature,
        ORL::JointPickingFeature,
        ORL::DeformerFeature,
        ORL::LocatorFeature,
        vkkk::vp::FrameAxisFeature,
        ORL::RuntimeHudFeature,
        ORL::TransformGuideFeature>;
    Viewport viewport(context);
    const std::filesystem::path font_path =
        std::filesystem::path{ORL_VKKK_SOURCE_DIR} / "resource/font/Roboto-Light.ttf";
    const auto grid_handle = viewport.add_feature<vkkk::vp::GridFeature>(camera);
    const auto mesh_handle = viewport.add_feature<ORL::SceneMeshFeature>(
        scene, viewport_frame.right_handed,
        std::filesystem::path{ORL_RESOURCE_DIR} / "shaders", selection);
    const auto csr_handle = viewport.add_feature<ORL::MeshCsrFeature>(
        scene, std::filesystem::path{ORL_RESOURCE_DIR} / "shaders");
    const auto auto_weight_handle = viewport.add_feature<ORL::AutoWeightFeature>(
        scene, components, weight_id, selection);
    if (auto* auto_weight = viewport.find_feature(auto_weight_handle)) {
        if (auto* csr = viewport.find_feature(csr_handle)) {
            auto_weight->set_csr(*csr);
        }
    }
    viewport.add_feature<ORL::SolverFeature>(
        scene_graph, components, selection);
    const auto deformer_handle = viewport.add_feature<ORL::DeformerFeature>(
        scene_graph, deformer_id, weight_id, selection);
    const auto joint_feature_handle = viewport.add_feature<ORL::JointFeature>(
        scene_graph, components, camera,
        std::filesystem::path{ORL_RESOURCE_DIR} / "shaders");
    viewport.add_feature<ORL::LocatorFeature>(
        components, camera,
        std::filesystem::path{ORL_RESOURCE_DIR} / "shaders", selection);
    viewport.add_feature<ORL::ControllerFeature>(
        components, camera, selection, std::filesystem::path{ORL_RESOURCE_DIR} / "shaders");
    const auto axis_handle = viewport.add_feature<vkkk::vp::FrameAxisFeature>(
        camera, font_path, make_coordinate_system(viewport_frame));
    viewport.add_feature<ORL::RuntimeHudFeature>();

    ORL::CameraNavigator navigator(camera, world_frame, viewport_frame.right_handed);
    navigator.update_ubo();
    bool show_grid = true;
    const auto ortho_grid_handle = viewport.add_feature<ORL::OrthoGridFeature>(
        navigator, std::filesystem::path{ORL_RESOURCE_DIR} / "shaders");
    ORL::LoadModelOp load_model(scene, context, &window_backend, world_frame);
    ORL::ClearSceneOp clear_scene(scene, context, components, selection, weight_id, deformer_id);
    ORL::CreateJointOp create_joint(components, camera, navigator.target, &window_backend, selection);
    ORL::ExtendJointChainOp extend_joint_chain(create_joint);
    if (auto* joint_feature = viewport.find_feature(joint_feature_handle)) {
        joint_feature->set_preview_source(create_joint);
    }
    ORL::CreateLocatorOp create_locator(
        components, navigator, &window_backend, selection);
    ORL::DeleteOp delete_op(components, scene, selection);
    ORL::MirrorOp mirror_op(components, selection);
    ORL::ToggleControllerAttachmentOp toggle_controller_attachment(
        components, selection);
    ORL::CycleControllerCurveOp cycle_controller_curve(components, selection);
    ORL::CreateIkOp create_ik(components, selection);
    ORL::SelectOp select_op(
        selection, components, scene, camera, &window_backend, create_joint);
    const auto joint_pick_handle = viewport.add_feature<ORL::JointPickingFeature>(
        scene_graph, components, camera,
        std::filesystem::path{ORL_RESOURCE_DIR} / "shaders");
    if (auto* gpu_pick = viewport.find_feature(joint_pick_handle)) {
        select_op.set_gpu_picking(*gpu_pick);
    }
    const auto mesh_pick_handle = viewport.add_feature<ORL::MeshPickingFeature>(
        scene, camera, std::filesystem::path{ORL_RESOURCE_DIR} / "shaders");
    if (auto* mesh_pick = viewport.find_feature(mesh_pick_handle)) {
        select_op.set_mesh_picking(*mesh_pick);
    }
    ORL::MoveOp move_op(selection, navigator, &window_backend);
    ORL::RotateOp rotate_op(selection, navigator, &window_backend);
    ORL::ScaleOp scale_op(selection, navigator, &window_backend);
    viewport.add_feature<ORL::TransformGuideFeature>(
        move_op, rotate_op, scale_op, camera,
        std::filesystem::path{ORL_RESOURCE_DIR} / "shaders");
    if (auto* csr = viewport.find_feature(csr_handle)) {
        clear_scene.set_csr(*csr);
    }
    if (auto* deformer = viewport.find_feature(deformer_handle)) {
        clear_scene.set_deformer(*deformer);
    }
    if (auto* auto_weight = viewport.find_feature(auto_weight_handle)) {
        clear_scene.set_auto_weight(*auto_weight);
    }
    clear_scene.set_create_joint(create_joint);
    clear_scene.set_create_locator(create_locator);
    clear_scene.set_move(move_op);
    clear_scene.set_rotate(rotate_op);
    clear_scene.set_scale(scale_op);
    ORL::CameraSwitchOp camera_switch(navigator);
    ORL::DisplayModeSwitch display_mode;
    display_mode.register_mode("phong", [&] {
        if (auto* mesh = viewport.find_feature(mesh_handle)) {
            mesh->set_display_mode("phong");
        }
    });
    display_mode.register_mode("xray", [&] {
        if (auto* mesh = viewport.find_feature(mesh_handle)) {
            mesh->set_display_mode("xray");
        }
    });
    ORL::ControlMap controls;
    std::size_t edit_scope_depth = 0;
    bool edit_scope_was_enabled = false;
#if ORL_USE_QT6
    const auto panel_has_focus = [](QWidget* panel) {
        auto* focused = QApplication::focusWidget();
        return panel != nullptr && focused != nullptr
            && (focused == panel || panel->isAncestorOf(focused));
    };
    controls.set_active_panel_provider(
        [node_graph_editor, property_editor, panel_has_focus] {
            if (panel_has_focus(node_graph_editor)) {
                return std::string{"node_graph"};
            }
            if (panel_has_focus(property_editor)) {
                return std::string{"properties"};
            }
            return std::string{"viewport"};
        });
#else
    controls.set_active_panel("viewport");
#endif
    controls.set_operation_scope_handler(
        [&](std::string_view, bool entering) {
            if (entering) {
                if (edit_scope_depth++ == 0) {
                    edit_scope_was_enabled =
                        ORL::runtime_config.evaluate_orl;
                    if (edit_scope_was_enabled) {
                        // Keep controller editing in animation mode while
                        // only the kernel pass is paused.
                        ORL::runtime_config.evaluate_orl = false;
                        scene_graph.scene_inputs().set_cuda_evaluation(false);
                    }
                }
                return;
            }
            if (edit_scope_depth == 0) {
                return;
            }
            if (--edit_scope_depth == 0 && edit_scope_was_enabled) {
                ORL::runtime_config.evaluate_orl = true;
                selection.set_controller_input_mode(true);
                scene_graph.request_evaluation();
            }
        });
#if ORL_USE_QT6
    controls.bind_op_variant("graph_stage_solver",
        [node_graph_editor, panel_has_focus](const ORL::InputEvent& event) {
            return node_graph_editor != nullptr
                && panel_has_focus(node_graph_editor)
                && event.key == vkkk::Key::Digit1;
        },
        [node_graph_editor](const ORL::InputEvent&) {
            if (node_graph_editor != nullptr) {
                node_graph_editor->set_stage(orlgraph::GraphStage::Solver);
            }
        });
    controls.bind_op_variant("graph_stage_deformer",
        [node_graph_editor, panel_has_focus](const ORL::InputEvent& event) {
            return node_graph_editor != nullptr
                && panel_has_focus(node_graph_editor)
                && event.key == vkkk::Key::Digit2;
        },
        [node_graph_editor](const ORL::InputEvent&) {
            if (node_graph_editor != nullptr) {
                node_graph_editor->set_stage(orlgraph::GraphStage::Deformer);
            }
        });
    controls.bind_op_variant("display_mode_switch",
        [node_graph_editor, panel_has_focus](const ORL::InputEvent&) {
            return node_graph_editor == nullptr
                || !panel_has_focus(node_graph_editor);
        },
        [&](const ORL::InputEvent& event) {
            display_mode.eval(event);
        });
#else
    controls.bind_op("display_mode_switch", display_mode);
#endif
    controls.bind_op("toggle_grid", [&](const ORL::InputEvent&) {
        show_grid = !show_grid;
        if (auto* ortho_grid = viewport.find_feature(ortho_grid_handle)) {
            ortho_grid->visible = show_grid;
        }
    });
    controls.bind_op("toggle_frame_axis", [&](const ORL::InputEvent&) {
        if (auto* axes = viewport.find_feature(axis_handle)) {
            axes->visible = !axes->visible;
        }
    });
    controls.bind_op("camera_orbit", [&](const ORL::InputEvent& event) {
        navigator.orbit(static_cast<float>(event.dx), static_cast<float>(event.dy));
    });
    controls.bind_op("camera_pan", [&](const ORL::InputEvent& event) {
        navigator.pan(static_cast<float>(event.dx), static_cast<float>(event.dy));
    });
    controls.bind_op("camera_zoom", [&](const ORL::InputEvent& event) {
        navigator.zoom(static_cast<float>(event.scroll_y));
    });
    controls.bind_edit_op("load_model", load_model);
    controls.bind_edit_op("clear_scene", clear_scene);
    controls.bind_edit_op("create_joint", create_joint);
    controls.bind_edit_op("extend_joint_chain", extend_joint_chain);
    controls.bind_edit_op("create_locator", create_locator);
    controls.bind_edit_op("delete_selection", delete_op);
    controls.bind_edit_op("mirror", mirror_op);
    const auto has_target_for_controller_toggle =
        [&selection](const ORL::InputEvent& event) {
            if (event.key != vkkk::Key::C
                || !selection.selected_mesh_name().empty())
            {
                return false;
            }
            bool has_target = false;
            for (const auto& ref : selection.refs()) {
                if (ref.kind == ORL::SelectionRef::Kind::Joint
                    || ref.kind == ORL::SelectionRef::Kind::Locator)
                {
                    if (has_target) {
                        return false;
                    }
                    has_target = true;
                }
                else if (ref.kind == ORL::SelectionRef::Kind::SceneObject
                    || ref.kind == ORL::SelectionRef::Kind::Vector)
                {
                    return false;
                }
            }
            return has_target;
        };
    const auto has_controller_and_target =
        [&selection](const ORL::InputEvent& event) {
            if (event.key != vkkk::Key::B) {
                return false;
            }
            if (!selection.selected_mesh_name().empty()
                || !selection.has_selected_controller())
            {
                return false;
            }
            bool has_target = false;
            for (const auto& ref : selection.refs()) {
                if (ref.kind == ORL::SelectionRef::Kind::Joint
                    || ref.kind == ORL::SelectionRef::Kind::Locator)
                {
                    if (has_target) {
                        return false;
                    }
                    has_target = true;
                }
            }
            return has_target;
        };
    controls.bind_edit_op_variant("toggle_controller_attachment",
        has_target_for_controller_toggle,
        [&](const ORL::InputEvent& event) {
            toggle_controller_attachment.eval(event);
        });
    controls.bind_edit_op_variant("toggle_controller_attachment",
        has_controller_and_target,
        [&](const ORL::InputEvent& event) {
            toggle_controller_attachment.eval(event);
        });
    controls.bind_edit_op("cycle_controller_curve", cycle_controller_curve);
    controls.bind_edit_op("create_ik", create_ik);
    controls.bind_op("toggle_orl_evaluation", [&](const ORL::InputEvent&) {
        ORL::runtime_config.evaluate_orl = !ORL::runtime_config.evaluate_orl;
        selection.set_controller_input_mode(ORL::runtime_config.evaluate_orl);
        if (ORL::runtime_config.evaluate_orl) {
            scene_graph.request_evaluation();
        } else {
            scene_graph.scene_inputs().set_cuda_evaluation(false);
        }
        std::cout << "ORL evaluation: "
                  << (ORL::runtime_config.evaluate_orl ? "enabled" : "disabled")
                  << '\n';
    });
    controls.bind_op("select", select_op);
    // Transform edits are continuous pose/input edits. Keep the solver
    // running while the modal drag updates controller or locator inputs.
    // Structural edits remain bind_edit_op operations and still pause
    // evaluation while their scene/graph state is being rebuilt.
    controls.bind_op("move", move_op);
    controls.bind_op("rotate", rotate_op);
    controls.bind_op("scale", scale_op);
    controls.bind_op("camera_switch", camera_switch);
    const auto has_mesh_and_joint_selection =
        [&selection](const ORL::InputEvent&) {
            return selection.valid_for_bind();
        };
    controls.bind_edit_op_variant("auto_weight",
        has_mesh_and_joint_selection,
        [&](const ORL::InputEvent&) {
            if (auto* auto_weight = viewport.find_feature(auto_weight_handle)) {
                auto_weight->request();
            }
        });
    controls.bind_op("auto_weight_cycle", [&](const ORL::InputEvent&) {
        if (auto* auto_weight = viewport.find_feature(auto_weight_handle)) {
            auto_weight->cycle_algorithm();
        }
    });
    controls.bind_edit_op_variant("setup_deformer",
        has_mesh_and_joint_selection,
        [&](const ORL::InputEvent&) {
            scene_graph.request_operation("bind");
        });

    try {
        controls.load_config(
            std::filesystem::path{ORL_RESOURCE_DIR} / "config" / "control_map.json");
    }
    catch (const std::exception& error) {
        std::cerr << "Failed to load control map: " << error.what() << '\n';
        return 1;
    }
    controls.attach(window_backend);

    while (!window_backend.should_close()) {
        window_backend.poll_events();
        controls.poll();

#if ORL_USE_QT6
        scene_graph.refresh_scene_inputs();
        if (node_graph_editor != nullptr) {
            node_graph_editor->refresh_scene_inputs();
        }
        if (property_editor != nullptr) {
            property_editor->refresh();
        }
#endif
        vkkk::Context::Frame frame{};
        if (!viewport.begin_frame(frame)) {
            continue;
        }

        const auto extent = viewport.extent();
        camera.ratio = static_cast<float>(extent.width) /
                       static_cast<float>(extent.height == 0 ? 1 : extent.height);
        navigator.update_ubo();
        if (auto* grid = viewport.find_feature(grid_handle)) {
            grid->visible = show_grid && !navigator.orthographic;
        }

        viewport.update(frame);
        viewport.record_frame(frame);
        viewport.end_frame(frame);
    }

    context.wait_idle();
    if (!context.save_pipeline_cache(pipeline_cache_path)) {
        std::cerr << "Failed to save viewer pipeline cache to "
                  << pipeline_cache_path << '\n';
    }
    controls.detach();
    return 0;
}
