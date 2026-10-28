#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <string>
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
#include "orlviewer/rhi/selected_adapter.hpp"
#include "orlviewer/rhi/vkkk_native.hpp"
#if ORL_USE_QT6
#include <QApplication>
#include <QFile>
#include <QStyleFactory>
#include "qt/node_color_theme.hpp"
#include "qt/node_graph_editor.hpp"
#include "qt/property_editor.hpp"
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
#include "vp/joint_feature.hpp"
#include "vp/joint_picking_feature.hpp"
#include "vp/locator_feature.hpp"
#include "vp/mesh_csr_feature.hpp"
#include "vp/mesh_picking_feature.hpp"
#include "vp/ortho_grid_feature.hpp"
#include "vp/reference_features.hpp"
#include "vp/rig_picking_feature.hpp"
#include "vp/runtime_hud_feature.hpp"
#include "vp/scene_mesh_feature.hpp"
#include "vp/transform_guide_feature.hpp"
#include "vp/viewport_host.hpp"

namespace {

constexpr std::uint32_t kViewportWidth = 1200;
constexpr std::uint32_t kViewportHeight = 800;
constexpr const char* kQtThemeName = "dark";

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

ORL::Rhi::FrameAxisSetup make_frame_axis_setup(const ViewportFrame& viewport_frame,
    const std::filesystem::path& font_path)
{
    ORL::Rhi::FrameAxisSetup setup;
    for (std::size_t axis = 0; axis < setup.axes.size(); ++axis) {
        setup.axes[axis] = ORL::to_rhi(viewport_frame.axes[axis]);
    }
    setup.labels = {"x", "y", "z"};
    setup.font_path = font_path.string();
    return setup;
}

ORL::Rhi::CameraState make_camera_state(const vkkk::Camera& camera) {
    ORL::Rhi::CameraState state;
    static_assert(sizeof(state.view) == sizeof(camera.ubo_data.view));
    std::memcpy(state.view.data(), &camera.ubo_data.view, sizeof(state.view));
    std::memcpy(
        state.projection.data(), &camera.ubo_data.proj, sizeof(state.projection));
    return state;
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

#if ORL_USE_QT6
std::filesystem::path qt_theme_file_path(std::string_view suffix) {
    return std::filesystem::path{ORL_RESOURCE_DIR} / "theme"
        / (std::string{kQtThemeName} + std::string{suffix});
}

void setup_qt_theme(QApplication& application) {
    if (auto* fusion_style = QStyleFactory::create(
            QStringLiteral("Fusion")); fusion_style != nullptr)
    {
        application.setStyle(fusion_style);
    }

    const auto theme_path = qt_theme_file_path(".qss");
    QFile theme_file(QString::fromStdString(theme_path.string()));
    if (!theme_file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        std::cerr << "Failed to load Qt theme: "
                  << theme_path << ": "
                  << theme_file.errorString().toStdString() << '\n';
        return;
    }
    application.setStyleSheet(QString::fromUtf8(theme_file.readAll()));
}

#endif

} // namespace

int main(int argc, char** argv) {
    const auto startup = parse_startup_options(argc, argv);
    const auto working_directory = std::filesystem::current_path();
    const auto spirv_cache_directory =
        working_directory / ".orlviewer_spirv_cache";
    const auto pipeline_cache_path =
        working_directory / "orlviewer.pipeline.cache";
#if ORL_USE_QT6
    QApplication application(argc, argv);
    setup_qt_theme(application);
#endif
    ORL::Rhi::AdapterConfig adapter_config;
    adapter_config.initial_extent = {kViewportWidth, kViewportHeight};
    adapter_config.validation = true;
    adapter_config.force_shader_recompile = startup.force_recompile;
    adapter_config.application_name = "ORL";
    adapter_config.window_title = "ORL Viewport";
    adapter_config.shader_directory =
        (std::filesystem::path{ORL_RESOURCE_DIR} / "shaders").string();
    adapter_config.shader_cache_directory = spirv_cache_directory.string();
    adapter_config.pipeline_cache_path = pipeline_cache_path.string();
    adapter_config.clear_color = {0.38f, 0.38f, 0.38f, 1.0f};

    ORL::Rhi::SelectedAdapter adapter;
    if (const auto status = adapter.initialize(adapter_config); !status) {
        std::cerr << "Failed to initialize the "
                  << ORL::Rhi::SelectedAdapter::backend_name()
                  << " RHI adapter: " << status.message << '\n';
        return 1;
    }
    // Legacy vkkk features and operations still consume the adapter-owned
    // native window and context until they move onto adapter calls.
    auto& window_backend = ORL::Rhi::VkkkNative::window(adapter);
    auto& context = ORL::Rhi::VkkkNative::context(adapter);

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
    std::string node_color_theme_error;
    auto node_color_theme = ORL::NodeColorTheme::from_file(
        qt_theme_file_path("_node_colors.json"), &node_color_theme_error);
    if (!node_color_theme_error.empty()) {
        std::cerr << "Qt node color theme warning: "
                  << node_color_theme_error
                  << "; using built-in fallback colors\n";
    }
    auto* node_graph_editor = new ORL::NodeGraphEditor();
    node_graph_editor->set_node_color_theme(std::move(node_color_theme));
    node_graph_editor->set_scene_graph_context(&scene_graph);
    node_graph_editor->set_project_context(
        &components, weight_id, deformer_id);
    if (window_backend.add_dock_panel(node_graph_editor, "Node Graph") < 0) {
        delete node_graph_editor;
        node_graph_editor = nullptr;
    }
#endif
    ORL::Selection selection(components, scene);
    ORL::SelectionModeRegistry selection_modes;
    ORL::register_default_selection_modes(selection_modes);
    if (const auto* mode = selection_modes.active(); mode != nullptr) {
        selection.set_mode(*mode);
    }
#if ORL_USE_QT6
    if (node_graph_editor != nullptr) {
        node_graph_editor->set_selection(&selection);
    }
    auto* property_editor = new ORL::PropertyEditor(
        selection, components, &scene_graph);
    property_editor->set_node_graph_editor(node_graph_editor);
    if (window_backend.set_hud_panel(property_editor, "Inspector") < 0) {
        delete property_editor;
        property_editor = nullptr;
    }
#endif
#if ORL_USE_QT6
    window_backend.set_right_dock_ratio(0.30);
#endif

    using Viewport = ORL::ViewportHost<
        ORL::WorldGridFeature,
        ORL::OrthoGridFeature,
        ORL::SceneMeshFeature,
        ORL::JointFeature,
        ORL::ControllerFeature,
        ORL::MeshPickingFeature,
        ORL::RigPickingFeature,
        ORL::MeshCsrFeature,
        ORL::AutoWeightFeature,
        ORL::SolverFeature,
        ORL::JointPickingFeature,
        ORL::DeformerFeature,
        ORL::LocatorFeature,
        ORL::FrameAxisFeature,
        ORL::RuntimeHudFeature,
        ORL::TransformGuideFeature>;
    Viewport viewport(adapter);
    const std::filesystem::path font_path =
        std::filesystem::path{ORL_VKKK_SOURCE_DIR} / "resource/font/Roboto-Light.ttf";
    const auto grid_handle = viewport.add_feature<ORL::WorldGridFeature>();
    const auto mesh_handle = viewport.add_feature<ORL::SceneMeshFeature>(
        scene, viewport_frame.right_handed, selection);
    const auto csr_handle = viewport.add_feature<ORL::MeshCsrFeature>(scene);
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
        scene_graph, components);
    viewport.add_feature<ORL::LocatorFeature>(
        scene_graph, components, selection);
    viewport.add_feature<ORL::ControllerFeature>(components, selection);
    const auto axis_handle = viewport.add_feature<ORL::FrameAxisFeature>(
        make_frame_axis_setup(viewport_frame, font_path));
    viewport.add_feature<ORL::RuntimeHudFeature>();

    ORL::CameraNavigator navigator(camera, world_frame, viewport_frame.right_handed);
    navigator.update_ubo();
    bool show_grid = true;
    const auto ortho_grid_handle = viewport.add_feature<ORL::OrthoGridFeature>(navigator);
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
        scene_graph, components);
    if (auto* gpu_pick = viewport.find_feature(joint_pick_handle)) {
        select_op.set_gpu_picking(*gpu_pick);
    }
    const auto mesh_pick_handle = viewport.add_feature<ORL::MeshPickingFeature>(scene);
    if (auto* mesh_pick = viewport.find_feature(mesh_pick_handle)) {
        select_op.set_mesh_picking(*mesh_pick);
    }
    const auto rig_pick_handle = viewport.add_feature<ORL::RigPickingFeature>(components);
    if (auto* rig_pick = viewport.find_feature(rig_pick_handle)) {
        select_op.set_rig_picking(*rig_pick);
    }
    ORL::MoveOp move_op(selection, navigator, &window_backend);
    ORL::RotateOp rotate_op(selection, navigator, &window_backend);
    ORL::ScaleOp scale_op(selection, navigator, &window_backend);
    viewport.add_feature<ORL::TransformGuideFeature>(
        move_op, rotate_op, scale_op, camera);
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
    const auto sync_graph_editor_evaluation_state =
        [node_graph_editor] {
        if (node_graph_editor == nullptr) {
            return;
        }
        ORL::GraphEvaluationUiState state =
            ORL::GraphEvaluationUiState::Editing;
        switch (ORL::runtime_config.evaluation_phase) {
        case ORL::OrlEvaluationPhase::Disabled:
            state = ORL::GraphEvaluationUiState::Editing;
            break;
        case ORL::OrlEvaluationPhase::Compiling:
            state = ORL::GraphEvaluationUiState::Compiling;
            break;
        case ORL::OrlEvaluationPhase::AwaitingOrder:
            state = ORL::GraphEvaluationUiState::AwaitingOrder;
            break;
        case ORL::OrlEvaluationPhase::Evaluating:
            state = ORL::GraphEvaluationUiState::Evaluating;
            break;
        }
        node_graph_editor->set_evaluation_ui_state(state);
    };
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
    controls.bind_op_variant("create_node_frame",
        [node_graph_editor, panel_has_focus](const ORL::InputEvent&) {
            return node_graph_editor != nullptr
                && panel_has_focus(node_graph_editor);
        },
        [node_graph_editor](const ORL::InputEvent&) {
            if (node_graph_editor != nullptr) {
                node_graph_editor->create_frame_from_selection();
            }
        });
    controls.bind_op_variant("copy_node_graph",
        [node_graph_editor, panel_has_focus](const ORL::InputEvent&) {
            return node_graph_editor != nullptr
                && panel_has_focus(node_graph_editor);
        },
        [node_graph_editor](const ORL::InputEvent&) {
            if (node_graph_editor != nullptr) {
                node_graph_editor->copy_selected_nodes();
            }
        });
    controls.bind_op_variant("paste_node_graph",
        [node_graph_editor, panel_has_focus](const ORL::InputEvent&) {
            return node_graph_editor != nullptr
                && panel_has_focus(node_graph_editor);
        },
        [node_graph_editor](const ORL::InputEvent&) {
            if (node_graph_editor != nullptr) {
                node_graph_editor->paste_nodes();
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
#if ORL_USE_QT6
    controls.bind_op("open_project",
        [node_graph_editor](const ORL::InputEvent&) {
            if (node_graph_editor != nullptr) {
                node_graph_editor->load_project_file();
            }
        });
    controls.bind_op("save_project",
        [node_graph_editor](const ORL::InputEvent&) {
            if (node_graph_editor != nullptr) {
                node_graph_editor->save_project_file(false);
            }
        });
    controls.bind_op("save_project_as",
        [node_graph_editor](const ORL::InputEvent&) {
            if (node_graph_editor != nullptr) {
                node_graph_editor->save_project_file(true);
            }
        });
#endif
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
        ORL::runtime_config.evaluation_phase =
            ORL::runtime_config.evaluate_orl
            ? ORL::OrlEvaluationPhase::Compiling
            : ORL::OrlEvaluationPhase::Disabled;
        selection.set_controller_input_mode(ORL::runtime_config.evaluate_orl);
        if (ORL::runtime_config.evaluate_orl) {
            scene_graph.request_evaluation();
        } else {
            scene_graph.scene_inputs().set_cuda_evaluation(false);
        }
        std::cout << "ORL evaluation: "
                  << (ORL::runtime_config.evaluate_orl ? "enabled" : "disabled")
                  << '\n';
#if ORL_USE_QT6
        sync_graph_editor_evaluation_state();
        if (ORL::runtime_config.evaluation_phase
                == ORL::OrlEvaluationPhase::Compiling
            && node_graph_editor != nullptr)
        {
            node_graph_editor->repaint();
        }
#endif
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

    int exit_code = 0;
    while (!adapter.should_close()) {
        adapter.poll_events();
        controls.poll();

#if ORL_USE_QT6
        scene_graph.refresh_scene_inputs();
        if (node_graph_editor != nullptr
            && ORL::runtime_config.evaluation_phase
                == ORL::OrlEvaluationPhase::AwaitingOrder)
        {
            node_graph_editor->resolve_order_conflicts();
        }
        if (node_graph_editor != nullptr) {
            node_graph_editor->refresh_scene_inputs();
        }
        if (property_editor != nullptr) {
            if (panel_has_focus(node_graph_editor)) {
                property_editor->set_context(
                    ORL::PropertyEditor::Context::NodeGraph);
            } else if (panel_has_focus(window_backend.viewport_panel())) {
                property_editor->set_context(
                    ORL::PropertyEditor::Context::Scene);
            }
            property_editor->refresh();
        }
#endif
        ORL::Rhi::FrameState frame;
        if (const auto status = viewport.begin_frame(frame); !status) {
            if (status.code == ORL::Rhi::StatusCode::drawable_unavailable) {
                continue;
            }
            std::cerr << "Viewer frame could not begin: " << status.message << '\n';
            exit_code = 1;
            break;
        }

        camera.ratio = static_cast<float>(frame.extent.width) /
                       static_cast<float>(frame.extent.height == 0 ? 1 : frame.extent.height);
        navigator.update_ubo();
        if (const auto status = adapter.set_camera(make_camera_state(camera)); !status) {
            std::cerr << "Viewer camera update failed: " << status.message << '\n';
        }
        if (create_joint.is_active()) {
            create_joint.refresh_preview();
        }
        if (auto* grid = viewport.find_feature(grid_handle)) {
            grid->visible = show_grid && !navigator.orthographic;
        }

        viewport.update(frame);
#if ORL_USE_QT6
        sync_graph_editor_evaluation_state();
#endif
        if (const auto status = viewport.end_frame(frame); !status) {
            std::cerr << "Viewer frame could not be submitted: " << status.message << '\n';
            exit_code = 1;
            break;
        }
    }

    if (const auto status = adapter.shutdown(); !status) {
        std::cerr << "Viewer RHI shutdown: " << status.message << '\n';
    }
    controls.detach();
    return exit_code;
}
