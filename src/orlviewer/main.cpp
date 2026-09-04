#include <algorithm>
#include <array>
#include <cstdint>
#include <filesystem>
#include <iostream>

#include <glm/geometric.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include <GLFW/glfw3.h>

#include "ORL/frame.h"

#include "asset_mgr/scene.h"
#include "camera_navigator.hpp"
#include "component_manager.hpp"
#include "concepts/camera.h"
#include "control_map.hpp"
#include "selection.hpp"
#include "ops/camera_switch_op.hpp"
#include "ops/clear_scene_op.hpp"
#include "ops/create_joint_op.hpp"
#include "ops/display_mode_switch.hpp"
#include "ops/load_model_op.hpp"
#include "ops/move_op.hpp"
#include "ops/rotate_op.hpp"
#include "ops/scale_op.hpp"
#include "ops/select_op.hpp"
#include "vp/auto_weight_feature.hpp"
#include "vp/deformer_feature.hpp"
#include "vp/frame_axis.hpp"
#include "vp/grid.hpp"
#include "vp/joint_feature.hpp"
#include "vp/joint_picking_feature.hpp"
#include "vp/mesh_csr_feature.hpp"
#include "vp/mesh_picking_feature.hpp"
#include "vp/ortho_grid_feature.hpp"
#include "vp/runtime_hud_feature.hpp"
#include "vp/scene_mesh_feature.hpp"
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

} // namespace

int main() {
    vkkk::Context context;
    GLFWwindow *window = context.init_glfw(kViewportWidth, kViewportHeight, "ORL Viewport", true);
    const auto glfw_extensions = vkkk::Context::get_glfw_instance_extensions();
    context.init(window,
                 "ORL",
                 VK_MAKE_VERSION(0, 1, 0),
                 "ORL Viewport",
                 vk::ApiVersion13,
                 true,
                 {},
                 glfw_extensions);

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
    ORL::Selection selection(components, scene);

    using Viewport = vkkk::vp::Viewport<
        vkkk::vp::GridFeature,
        ORL::OrthoGridFeature,
        ORL::SceneMeshFeature,
        ORL::JointFeature,
        ORL::JointPickingFeature,
        ORL::MeshPickingFeature,
        ORL::MeshCsrFeature,
        ORL::AutoWeightFeature,
        ORL::DeformerFeature,
        vkkk::vp::FrameAxisFeature,
        ORL::RuntimeHudFeature>;
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
    const auto deformer_handle = viewport.add_feature<ORL::DeformerFeature>(
        scene, components, deformer_id, weight_id, selection);
    viewport.add_feature<ORL::JointFeature>(
        components, camera, std::filesystem::path{ORL_RESOURCE_DIR} / "shaders");
    const auto axis_handle = viewport.add_feature<vkkk::vp::FrameAxisFeature>(
        camera, font_path, make_coordinate_system(viewport_frame));
    viewport.add_feature<ORL::RuntimeHudFeature>();

    ORL::CameraNavigator navigator(camera, world_frame, viewport_frame.right_handed);
    navigator.update_ubo();
    bool show_grid = true;
    const auto ortho_grid_handle = viewport.add_feature<ORL::OrthoGridFeature>(
        navigator, std::filesystem::path{ORL_RESOURCE_DIR} / "shaders");
    ORL::LoadModelOp load_model(scene, context, window, world_frame);
    ORL::ClearSceneOp clear_scene(scene, context, components, selection, weight_id, deformer_id);
    ORL::CreateJointOp create_joint(components, camera, navigator.target, window, selection);
    ORL::SelectOp select_op(selection, components, scene, camera, window, create_joint);
    const auto joint_pick_handle = viewport.add_feature<ORL::JointPickingFeature>(
        components, camera, std::filesystem::path{ORL_RESOURCE_DIR} / "shaders");
    if (auto* gpu_pick = viewport.find_feature(joint_pick_handle)) {
        select_op.set_gpu_picking(*gpu_pick);
    }
    const auto mesh_pick_handle = viewport.add_feature<ORL::MeshPickingFeature>(
        scene, camera, std::filesystem::path{ORL_RESOURCE_DIR} / "shaders");
    if (auto* mesh_pick = viewport.find_feature(mesh_pick_handle)) {
        select_op.set_mesh_picking(*mesh_pick);
    }
    ORL::MoveOp move_op(selection, navigator, window);
    ORL::RotateOp rotate_op(selection, navigator, window);
    ORL::ScaleOp scale_op(selection, navigator, window);
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
    controls.bind_op("load_model", load_model);
    controls.bind_op("clear_scene", clear_scene);
    controls.bind_op("create_joint", create_joint);
    controls.bind_op("select", select_op);
    controls.bind_op("move", move_op);
    controls.bind_op("rotate", rotate_op);
    controls.bind_op("scale", scale_op);
    controls.bind_op("camera_switch", camera_switch);
    controls.bind_op("display_mode_switch", display_mode);
    controls.bind_op("auto_weight", [&](const ORL::InputEvent&) {
        if (auto* auto_weight = viewport.find_feature(auto_weight_handle)) {
            auto_weight->request();
        }
    });
    controls.bind_op("auto_weight_cycle", [&](const ORL::InputEvent&) {
        if (auto* auto_weight = viewport.find_feature(auto_weight_handle)) {
            auto_weight->cycle_algorithm();
        }
    });
    controls.bind_op("setup_deformer", [&](const ORL::InputEvent&) {
        if (auto* deformer = viewport.find_feature(deformer_handle)) {
            deformer->request();
        }
    });

    try {
        controls.load_config(
            std::filesystem::path{ORL_RESOURCE_DIR} / "config" / "control_map.json");
    }
    catch (const std::exception& error) {
        std::cerr << "Failed to load control map: " << error.what() << '\n';
        return 1;
    }
    controls.attach(window);

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        controls.poll();

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
    controls.detach();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
