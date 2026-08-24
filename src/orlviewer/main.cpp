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
#include "component_manager.hpp"
#include "concepts/camera.h"
#include "control_map.hpp"
#include "ops/create_joint_op.hpp"
#include "ops/display_mode_switch.hpp"
#include "ops/load_model_op.hpp"
#include "vp/frame_axis.hpp"
#include "vp/grid.hpp"
#include "vp/joint_feature.hpp"
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

void update_camera_ubo(vkkk::Camera& camera, bool right_handed) {
    if (right_handed) {
        camera.update_ubo_data();
        return;
    }

    // vkkk::Camera is RH (glm::lookAt / glm::perspective). Match a left-handed
    // ORL Frame with LH view/proj, then apply the same Vulkan NDC Y flip.
    camera.ubo_data.view = glm::lookAtLH(camera.pos, camera.pos + camera.front, camera.up);
    glm::mat4 proj = glm::perspectiveLH(
        glm::radians(camera.fov), camera.ratio, camera.near, camera.far);
    proj[1][1] *= -1.0f;
    camera.ubo_data.proj = proj;
}

struct CameraNavigator {
    CameraNavigator(vkkk::Camera& camera, bool right_handed)
        : camera(camera)
        , target(camera.pos + camera.front * glm::length(camera.pos))
        , right_handed(right_handed)
    {
    }

    // glm::cross / angleAxis are right-handed in world space. lookAtLH only
    // mirrors screen X (camera-right = up × front). Horizontal mouse motion
    // must flip; pitch stays around world front × up so vertical does not.
    float screen_x_sign() const {
        return right_handed ? 1.0f : -1.0f;
    }

    glm::vec3 world_right() const {
        return glm::normalize(glm::cross(camera.front, camera.up));
    }

    void orbit(float dx, float dy) {
        const glm::vec3 offset = camera.pos - target;
        const glm::quat yaw = glm::angleAxis(-dx * 0.005f * screen_x_sign(), camera.up);
        const glm::quat pitch = glm::angleAxis(-dy * 0.005f, world_right());
        camera.pos = target + pitch * yaw * offset;
        camera.front = glm::normalize(target - camera.pos);
    }

    void pan(float dx, float dy) {
        const float distance = glm::length(camera.pos - target);
        const glm::vec3 right = world_right();
        const glm::vec3 up = glm::normalize(glm::cross(right, camera.front));
        const glm::vec3 translation =
            (-right * dx * screen_x_sign() + up * dy) * distance * 0.002f;
        camera.pos += translation;
        target += translation;
    }

    void zoom(float amount) {
        const glm::vec3 offset = camera.pos - target;
        const float distance = std::max(0.1f, glm::length(offset) * (1.0f - amount * 0.1f));
        camera.pos = target + glm::normalize(offset) * distance;
        camera.front = glm::normalize(target - camera.pos);
    }

    vkkk::Camera& camera;
    glm::vec3 target{0.0f};
    bool right_handed = true;
};

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
    update_camera_ubo(camera, viewport_frame.right_handed);

    vkkk::Scene scene;
    scene.camera = &camera;
    ORL::ComponentManager components;

    using Viewport = vkkk::vp::Viewport<
        vkkk::vp::GridFeature,
        ORL::SceneMeshFeature,
        ORL::JointFeature,
        vkkk::vp::FrameAxisFeature>;
    Viewport viewport(context);
    const std::filesystem::path font_path =
        std::filesystem::path{ORL_VKKK_SOURCE_DIR} / "resource/font/Roboto-Light.ttf";
    const auto grid_handle = viewport.add_feature<vkkk::vp::GridFeature>(camera);
    const auto mesh_handle = viewport.add_feature<ORL::SceneMeshFeature>(
        scene, viewport_frame.right_handed,
        std::filesystem::path{ORL_RESOURCE_DIR} / "shaders");
    viewport.add_feature<ORL::JointFeature>(
        components, camera, std::filesystem::path{ORL_RESOURCE_DIR} / "shaders");
    const auto axis_handle = viewport.add_feature<vkkk::vp::FrameAxisFeature>(
        camera, font_path, make_coordinate_system(viewport_frame));

    CameraNavigator navigator(camera, viewport_frame.right_handed);
    ORL::LoadModelOp load_model(scene, context, window, world_frame);
    ORL::CreateJointOp create_joint(components, camera, navigator.target, window);
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
        if (auto* grid = viewport.find_feature(grid_handle)) {
            grid->visible = !grid->visible;
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
    controls.bind_op("create_joint", create_joint);
    controls.bind_op("display_mode_switch", display_mode);

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
        update_camera_ubo(camera, viewport_frame.right_handed);

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
