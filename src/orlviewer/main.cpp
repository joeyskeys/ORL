#include <array>
#include <cstdint>
#include <filesystem>

#include <glm/geometric.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <GLFW/glfw3.h>

#include "ORL/frame.h"

#include "asset_mgr/scene.h"
#include "concepts/camera.h"
#include "vp/frame_axis.hpp"
#include "vp/grid.hpp"
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
    ViewportFrame viewport_frame;
    viewport_frame.axes[0] = direction_from_frame_axis(ORL::Frame::get_axis_x_index(frame.flag));
    viewport_frame.axes[1] = direction_from_frame_axis(ORL::Frame::get_axis_y_index(frame.flag));
    viewport_frame.axes[2] = direction_from_frame_axis(ORL::Frame::get_axis_z_index(frame.flag));
    viewport_frame.right_handed =
        glm::dot(glm::cross(viewport_frame.axes[0], viewport_frame.axes[1]),
                 viewport_frame.axes[2]) > 0.0f;

    // 3/4 view from the Frame's +++ octant, looking at the origin, Y as up.
    viewport_frame.camera_pos =
        3.0f * (viewport_frame.axes[0] + viewport_frame.axes[1] + viewport_frame.axes[2]);
    viewport_frame.camera_front = glm::normalize(-viewport_frame.camera_pos);
    viewport_frame.camera_up = viewport_frame.axes[1];
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

    using Viewport = vkkk::vp::Viewport<vkkk::vp::GridFeature, vkkk::vp::FrameAxisFeature>;
    Viewport viewport(context);
    const std::filesystem::path font_path =
        std::filesystem::path{ORL_VKKK_SOURCE_DIR} / "resource/font/Roboto-Light.ttf";
    viewport.add_feature<vkkk::vp::GridFeature>(camera);
    viewport.add_feature<vkkk::vp::FrameAxisFeature>(
        camera, font_path, make_coordinate_system(viewport_frame));

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

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
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
