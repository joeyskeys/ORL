#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <glm/ext/matrix_clip_space.hpp>
#include <glm/geometric.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/vec3.hpp>

#include "ORL/frame.h"
#include "concepts/camera.h"

namespace ORL
{

// World-space unit vector for a Frame semantic direction (right/up/in/...).
// Viewport world axes are the Frame's local +X/+Y/+Z.
inline glm::vec3 frame_world_direction(const Frame& frame, std::uint32_t semantic_dir) {
    const std::uint32_t encoded[3] = {
        Frame::get_axis_x_index(frame.flag),
        Frame::get_axis_y_index(frame.flag),
        Frame::get_axis_z_index(frame.flag),
    };
    const glm::vec3 world[3] = {
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f},
    };
    for (int i = 0; i < 3; ++i) {
        if ((encoded[i] & Frame::orientation_bits_mask)
            != (semantic_dir & Frame::orientation_bits_mask))
        {
            continue;
        }
        const float sign = ((encoded[i] & 1u) == (semantic_dir & 1u)) ? 1.0f : -1.0f;
        return world[i] * sign;
    }
    return {};
}

struct CameraNavigator {
    CameraNavigator(vkkk::Camera& camera, const Frame& frame, bool right_handed)
        : camera(camera)
        , target(camera.pos + camera.front * glm::length(camera.pos))
        , right_handed(right_handed)
        , semantic_right(frame_world_direction(frame, Frame::dir_right))
        , semantic_up(frame_world_direction(frame, Frame::dir_up))
        , semantic_in(frame_world_direction(frame, Frame::dir_in))
    {
        store_persp();
    }

    enum class OrthoView { None, Front, Right, Top };

    struct ViewState {
        glm::vec3 pos{0.0f};
        glm::vec3 front{0.0f, 0.0f, 1.0f};
        glm::vec3 up{0.0f, 1.0f, 0.0f};
        glm::vec3 target{0.0f};
        float fov = 45.0f;
    };

    float screen_x_sign() const {
        return right_handed ? 1.0f : -1.0f;
    }

    glm::vec3 world_right() const {
        return glm::normalize(glm::cross(camera.front, camera.up));
    }

    static constexpr float kMinOrthoFov = 0.5f;
    static constexpr float kMaxOrthoFov = 90.0f;

    float ortho_half_height() const {
        const float distance = std::max(0.1f, glm::length(camera.pos - target));
        return distance * std::tan(glm::radians(camera.fov) * 0.5f);
    }

    float ortho_park_distance() const {
        return std::max(50.0f, camera.far * 0.5f);
    }

    void store_persp() {
        persp.pos = camera.pos;
        persp.front = camera.front;
        persp.up = camera.up;
        persp.target = target;
        persp.fov = camera.fov;
    }

    void restore_persp() {
        camera.pos = persp.pos;
        camera.front = persp.front;
        camera.up = persp.up;
        target = persp.target;
        camera.fov = persp.fov;
    }

    void leave_orthographic() {
        restore_persp();
        orthographic = false;
        ortho_view = OrthoView::None;
    }

    void orbit(float dx, float dy) {
        if (orthographic) {
            return;
        }
        const glm::vec3 offset = camera.pos - target;
        const glm::quat yaw = glm::angleAxis(-dx * 0.005f * screen_x_sign(), camera.up);
        const glm::quat pitch = glm::angleAxis(-dy * 0.005f, world_right());
        camera.pos = target + pitch * yaw * offset;
        camera.front = glm::normalize(target - camera.pos);
    }

    void pan(float dx, float dy) {
        const float scale = orthographic ? ortho_half_height() : glm::length(camera.pos - target);
        const glm::vec3 right = world_right();
        const glm::vec3 up = glm::normalize(glm::cross(right, camera.front));
        const glm::vec3 translation =
            (-right * dx * screen_x_sign() + up * dy) * scale * 0.002f;
        camera.pos += translation;
        target += translation;
    }

    void zoom(float amount) {
        if (orthographic) {
            camera.fov = std::clamp(camera.fov * (1.0f - amount * 0.1f), kMinOrthoFov, kMaxOrthoFov);
            return;
        }
        const glm::vec3 offset = camera.pos - target;
        const float distance = std::max(0.1f, glm::length(offset) * (1.0f - amount * 0.1f));
        camera.pos = target + glm::normalize(offset) * distance;
        camera.front = glm::normalize(target - camera.pos);
    }

    // Place the camera on the semantic "from" axis, looking at the pivot.
    // Numpad 1/3/7: front (from in), right (from right), top (from up).
    // Pressing the same view key again returns to perspective.
    void look_front() { toggle_look(OrthoView::Front, semantic_in, semantic_up); }
    void look_right() { toggle_look(OrthoView::Right, semantic_right, semantic_up); }
    void look_top() { toggle_look(OrthoView::Top, semantic_up, semantic_in); }

    void look_from(glm::vec3 from, glm::vec3 up) {
        const float from_len = glm::length(from);
        const float up_len = glm::length(up);
        if (from_len < 1.0e-6f || up_len < 1.0e-6f) {
            return;
        }
        from /= from_len;
        up /= up_len;
        if (std::abs(glm::dot(from, up)) > 0.99f) {
            up = std::abs(glm::dot(from, semantic_in)) > 0.99f ? semantic_right : semantic_in;
        }

        if (!orthographic) {
            store_persp();
        }
        const float half_h = ortho_half_height();
        const float parked = ortho_park_distance();
        camera.fov = std::clamp(
            glm::degrees(2.0f * std::atan(half_h / parked)), kMinOrthoFov, kMaxOrthoFov);
        camera.pos = target + from * parked;
        camera.front = glm::normalize(target - camera.pos);
        camera.up = glm::normalize(up - camera.front * glm::dot(up, camera.front));
        orthographic = true;
    }

    void toggle_look(OrthoView view, glm::vec3 from, glm::vec3 up) {
        if (orthographic && ortho_view == view) {
            leave_orthographic();
            return;
        }
        look_from(from, up);
        ortho_view = view;
    }

    void update_ubo() {
        if (!orthographic && right_handed) {
            camera.update_ubo_data();
            return;
        }

        if (right_handed) {
            camera.ubo_data.view = glm::lookAt(camera.pos, camera.pos + camera.front, camera.up);
        }
        else {
            camera.ubo_data.view = glm::lookAtLH(camera.pos, camera.pos + camera.front, camera.up);
        }

        if (!orthographic) {
            glm::mat4 proj = glm::perspectiveLH(
                glm::radians(camera.fov), camera.ratio, camera.near, camera.far);
            proj[1][1] *= -1.0f;
            camera.ubo_data.proj = proj;
            return;
        }

        const float distance = std::max(0.1f, glm::length(camera.pos - target));
        const float half_h = distance * std::tan(glm::radians(camera.fov) * 0.5f);
        const float half_w = half_h * std::max(camera.ratio, 1.0e-4f);
        const float ortho_near = std::min(camera.near, distance * 0.01f);
        const float ortho_far = std::max(camera.far, distance * 2.0f);
        // Vulkan clips NDC z to [0, 1]. Default GLM ortho uses OpenGL [-1, 1],
        // which puts the look plane (and the XZ grid in top view) at z <= 0.
        glm::mat4 proj = right_handed
            ? glm::orthoRH_ZO(-half_w, half_w, -half_h, half_h, ortho_near, ortho_far)
            : glm::orthoLH_ZO(-half_w, half_w, -half_h, half_h, ortho_near, ortho_far);
        proj[1][1] *= -1.0f;
        camera.ubo_data.proj = proj;
    }

    vkkk::Camera& camera;
    glm::vec3 target{0.0f};
    bool right_handed = true;
    bool orthographic = false;
    OrthoView ortho_view = OrthoView::None;
    ViewState persp;
    glm::vec3 semantic_right{1.0f, 0.0f, 0.0f};
    glm::vec3 semantic_up{0.0f, 1.0f, 0.0f};
    glm::vec3 semantic_in{0.0f, 0.0f, 1.0f};
};

} // namespace ORL
