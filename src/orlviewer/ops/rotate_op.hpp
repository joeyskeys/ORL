#pragma once

#include <cmath>
#include <vector>

#include <GLFW/glfw3.h>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/mat4x4.hpp>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

#include "camera_navigator.hpp"
#include "selection.hpp"
#include "vp_operation.hpp"

namespace ORL
{

// Screen-space rotate of whatever Selection currently points at.
// Angle is measured around the projected pivot; the axis is the view
// direction so it follows the active ORL Frame / ortho camera.
class RotateOp : public VpOperation<RotateOp> {
public:
    static constexpr OpMode kMode = OpMode::Modal;

    RotateOp(Selection& selection, CameraNavigator& navigator, GLFWwindow* window)
        : selection(selection)
        , navigator(navigator)
        , window(window)
    {
    }

    bool is_active() const { return engaged; }

    void on_enter() {
        starts.clear();
        const auto* focus = selection.focus();
        if (focus == nullptr || window == nullptr) {
            engaged = false;
            return;
        }

        glm::vec3 pivot_sum{0.0f};
        for (const auto& ref : selection.refs()) {
            if (ref.kind != focus->kind) {
                continue;
            }
            const auto attr = selection.dest(ref);
            if (!attr) {
                starts.clear();
                engaged = false;
                return;
            }
            Start start;
            start.world_pos = attr.world_position();
            if (attr.kind == XformAttrKind::Matrix && attr.matrix != nullptr) {
                start.matrix = *attr.matrix;
            }
            start.local_rot = attr.local_rotation();
            starts.push_back(start);
            pivot_sum += start.world_pos;
        }
        if (starts.empty()) {
            engaged = false;
            return;
        }

        pivot = pivot_sum / static_cast<float>(starts.size());
        int width = 0;
        int height = 0;
        glfwGetWindowSize(window, &width, &height);
        if (!navigator.project_window(pivot, width, height, pivot_screen)) {
            engaged = false;
            return;
        }

        double cursor_x = 0.0;
        double cursor_y = 0.0;
        glfwGetCursorPos(window, &cursor_x, &cursor_y);
        grab_angle = screen_angle(cursor_x, cursor_y);
        accum = 0.0f;
        engaged = true;
        apply();
    }

    void on_confirm() { engaged = false; }

    void on_cancel() {
        accum = 0.0f;
        apply();
        engaged = false;
    }

    void on_eval(const InputEvent& event) {
        if (!engaged || event.kind != InputEvent::Kind::MouseMove) {
            return;
        }
        accum = screen_angle(event.x, event.y) - grab_angle;
        apply();
    }

private:
    struct Start {
        glm::mat4 matrix{1.0f};
        glm::vec3 world_pos{0.0f};
        glm::quat local_rot{1.0f, 0.0f, 0.0f, 0.0f};
    };

    float screen_angle(double cursor_x, double cursor_y) const {
        const float dx = static_cast<float>(cursor_x) - pivot_screen.x;
        const float dy = static_cast<float>(cursor_y) - pivot_screen.y;
        if (dx * dx + dy * dy < 4.0f) {
            return grab_angle;
        }
        return std::atan2(dy, dx);
    }

    void apply() {
        const auto* focus = selection.focus();
        if (focus == nullptr) {
            return;
        }

        glm::vec3 axis = navigator.camera.front;
        const float axis_len = glm::length(axis);
        if (axis_len < 1.0e-6f) {
            return;
        }
        axis /= axis_len;
        const float angle = accum * navigator.screen_x_sign();
        const glm::mat4 orbit = glm::translate(glm::mat4{1.0f}, pivot)
            * glm::rotate(glm::mat4{1.0f}, angle, axis)
            * glm::translate(glm::mat4{1.0f}, -pivot);
        const glm::quat delta = glm::angleAxis(angle, axis);

        std::size_t i = 0;
        for (const auto& ref : selection.refs()) {
            if (ref.kind != focus->kind) {
                continue;
            }
            if (i >= starts.size()) {
                break;
            }
            auto attr = selection.dest(ref);
            if (!attr) {
                ++i;
                continue;
            }
            if (attr.kind == XformAttrKind::Matrix && attr.matrix != nullptr) {
                *attr.matrix = orbit * starts[i].matrix;
            }
            else {
                attr.set_world_position(glm::vec3{orbit * glm::vec4{starts[i].world_pos, 1.0f}});
                if (attr.rotation != nullptr) {
                    const glm::quat parent = glm::normalize(glm::quat_cast(attr.to_world));
                    const glm::quat world = delta * parent * starts[i].local_rot;
                    attr.set_local_rotation(glm::inverse(parent) * world);
                }
            }
            ++i;
        }
    }

    Selection& selection;
    CameraNavigator& navigator;
    GLFWwindow* window = nullptr;
    bool engaged = false;
    float accum = 0.0f;
    float grab_angle = 0.0f;
    glm::vec3 pivot{0.0f};
    glm::vec2 pivot_screen{0.0f};
    std::vector<Start> starts;
};

} // namespace ORL
