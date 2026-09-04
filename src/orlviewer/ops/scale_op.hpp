#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include <GLFW/glfw3.h>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/mat4x4.hpp>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

#include "camera_navigator.hpp"
#include "selection.hpp"
#include "vp_operation.hpp"

namespace ORL
{

// Screen-space uniform scale of whatever Selection currently points at.
// Factor is current mouse distance from the projected pivot over the grab
// distance, matching Blender S.
class ScaleOp : public VpOperation<ScaleOp> {
public:
    static constexpr OpMode kMode = OpMode::Modal;

    ScaleOp(Selection& selection, CameraNavigator& navigator, GLFWwindow* window)
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
            start.local_scale = attr.local_scale();
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
        grab_radius = std::max(8.0f, screen_radius(cursor_x, cursor_y));
        accum = 1.0f;
        engaged = true;
        apply();
    }

    void on_confirm() { engaged = false; }

    void on_cancel() {
        accum = 1.0f;
        apply();
        engaged = false;
    }

    void on_eval(const InputEvent& event) {
        if (!engaged || event.kind != InputEvent::Kind::MouseMove) {
            return;
        }
        accum = screen_radius(event.x, event.y) / grab_radius;
        apply();
    }

private:
    struct Start {
        glm::mat4 matrix{1.0f};
        glm::vec3 world_pos{0.0f};
        glm::vec3 local_scale{1.0f};
    };

    float screen_radius(double cursor_x, double cursor_y) const {
        const float dx = static_cast<float>(cursor_x) - pivot_screen.x;
        const float dy = static_cast<float>(cursor_y) - pivot_screen.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    void apply() {
        const auto* focus = selection.focus();
        if (focus == nullptr) {
            return;
        }

        const float factor = std::max(accum, 0.001f);
        const glm::mat4 around = glm::translate(glm::mat4{1.0f}, pivot)
            * glm::scale(glm::mat4{1.0f}, glm::vec3{factor})
            * glm::translate(glm::mat4{1.0f}, -pivot);

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
                *attr.matrix = around * starts[i].matrix;
            }
            else {
                attr.set_world_position(glm::vec3{around * glm::vec4{starts[i].world_pos, 1.0f}});
                if (attr.scale != nullptr) {
                    attr.set_local_scale(starts[i].local_scale * factor);
                }
            }
            ++i;
        }
    }

    Selection& selection;
    CameraNavigator& navigator;
    GLFWwindow* window = nullptr;
    bool engaged = false;
    float accum = 1.0f;
    float grab_radius = 8.0f;
    glm::vec3 pivot{0.0f};
    glm::vec2 pivot_screen{0.0f};
    std::vector<Start> starts;
};

} // namespace ORL
