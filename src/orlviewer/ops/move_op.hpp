#pragma once

#include <vector>

#include <GLFW/glfw3.h>
#include <glm/vec3.hpp>

#include "camera_navigator.hpp"
#include "selection.hpp"
#include "vp_operation.hpp"

namespace ORL
{

// Screen-plane translate of whatever Selection currently points at.
// Motion is unprojected through the current view/proj so it follows the
// active ORL Frame (handedness, ortho/persp) instead of a fixed axis set.
class MoveOp : public VpOperation<MoveOp> {
public:
    static constexpr OpMode kMode = OpMode::Modal;

    MoveOp(Selection& selection, CameraNavigator& navigator, GLFWwindow* window)
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
        // Bind keeps mesh + joints selected together. Transform only the
        // kind last clicked so moving a joint does not drag the mesh object.
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
            starts.push_back(attr.world_position());
        }
        if (starts.empty()) {
            engaged = false;
            return;
        }

        plane_point = starts[0];
        for (std::size_t i = 1; i < starts.size(); ++i) {
            plane_point += starts[i];
        }
        plane_point /= static_cast<float>(starts.size());

        double cursor_x = 0.0;
        double cursor_y = 0.0;
        glfwGetCursorPos(window, &cursor_x, &cursor_y);
        if (!hit(cursor_x, cursor_y, grab_origin)) {
            engaged = false;
            return;
        }

        accum = {0.0f, 0.0f, 0.0f};
        engaged = true;
        apply();
    }

    void on_confirm() { engaged = false; }

    void on_cancel() {
        accum = {0.0f, 0.0f, 0.0f};
        apply();
        engaged = false;
    }

    void on_eval(const InputEvent& event) {
        if (!engaged || event.kind != InputEvent::Kind::MouseMove) {
            return;
        }

        glm::vec3 hit_point{};
        if (!hit(event.x, event.y, hit_point)) {
            return;
        }
        accum = hit_point - grab_origin;
        apply();
    }

private:
    bool hit(double cursor_x, double cursor_y, glm::vec3& world) const {
        int width = 0;
        int height = 0;
        glfwGetWindowSize(window, &width, &height);
        return navigator.view_plane_hit(cursor_x, cursor_y, width, height, plane_point, world);
    }

    void apply() {
        const auto* focus = selection.focus();
        if (focus == nullptr) {
            return;
        }
        std::size_t i = 0;
        for (const auto& ref : selection.refs()) {
            if (ref.kind != focus->kind) {
                continue;
            }
            if (i >= starts.size()) {
                break;
            }
            auto attr = selection.dest(ref);
            if (attr) {
                attr.set_world_position(starts[i] + accum);
            }
            ++i;
        }
    }

    Selection& selection;
    CameraNavigator& navigator;
    GLFWwindow* window = nullptr;
    bool engaged = false;
    glm::vec3 accum{0.0f};
    glm::vec3 plane_point{0.0f};
    glm::vec3 grab_origin{0.0f};
    std::vector<glm::vec3> starts;
};

} // namespace ORL
