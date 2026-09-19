#pragma once

#include <array>
#include <cmath>
#include <vector>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/mat4x4.hpp>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

#include "camera_navigator.hpp"
#include "gui/window_backend.hpp"
#include "selection.hpp"
#include "transform_mode.hpp"
#include "vp_operation.hpp"

namespace ORL
{

// Screen-space rotate of whatever Selection currently points at.
// Angle is measured around the projected pivot; the axis is the view
// direction so it follows the active ORL Frame / ortho camera.
class RotateOp : public VpOperation<RotateOp> {
public:
    static constexpr OpMode kMode = OpMode::Modal;

    RotateOp(Selection& selection, CameraNavigator& navigator, vkkk::WindowBackend* window)
        : selection(selection)
        , navigator(navigator)
        , window(window)
    {
    }

    bool is_active() const { return engaged; }
    bool guide_visible() const {
        return engaged && constraint.constrained();
    }

    TransformAxis guide_axis_kind() const {
        return constraint.axis;
    }

    glm::vec3 guide_axis() const {
        return interaction_axis();
    }

    glm::vec3 guide_origin() const {
        return pivot;
    }

    void on_enter() {
        constraint.reset();
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
            if (attr.has_world_matrix()) {
                start.matrix = attr.world_matrix();
            }
            for (int axis = 0; axis < 3; ++axis) {
                start.local_axes[static_cast<std::size_t>(axis)] =
                    attr.local_axis(axis);
                if (glm::length(start.local_axes[static_cast<std::size_t>(axis)])
                    < 1.0e-6f)
                {
                    start.local_axes[static_cast<std::size_t>(axis)] =
                        transform_axis_vector(
                            static_cast<TransformAxis>(axis));
                }
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
        const auto size = window->window_size();
        const int width = static_cast<int>(size.width);
        const int height = static_cast<int>(size.height);
        if (!navigator.project_window(pivot, width, height, pivot_screen)) {
            engaged = false;
            return;
        }

        const auto pointer = window->pointer();
        grab_screen = {static_cast<float>(pointer.x),
            static_cast<float>(pointer.y)};
        grab_angle = screen_angle(pointer.x, pointer.y);
        engaged = true;
        apply(pointer.x, pointer.y);
    }

    void on_confirm() { engaged = false; }

    void on_cancel() {
        apply(grab_screen.x, grab_screen.y);
        engaged = false;
    }

    void on_eval(const InputEvent& event) {
        if (!engaged) {
            return;
        }
        if (constraint.handle_axis_key(event)) {
            apply(event.x, event.y);
            return;
        }
        if (event.kind != InputEvent::Kind::MouseMove) {
            return;
        }
        apply(event.x, event.y);
    }

private:
    struct Start {
        glm::mat4 matrix{1.0f};
        glm::vec3 world_pos{0.0f};
        std::array<glm::vec3, 3> local_axes{};
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

    glm::vec3 interaction_axis() const {
        if (constraint.space == TransformSpace::World) {
            return transform_axis_vector(constraint.axis);
        }
        if (constraint.space == TransformSpace::Local
            && constraint.axis_index() >= 0
            && !starts.empty())
        {
            return starts.back().local_axes[
                static_cast<std::size_t>(constraint.axis_index())];
        }
        return {};
    }

    bool axis_plane_vector(double cursor_x, double cursor_y,
        const glm::vec3& axis, glm::vec3& vector) const
    {
        const auto size = window->window_size();
        glm::vec3 hit{};
        if (!navigator.plane_hit(cursor_x, cursor_y,
                static_cast<int>(size.width), static_cast<int>(size.height),
                pivot, axis, hit))
        {
            return false;
        }
        vector = hit - pivot;
        const float length = glm::length(vector);
        if (length < 1.0e-6f) {
            return false;
        }
        vector /= length;
        return true;
    }

    float constrained_angle(double cursor_x, double cursor_y) const {
        const glm::vec3 axis = interaction_axis();
        const float axis_length = glm::length(axis);
        if (axis_length < 1.0e-6f) {
            return screen_angle(cursor_x, cursor_y) - grab_angle;
        }
        const glm::vec3 unit_axis = axis / axis_length;
        glm::vec3 start_vector{};
        glm::vec3 current_vector{};
        if (!axis_plane_vector(
                grab_screen.x, grab_screen.y, unit_axis, start_vector)
            || !axis_plane_vector(
                cursor_x, cursor_y, unit_axis, current_vector))
        {
            return screen_angle(cursor_x, cursor_y) - grab_angle;
        }
        return std::atan2(
            glm::dot(glm::cross(start_vector, current_vector), unit_axis),
            glm::dot(start_vector, current_vector));
    }

    void apply(double cursor_x, double cursor_y) {
        const auto* focus = selection.focus();
        if (focus == nullptr) {
            return;
        }

        glm::vec3 axis{};
        float angle = 0.0f;
        if (constraint.space == TransformSpace::Screen) {
            axis = navigator.camera.front;
            angle = (screen_angle(cursor_x, cursor_y) - grab_angle)
                * navigator.screen_x_sign();
        }
        else {
            axis = interaction_axis();
            angle = constrained_angle(cursor_x, cursor_y);
        }
        const float axis_length = glm::length(axis);
        if (axis_length < 1.0e-6f) {
            return;
        }
        axis /= axis_length;

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

            const glm::vec3 object_axis =
                constraint.space == TransformSpace::Screen
                ? axis
                : constraint.space == TransformSpace::World
                ? transform_axis_vector(constraint.axis)
                : starts[i].local_axes[
                    static_cast<std::size_t>(constraint.axis_index())];
            const float object_axis_length = glm::length(object_axis);
            if (object_axis_length < 1.0e-6f) {
                ++i;
                continue;
            }
            const glm::vec3 normalized_object_axis =
                object_axis / object_axis_length;
            const glm::mat4 orbit = glm::translate(glm::mat4{1.0f}, pivot)
                * glm::rotate(glm::mat4{1.0f}, angle, normalized_object_axis)
                * glm::translate(glm::mat4{1.0f}, -pivot);
            const glm::quat delta =
                glm::angleAxis(angle, normalized_object_axis);
            if (attr.has_world_matrix()) {
                attr.set_world_matrix(orbit * starts[i].matrix);
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
    vkkk::WindowBackend* window = nullptr;
    bool engaged = false;
    float grab_angle = 0.0f;
    glm::vec3 pivot{0.0f};
    glm::vec2 pivot_screen{0.0f};
    glm::vec2 grab_screen{0.0f};
    TransformConstraint constraint;
    std::vector<Start> starts;
};

} // namespace ORL
