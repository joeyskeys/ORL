#pragma once

#include <array>
#include <vector>

#include <glm/vec3.hpp>

#include "camera_navigator.hpp"
#include "gui/window_backend.hpp"
#include "selection.hpp"
#include "transform_mode.hpp"
#include "vp_operation.hpp"

namespace ORL
{

// Screen-plane translate of whatever Selection currently points at.
// Motion is unprojected through the current view/proj so it follows the
// active ORL Frame (handedness, ortho/persp) instead of a fixed axis set.
class MoveOp : public VpOperation<MoveOp> {
public:
    static constexpr OpMode kMode = OpMode::Modal;

    MoveOp(Selection& selection, CameraNavigator& navigator, vkkk::WindowBackend* window)
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
        const auto* focus = selection.focus();
        if (!guide_visible() || focus == nullptr) {
            return plane_point;
        }

        glm::vec3 sum{0.0f};
        std::size_t count = 0;
        for (const auto& ref : selection.refs()) {
            if (ref.kind != focus->kind) {
                continue;
            }
            const auto attr = selection.dest(ref);
            if (!attr) {
                continue;
            }
            sum += attr.world_position();
            ++count;
        }
        return count == 0
            ? plane_point
            : sum / static_cast<float>(count);
    }

    void on_enter() {
        constraint.reset();
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
            Start start;
            start.world_position = attr.world_position();
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
            starts.push_back(start);
        }
        if (starts.empty()) {
            engaged = false;
            return;
        }

        plane_point = starts[0].world_position;
        for (std::size_t i = 1; i < starts.size(); ++i) {
            plane_point += starts[i].world_position;
        }
        plane_point /= static_cast<float>(starts.size());

        const auto pointer = window->pointer();
        grab_screen = {static_cast<float>(pointer.x),
            static_cast<float>(pointer.y)};
        if (!hit(pointer.x, pointer.y, grab_origin)) {
            engaged = false;
            return;
        }

        engaged = true;
        apply(pointer.x, pointer.y);
    }

    void on_confirm() { engaged = false; }

    void on_cancel() {
        restore();
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
        glm::vec3 world_position{0.0f};
        std::array<glm::vec3, 3> local_axes{};
    };

    bool hit(double cursor_x, double cursor_y, glm::vec3& world) const {
        const auto size = window->window_size();
        const int width = static_cast<int>(size.width);
        const int height = static_cast<int>(size.height);
        return navigator.view_plane_hit(cursor_x, cursor_y, width, height, plane_point, world);
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

    float constrained_amount(double cursor_x, double cursor_y) const {
        const auto size = window->window_size();
        glm::vec2 direction{};
        float pixels_per_unit = 0.0f;
        if (!project_axis_window(
                navigator, static_cast<int>(size.width),
                static_cast<int>(size.height), plane_point,
                interaction_axis(), direction, pixels_per_unit))
        {
            return 0.0f;
        }
        const glm::vec2 delta{
            static_cast<float>(cursor_x) - grab_screen.x,
            static_cast<float>(cursor_y) - grab_screen.y,
        };
        return glm::dot(delta, direction) / pixels_per_unit;
    }

    void restore() {
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
                attr.set_world_position(starts[i].world_position);
            }
            ++i;
        }
    }

    void apply(double cursor_x, double cursor_y) {
        const auto* focus = selection.focus();
        if (focus == nullptr) {
            return;
        }

        if (constraint.space == TransformSpace::Screen) {
            glm::vec3 hit_point{};
            if (!hit(cursor_x, cursor_y, hit_point)) {
                return;
            }
            const glm::vec3 delta = hit_point - grab_origin;
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
                    attr.set_world_position(starts[i].world_position + delta);
                }
                ++i;
            }
            return;
        }

        const float amount = constrained_amount(cursor_x, cursor_y);
        const int axis_index = constraint.axis_index();
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
                const glm::vec3 axis = constraint.space == TransformSpace::World
                    ? transform_axis_vector(constraint.axis)
                    : starts[i].local_axes[
                        static_cast<std::size_t>(axis_index)];
                attr.set_world_position(
                    starts[i].world_position + axis * amount);
            }
            ++i;
        }
    }

    Selection& selection;
    CameraNavigator& navigator;
    vkkk::WindowBackend* window = nullptr;
    bool engaged = false;
    glm::vec3 plane_point{0.0f};
    glm::vec3 grab_origin{0.0f};
    glm::vec2 grab_screen{0.0f};
    TransformConstraint constraint;
    std::vector<Start> starts;
};

} // namespace ORL
