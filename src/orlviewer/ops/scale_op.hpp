#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

#include <glm/gtc/matrix_transform.hpp>
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

// Screen-space uniform scale of whatever Selection currently points at.
// Factor is current mouse distance from the projected pivot over the grab
// distance, matching Blender S.
class ScaleOp : public VpOperation<ScaleOp> {
public:
    static constexpr OpMode kMode = OpMode::Modal;

    ScaleOp(Selection& selection, CameraNavigator& navigator, vkkk::WindowBackend* window)
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
            start.local_scale = attr.local_scale();
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
        grab_radius = std::max(8.0f, screen_radius(pointer.x, pointer.y));
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
        glm::vec3 local_scale{1.0f};
    };

    float screen_radius(double cursor_x, double cursor_y) const {
        const float dx = static_cast<float>(cursor_x) - pivot_screen.x;
        const float dy = static_cast<float>(cursor_y) - pivot_screen.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    float screen_factor(double cursor_x, double cursor_y) const {
        if (std::abs(cursor_x - grab_screen.x) < 1.0e-6
            && std::abs(cursor_y - grab_screen.y) < 1.0e-6)
        {
            return 1.0f;
        }
        return std::max(
            screen_radius(cursor_x, cursor_y) / grab_radius, 0.001f);
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

    float constrained_factor(double cursor_x, double cursor_y) const {
        const auto size = window->window_size();
        glm::vec2 direction{};
        float pixels_per_unit = 0.0f;
        if (!project_axis_window(
                navigator, static_cast<int>(size.width),
                static_cast<int>(size.height), pivot,
                interaction_axis(), direction, pixels_per_unit))
        {
            return 1.0f;
        }
        const glm::vec2 delta{
            static_cast<float>(cursor_x) - grab_screen.x,
            static_cast<float>(cursor_y) - grab_screen.y,
        };
        return std::max(
            1.0f + glm::dot(delta, direction) / grab_radius, 0.001f);
    }

    void apply(double cursor_x, double cursor_y) {
        const auto* focus = selection.focus();
        if (focus == nullptr) {
            return;
        }

        const float factor = constraint.space == TransformSpace::Screen
            ? screen_factor(cursor_x, cursor_y)
            : constrained_factor(cursor_x, cursor_y);
        const glm::mat4 uniform_around = glm::translate(glm::mat4{1.0f}, pivot)
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
            glm::mat4 around = uniform_around;
            if (constraint.space != TransformSpace::Screen) {
                const glm::vec3 axis =
                    constraint.space == TransformSpace::World
                    ? transform_axis_vector(constraint.axis)
                    : starts[i].local_axes[
                        static_cast<std::size_t>(constraint.axis_index())];
                around = glm::translate(glm::mat4{1.0f}, pivot)
                    * axis_scale_matrix(factor, axis)
                    * glm::translate(glm::mat4{1.0f}, -pivot);
            }
            if (attr.has_world_matrix()) {
                attr.set_world_matrix(around * starts[i].matrix);
            }
            else {
                attr.set_world_position(glm::vec3{around * glm::vec4{starts[i].world_pos, 1.0f}});
                if (attr.scale != nullptr) {
                    auto scale = starts[i].local_scale;
                    if (constraint.space == TransformSpace::Screen) {
                        scale *= factor;
                    }
                    else if (constraint.space == TransformSpace::Local) {
                        scale[constraint.axis_index()] *= factor;
                    }
                    else {
                        int local_axis = 0;
                        float best_alignment = -1.0f;
                        const glm::vec3 world_axis =
                            transform_axis_vector(constraint.axis);
                        for (int axis = 0; axis < 3; ++axis) {
                            const float alignment = std::abs(glm::dot(
                                starts[i].local_axes[
                                    static_cast<std::size_t>(axis)],
                                world_axis));
                            if (alignment > best_alignment) {
                                best_alignment = alignment;
                                local_axis = axis;
                            }
                        }
                        scale[local_axis] *= factor;
                    }
                    attr.set_local_scale(scale);
                }
            }
            ++i;
        }
    }

    Selection& selection;
    CameraNavigator& navigator;
    vkkk::WindowBackend* window = nullptr;
    bool engaged = false;
    float grab_radius = 8.0f;
    glm::vec3 pivot{0.0f};
    glm::vec2 pivot_screen{0.0f};
    glm::vec2 grab_screen{0.0f};
    TransformConstraint constraint;
    std::vector<Start> starts;
};

} // namespace ORL
