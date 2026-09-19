#pragma once

#include <glm/mat4x4.hpp>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>

#include "camera_navigator.hpp"
#include "gui/input.hpp"

namespace ORL
{

enum class TransformSpace {
    Screen,
    World,
    Local,
};

enum class TransformAxis {
    None = -1,
    X = 0,
    Y = 1,
    Z = 2,
};

// Axis keys first constrain a screen-space operation to a world axis, then
// switch that axis to local space. Pressing the same key in local space
// returns to screen space; another axis changes only the active axis.
struct TransformConstraint {
    TransformSpace space = TransformSpace::Screen;
    TransformAxis axis = TransformAxis::None;

    void reset() {
        space = TransformSpace::Screen;
        axis = TransformAxis::None;
    }

    int axis_index() const {
        return static_cast<int>(axis);
    }

    bool constrained() const {
        return space != TransformSpace::Screen
            && axis != TransformAxis::None;
    }

    bool handle_axis_key(const InputEvent& event) {
        if (event.kind != InputEvent::Kind::Key
            || event.action != vkkk::InputAction::Press)
        {
            return false;
        }

        const auto pressed_axis = axis_for_key(event.key);
        if (pressed_axis == TransformAxis::None) {
            return false;
        }

        if (space == TransformSpace::Screen) {
            space = TransformSpace::World;
            axis = pressed_axis;
            return true;
        }
        if (axis != pressed_axis) {
            axis = pressed_axis;
            return true;
        }
        if (space == TransformSpace::World) {
            space = TransformSpace::Local;
            return true;
        }

        space = TransformSpace::Screen;
        axis = TransformAxis::None;
        return true;
    }

    static TransformAxis axis_for_key(vkkk::Key key) {
        switch (key) {
        case vkkk::Key::X:
            return TransformAxis::X;
        case vkkk::Key::Y:
            return TransformAxis::Y;
        case vkkk::Key::Z:
            return TransformAxis::Z;
        default:
            return TransformAxis::None;
        }
    }
};

inline glm::vec3 transform_axis_vector(TransformAxis axis) {
    switch (axis) {
    case TransformAxis::X:
        return {1.0f, 0.0f, 0.0f};
    case TransformAxis::Y:
        return {0.0f, 1.0f, 0.0f};
    case TransformAxis::Z:
        return {0.0f, 0.0f, 1.0f};
    default:
        return {};
    }
}

inline bool project_axis_window(const CameraNavigator& navigator,
    int width, int height, const glm::vec3& origin, const glm::vec3& axis,
    glm::vec2& direction, float& pixels_per_unit)
{
    const float axis_length = glm::length(axis);
    if (axis_length < 1.0e-6f) {
        return false;
    }

    glm::vec2 origin_window{};
    glm::vec2 axis_window{};
    const glm::vec3 unit_axis = axis / axis_length;
    if (!navigator.project_window(origin, width, height, origin_window)
        || !navigator.project_window(
            origin + unit_axis, width, height, axis_window))
    {
        return false;
    }

    const glm::vec2 projected = axis_window - origin_window;
    pixels_per_unit = glm::length(projected);
    if (pixels_per_unit < 1.0e-6f) {
        return false;
    }
    direction = projected / pixels_per_unit;
    return true;
}

inline glm::mat4 axis_scale_matrix(float factor, glm::vec3 axis) {
    const float axis_length = glm::length(axis);
    if (axis_length < 1.0e-6f) {
        return glm::mat4{1.0f};
    }
    axis /= axis_length;

    glm::mat4 result{1.0f};
    const float delta = factor - 1.0f;
    for (int column = 0; column < 3; ++column) {
        for (int row = 0; row < 3; ++row) {
            result[column][row] += delta * axis[column] * axis[row];
        }
    }
    return result;
}

} // namespace ORL
