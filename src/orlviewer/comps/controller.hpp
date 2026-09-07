#pragma once

#include <cmath>
#include <cstddef>
#include <vector>

#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

namespace orlviewer {

// Viewer-only draw shape. ORL never sees this; it only receives the xform.
enum class ControllerShape {
    Curve,
    Polygon,
};

inline constexpr const char* kControllerOrlType = "matrix";
inline constexpr std::size_t kControllerXformStride = sizeof(double) * 16;

// Locator-style rig control. The interaction payload is a 4x4 xform; a curve
// or polygon is only the viewport gizmo. Pass pack_xform() into ORL as matrix.
struct Controller {
    glm::mat4 xform{1.0f};
    ControllerShape shape = ControllerShape::Curve;
};

inline glm::vec3 controller_origin(const Controller& controller) {
    return glm::vec3{controller.xform[3]};
}

inline void set_controller_origin(Controller& controller, const glm::vec3& world) {
    controller.xform[3] = glm::vec4{world, 1.0f};
}

// glm is column-major; ORL matrices are row-major with translation in column 3.
inline void pack_xform(const Controller& controller, double out[16]) {
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            out[row * 4 + col] = static_cast<double>(controller.xform[col][row]);
        }
    }
}

inline Controller make_controller(const glm::vec3& origin,
    ControllerShape shape = ControllerShape::Curve)
{
    Controller controller;
    controller.shape = shape;
    set_controller_origin(controller, origin);
    return controller;
}

inline std::vector<glm::vec3> controller_curve_points(float radius = 0.2f, int segments = 32) {
    std::vector<glm::vec3> points;
    points.reserve(static_cast<std::size_t>(segments));
    constexpr float kPi = 3.14159265f;
    for (int i = 0; i < segments; ++i) {
        const float a = (static_cast<float>(i) / static_cast<float>(segments)) * 2.0f * kPi;
        points.emplace_back(std::cos(a) * radius, std::sin(a) * radius, 0.0f);
    }
    return points;
}

inline std::vector<glm::vec3> controller_polygon_points(float half = 0.15f) {
    return {
        {-half, -half, 0.0f},
        { half, -half, 0.0f},
        { half,  half, 0.0f},
        {-half,  half, 0.0f},
    };
}

inline std::vector<glm::vec3> controller_shape_points(const Controller& controller) {
    return controller.shape == ControllerShape::Polygon
        ? controller_polygon_points()
        : controller_curve_points();
}

inline glm::vec3 controller_world(const Controller& controller, const glm::vec3& local) {
    return glm::vec3{controller.xform * glm::vec4{local, 1.0f}};
}

} // namespace orlviewer
