#pragma once

#include <cstddef>
#include <vector>

#include <glm/vec3.hpp>

#include "../../orlexec/orlrig/controller.hpp"
#include "controller_curves.hpp"

namespace orlviewer {

inline constexpr const char* kControllerOrlType = orlrig::kControllerOrlType;
inline constexpr std::size_t kControllerXformStride = orlrig::kControllerXformStride;

inline glm::vec3 controller_origin(const orlrig::Controller& controller) {
    return orlrig::controller_origin(controller);
}

inline void set_controller_origin(orlrig::Controller& controller, const glm::vec3& world) {
    orlrig::set_controller_origin(controller, world);
}

inline void pack_xform(const orlrig::Controller& controller, double out[16]) {
    orlrig::pack_xform(controller, out);
}

inline std::vector<glm::vec3> controller_polygon_points(float half = 0.15f) {
    return {
        {-half, -half, 0.0f},
        { half, -half, 0.0f},
        { half,  half, 0.0f},
        {-half,  half, 0.0f},
    };
}

inline std::vector<glm::vec3> controller_shape_points(ControllerShape shape) {
    return controller_curves::points(shape);
}

inline glm::vec3 controller_world(
    const orlrig::Controller& controller, const glm::vec3& local)
{
    return glm::vec3{controller.xform * glm::vec4{local, 1.0f}};
}

} // namespace orlviewer
