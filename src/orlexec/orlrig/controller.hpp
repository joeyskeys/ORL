#pragma once

#include <cstddef>

#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

#include "abi.hpp"

namespace orlrig
{

// Authoring-only controller payload. xform is the setup transform used to
// place the control shape in the rig. input_xform records the user animation
// input separately, so setup scale/orientation is never mistaken for a pose.
// New ORL graphs do not consume this payload directly.
struct Controller {
    glm::mat4 xform{1.0f};
    glm::mat4 input_xform{1.0f};
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

inline Controller make_controller(const glm::vec3& origin) {
    Controller controller;
    set_controller_origin(controller, origin);
    return controller;
}

inline glm::vec3 controller_world(const Controller& controller, const glm::vec3& local) {
    return glm::vec3{controller.xform * controller.input_xform
        * glm::vec4{local, 1.0f}};
}

} // namespace orlrig
