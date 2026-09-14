#pragma once

#include <cstddef>

#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

#include "abi.hpp"

namespace orlrig
{

// Runtime locator payload. Locators are renderer-independent world-space
// transforms; pack_xform() converts the GLM representation to ORL's
// row-major double matrix ABI.
struct Locator {
    glm::mat4 xform{1.0f};
};

inline glm::vec3 locator_origin(const Locator& locator) {
    return glm::vec3{locator.xform[3]};
}

inline void set_locator_origin(Locator& locator, const glm::vec3& world) {
    locator.xform[3] = glm::vec4{world, 1.0f};
}

inline void pack_xform(const Locator& locator, double out[16]) {
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            out[row * 4 + col] =
                static_cast<double>(locator.xform[col][row]);
        }
    }
}

inline Locator make_locator(const glm::vec3& origin) {
    Locator locator;
    set_locator_origin(locator, origin);
    return locator;
}

static_assert(kLocatorStride == kMatrixStride);

} // namespace orlrig
