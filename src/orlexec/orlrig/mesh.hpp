#pragma once

#include <cstdint>
#include <vector>

#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>

namespace orlrig
{

// Renderer-independent mesh data consumed by rigging algorithms. Positions
// are object-space; model maps them into the rig/world space.
struct MeshData {
    std::vector<glm::vec3> positions;
    std::vector<std::uint32_t> indices;
    glm::mat4 model{1.0f};
};

struct MeshCsrData {
    std::vector<std::uint32_t> offsets;
    std::vector<std::uint32_t> neighbors;
};

} // namespace orlrig
