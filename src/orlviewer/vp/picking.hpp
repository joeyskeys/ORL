#pragma once

#include <cstdint>

namespace ORL
{

struct GpuPickHit {
    std::uint32_t id = 0;
    float depth = 0.0f;
};

} // namespace ORL
