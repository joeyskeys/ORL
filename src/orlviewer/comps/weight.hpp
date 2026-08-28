#pragma once

#include <cstdint>

#include "orl_exec.hpp"

namespace ORL
{

// Skin-weight buffer owned by ComponentManager. Layout matches ORL
// bone_indices[] / weights[]: influences_per_vertex consecutive values
// per vertex (closest-bone writes one influence per vertex).
struct WeightData {
    exec::OrlBuffer bone_indices;
    exec::OrlBuffer weights;
    std::int64_t influences_per_vertex = 1;

    WeightData()
        : bone_indices("int", sizeof(std::int64_t))
        , weights("float", sizeof(double))
    {
    }
};

} // namespace ORL
