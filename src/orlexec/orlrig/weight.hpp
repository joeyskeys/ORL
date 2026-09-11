#pragma once

#include <cstddef>
#include <cstdint>

#include "abi.hpp"
#include "../orl_exec.hpp"

namespace orlrig
{

struct Weight {
    double weight;
    std::int64_t joint;
};

static_assert(sizeof(Weight) == kWeightStride, "Weight stride must stay 16 bytes");
static_assert(offsetof(Weight, joint) == 8, "Weight joint must start at 8");

// Vertex-major skin weights: weights[vertex * weight_cnt + slot].
struct WeightData {
    ORL::exec::OrlBuffer weights;
    std::int64_t weight_cnt = static_cast<std::int64_t>(kDefaultWeightCount);

    WeightData()
        : weights(kWeightOrlType, kWeightStride)
    {
    }
};

} // namespace orlrig
