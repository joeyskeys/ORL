#pragma once

#include <cstddef>
#include <cstdint>

#include "orl_exec.hpp"

namespace orlviewer {

// Shared CPU/GPU skin-weight cell. Field order matches resource/stdlib/weight.orl.
inline constexpr const char* kWeightOrlType = "Weight";
inline constexpr std::size_t kWeightStride = 16;
inline constexpr std::int64_t kDefaultWeightCount = 5;

struct Weight {
    double weight;
    std::int64_t joint;
};

static_assert(sizeof(Weight) == kWeightStride, "Weight stride must stay 16 bytes");
static_assert(offsetof(Weight, joint) == 8, "Weight joint must start at 8");

} // namespace orlviewer

namespace ORL
{

// Skin-weight buffer owned by ComponentManager. Layout is vertex-major:
//   weights[vertex * weight_cnt + slot]
// Length is weight_cnt * vertex_count. Default is 5 cells per vertex.
struct WeightData {
    exec::OrlBuffer weights;
    std::int64_t weight_cnt = orlviewer::kDefaultWeightCount;

    WeightData()
        : weights(orlviewer::kWeightOrlType, orlviewer::kWeightStride)
    {
    }
};

} // namespace ORL
