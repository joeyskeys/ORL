#pragma once

#include "../../orlexec/orlrig/weight.hpp"

namespace orlviewer
{

using orlrig::Weight;

inline constexpr const char* kWeightOrlType = orlrig::kWeightOrlType;
inline constexpr std::size_t kWeightStride = orlrig::kWeightStride;
inline constexpr std::int64_t kDefaultWeightCount =
    static_cast<std::int64_t>(orlrig::kDefaultWeightCount);

} // namespace orlviewer

namespace ORL
{

using WeightData = orlrig::WeightData;

} // namespace ORL
