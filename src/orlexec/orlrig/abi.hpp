#pragma once

#include <cstddef>

namespace orlrig
{

inline constexpr const char* kJointOrlType = "Joint";
inline constexpr std::size_t kJointStride = 128;

inline constexpr const char* kWeightOrlType = "Weight";
inline constexpr std::size_t kWeightStride = 16;
inline constexpr const char* kPointOrlType = "point";
inline constexpr std::size_t kPointStride = sizeof(double) * 4;
inline constexpr const char* kMatrixOrlType = "matrix";
inline constexpr std::size_t kMatrixStride = sizeof(double) * 16;
inline constexpr const char* kControllerOrlType = kMatrixOrlType;
inline constexpr std::size_t kControllerXformStride = kMatrixStride;
inline constexpr std::size_t kDefaultWeightCount = 5;

} // namespace orlrig
