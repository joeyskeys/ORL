#pragma once

#include <cstddef>
#include <cstdint>

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
inline constexpr const char* kLocatorOrlType = "Locator";
inline constexpr std::size_t kLocatorStride = kMatrixStride;
// Kept as a compatibility alias for graphs created before controllers became
// viewer-only attachments.
inline constexpr const char* kControllerOrlType = kMatrixOrlType;
inline constexpr std::size_t kControllerXformStride = kMatrixStride;
inline constexpr const char* kSolverContextOrlType = "SolverContext";

struct SolverContext {
    std::int64_t joint_count = 0;
    std::int64_t controller_count = 0;
    std::int64_t locator_count = 0;
    // Byte offsets from the beginning of the bound SolverContext storage.
    // These are offsets rather than host/device pointers so the same packed
    // arena can be used by both the CPU JIT and CUDA kernels.
    std::int64_t joints_offset = 0;
    std::int64_t controllers_offset = 0;
    std::int64_t locators_offset = 0;
};

inline constexpr std::size_t kSolverContextStride = sizeof(SolverContext);

static_assert(offsetof(SolverContext, joint_count) == 0);
static_assert(offsetof(SolverContext, controller_count) == sizeof(std::int64_t));
static_assert(offsetof(SolverContext, locator_count)
    == sizeof(std::int64_t) * 2);
static_assert(offsetof(SolverContext, joints_offset)
    == sizeof(std::int64_t) * 3);
static_assert(offsetof(SolverContext, controllers_offset)
    == sizeof(std::int64_t) * 4);
static_assert(offsetof(SolverContext, locators_offset)
    == sizeof(std::int64_t) * 5);
static_assert(sizeof(SolverContext) == sizeof(std::int64_t) * 6);

inline constexpr const char* kSolverDispatchContextOrlType =
    "SolverDispatchContext";
inline constexpr const char* kSolverDispatchDataOrlType = "int";

struct SolverDispatchContext {
    std::int64_t level_count = 0;
    std::int64_t range_count = 0;
    std::int64_t index_count = 0;
    std::int64_t range_offset = 0;
    std::int64_t index_offset = 0;
    std::int64_t full_evaluation = 0;
};

inline constexpr std::size_t kSolverDispatchContextStride =
    sizeof(SolverDispatchContext);
inline constexpr std::size_t kSolverDispatchDataStride =
    sizeof(std::int64_t);

static_assert(offsetof(SolverDispatchContext, level_count) == 0);
static_assert(offsetof(SolverDispatchContext, range_count)
    == sizeof(std::int64_t));
static_assert(offsetof(SolverDispatchContext, index_count)
    == sizeof(std::int64_t) * 2);
static_assert(sizeof(SolverDispatchContext) == sizeof(std::int64_t) * 6);

inline constexpr const char* kHierarchyContextOrlType =
    "HierarchyContext";
inline constexpr const char* kHierarchyDataOrlType = "int";
inline constexpr std::size_t kHierarchyDataStride = sizeof(std::int64_t);

struct HierarchyContext {
    std::int64_t joint_count = 0;
    std::int64_t level_count = 0;
    std::int64_t ancestor_count = 0;
    std::int64_t ancestor_storage = 0;
    std::int64_t preorder_offset = 0;
    std::int64_t depth_offset = 0;
    std::int64_t subtree_begin_offset = 0;
    std::int64_t subtree_end_offset = 0;
    std::int64_t level_offsets_offset = 0;
    std::int64_t level_joints_offset = 0;
    std::int64_t parent_joints_offset = 0;
    std::int64_t ancestor_offsets_offset = 0;
    std::int64_t ancestor_joints_offset = 0;
    std::int64_t data_count = 0;
};

inline constexpr std::size_t kHierarchyContextStride =
    sizeof(HierarchyContext);

static_assert(offsetof(HierarchyContext, joint_count) == 0);
static_assert(offsetof(HierarchyContext, level_count)
    == sizeof(std::int64_t));
static_assert(offsetof(HierarchyContext, preorder_offset)
    == sizeof(std::int64_t) * 4);
static_assert(offsetof(HierarchyContext, data_count)
    == sizeof(std::int64_t) * 13);
static_assert(sizeof(HierarchyContext) == sizeof(std::int64_t) * 14);

inline constexpr std::size_t kDefaultWeightCount = 5;

} // namespace orlrig
