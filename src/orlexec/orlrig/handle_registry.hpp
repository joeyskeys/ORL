#pragma once

#include "orl_runtime_signature.h"
#include "../../orlcomp/orl_handle_views.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>

namespace orlrig
{

inline constexpr std::string_view kJointHandleCanonical =
    "orlrig::joint_handle";
inline constexpr std::string_view kLocatorHandleCanonical =
    "orlrig::locator_handle";
inline constexpr std::string_view kWorldTransformStruct =
    "WorldTransform";

struct HandleStorage {
    void* data = nullptr;
    std::size_t count = 0;
    std::size_t stride = 0;
    bool writable = false;
};

struct HandleViewContext {
    HandleStorage joints;
    HandleStorage locators;
    std::uint64_t topology_revision = 0;
};

void set_active_handle_context(HandleViewContext* context);

class ScopedHandleViewContext {
public:
    explicit ScopedHandleViewContext(HandleViewContext* context);
    ~ScopedHandleViewContext();

    ScopedHandleViewContext(const ScopedHandleViewContext&) = delete;
    ScopedHandleViewContext& operator=(
        const ScopedHandleViewContext&) = delete;

private:
    HandleViewContext* previous_ = nullptr;
};

bool register_rig_handle_views(
    orlcomp::HandleViewRegistry& registry,
    std::string* error = nullptr);

bool validate_handle_context(
    const HandleViewContext& context,
    std::uint64_t expected_revision,
    std::string* error = nullptr);

bool read_joint_int(const HandleViewContext& context,
    orlcomp::HandleValue handle, std::uint32_t field,
    std::int64_t* value, std::string* error = nullptr);
bool write_joint_int(const HandleViewContext& context,
    orlcomp::HandleValue handle, std::uint32_t field,
    std::int64_t value, std::string* error = nullptr);
bool read_joint_vec4(const HandleViewContext& context,
    orlcomp::HandleValue handle, std::uint32_t field,
    double value[4], std::string* error = nullptr);
bool write_joint_vec4(const HandleViewContext& context,
    orlcomp::HandleValue handle, std::uint32_t field,
    const double value[4], std::string* error = nullptr);
bool read_locator_matrix(const HandleViewContext& context,
    orlcomp::HandleValue handle, double value[16],
    std::string* error = nullptr);
bool write_locator_matrix(const HandleViewContext& context,
    orlcomp::HandleValue handle, const double value[16],
    std::string* error = nullptr);
bool read_world_matrix(const HandleViewContext& context,
    orlcomp::HandleValue handle, double value[16],
    std::string* error = nullptr);
bool write_world_matrix(const HandleViewContext& context,
    orlcomp::HandleValue handle, const double value[16],
    std::string* error = nullptr);

extern "C" {
std::int64_t __orlrig_joint_read_i64(
    std::uint64_t type_id, std::int64_t slot, std::int64_t field);
void __orlrig_joint_write_i64(
    std::uint64_t type_id, std::int64_t slot, std::int64_t field,
    std::int64_t value);
void __orlrig_joint_read_vec4(
    double* value, std::uint64_t type_id, std::int64_t slot,
    std::int64_t field);
void __orlrig_joint_write_vec4(
    const double* value, std::uint64_t type_id, std::int64_t slot,
    std::int64_t field);
void __orlrig_locator_read_matrix(
    double* value, std::uint64_t type_id, std::int64_t slot,
    std::int64_t field);
void __orlrig_locator_write_matrix(
    const double* value, std::uint64_t type_id, std::int64_t slot,
    std::int64_t field);
void __orlrig_world_read_matrix(
    double* value, std::uint64_t type_id, std::int64_t slot,
    std::int64_t field);
void __orlrig_world_write_matrix(
    const double* value, std::uint64_t type_id, std::int64_t slot,
    std::int64_t field);
}

} // namespace orlrig
