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
    // Optional device-resident storage for the same logical arena. When
    // present, CUDA executions must consume this pointer instead of creating
    // a second typed-handle arena from the host mirror.
    std::uint64_t device_ptr = 0;
    std::size_t device_bytes = 0;
};

struct HandleViewContext {
    HandleStorage joints;
    HandleStorage locators;
    std::uint64_t topology_revision = 0;
    // Host-side content generations let a CUDA execution avoid re-uploading
    // an arena whose device state was just produced by a fused solver. A
    // zero generation means the caller cannot prove that the host storage is
    // unchanged, so the execution uploads it conservatively.
    std::uint64_t joints_revision = 0;
    std::uint64_t locators_revision = 0;
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
std::int64_t __orlrig_handle_valid(
    std::uint64_t type_id, std::int64_t slot);
void __orlrig_joint_parent(
    std::uint64_t type_id, std::int64_t slot,
    std::uint64_t* parent_type_id, std::int64_t* parent_slot);
std::int64_t __orlrig_joint_is_ancestor(
    std::uint64_t root_type_id, std::int64_t root_slot,
    std::uint64_t end_type_id, std::int64_t end_slot);
}

} // namespace orlrig
