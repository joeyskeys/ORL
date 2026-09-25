#include "handle_registry.hpp"

#include "joint.hpp"
#include "locator.hpp"

#include <algorithm>
#include <cstring>
#include <vector>

namespace orlrig
{

namespace
{

thread_local HandleViewContext* active_context = nullptr;

void set_error(std::string* error, std::string message) {
    if (error != nullptr) {
        *error = std::move(message);
    }
}

bool valid_handle(
    const HandleViewContext& context,
    orlcomp::HandleValue handle,
    std::uint64_t expected_type,
    const HandleStorage& storage,
    std::size_t stride,
    std::string* error)
{
    if (!orlcomp::IsValidHandleValue(handle)) {
        set_error(error, "Handle token is invalid");
        return false;
    }
    if (handle.type_id != expected_type) {
        set_error(error, "Handle token has the wrong nominal type");
        return false;
    }
    if (storage.data == nullptr || storage.count == 0
        || storage.stride < stride
        || handle.slot < 0
        || static_cast<std::size_t>(handle.slot) >= storage.count)
    {
        set_error(error, "Handle slot is outside the bound storage");
        return false;
    }
    if (!context.topology_revision) {
        set_error(error, "Handle context has no topology revision");
        return false;
    }
    return true;
}

Joint* joint_at(const HandleStorage& storage, std::int64_t slot) {
    return reinterpret_cast<Joint*>(
        static_cast<std::byte*>(storage.data)
        + static_cast<std::size_t>(slot) * storage.stride);
}

const double* locator_at_const(
    const HandleStorage& storage, std::int64_t slot) {
    return reinterpret_cast<const double*>(
        static_cast<const std::byte*>(storage.data)
        + static_cast<std::size_t>(slot) * storage.stride);
}

double* locator_at(const HandleStorage& storage, std::int64_t slot) {
    return reinterpret_cast<double*>(
        static_cast<std::byte*>(storage.data)
        + static_cast<std::size_t>(slot) * storage.stride);
}

bool writable(const HandleStorage& storage, std::string* error) {
    if (!storage.writable) {
        set_error(error, "Handle view storage is read-only");
        return false;
    }
    return true;
}

} // namespace

void set_active_handle_context(HandleViewContext* context) {
    active_context = context;
}

ScopedHandleViewContext::ScopedHandleViewContext(
    HandleViewContext* context)
    : previous_(active_context)
{
    active_context = context;
}

ScopedHandleViewContext::~ScopedHandleViewContext() {
    active_context = previous_;
}

bool register_rig_handle_views(
    orlcomp::HandleViewRegistry& registry, std::string* error)
{
    const auto ensure = [&registry, error](
        orlcomp::HandleViewDescriptor descriptor) {
        if (registry.find(
                descriptor.source_handle, descriptor.destination_struct)
            != nullptr)
        {
            return true;
        }
        return registry.register_view(std::move(descriptor), error);
    };

    if (!ensure({
            std::string{kJointHandleCanonical}, "Joint", 1, 3,
            "joint_arena",
            {
                {"parent", orlcomp::HandleViewFieldKind::Int64, 0,
                    "__orlrig_joint_read_i64",
                    "__orlrig_joint_write_i64", "int", 0, 8},
                {"selected", orlcomp::HandleViewFieldKind::Int64, 1,
                    "__orlrig_joint_read_i64",
                    "__orlrig_joint_write_i64", "int", 8, 8},
                {"pad0", orlcomp::HandleViewFieldKind::Int64, 2,
                    "__orlrig_joint_read_i64",
                    "__orlrig_joint_write_i64", "int", 16, 8},
                {"pad1", orlcomp::HandleViewFieldKind::Int64, 3,
                    "__orlrig_joint_read_i64",
                    "__orlrig_joint_write_i64", "int", 24, 8},
                {"translation", orlcomp::HandleViewFieldKind::Vec4, 4,
                    "__orlrig_joint_read_vec4",
                    "__orlrig_joint_write_vec4", "vec4", 32, 32},
                {"rotation", orlcomp::HandleViewFieldKind::Vec4, 5,
                    "__orlrig_joint_read_vec4",
                    "__orlrig_joint_write_vec4", "quat", 64, 32},
                {"scale", orlcomp::HandleViewFieldKind::Vec4, 6,
                    "__orlrig_joint_read_vec4",
                    "__orlrig_joint_write_vec4", "vec4", 96, 32},
            }}))
    {
        return false;
    }
    if (!ensure({
            std::string{kLocatorHandleCanonical}, "Locator", 1, 3,
            "locator_arena",
            {{
                "xform", orlcomp::HandleViewFieldKind::Matrix, 0,
                "__orlrig_locator_read_matrix",
                "__orlrig_locator_write_matrix",
                "matrix", 0, 128,
            }}}))
    {
        return false;
    }
    return ensure({
        std::string{kJointHandleCanonical},
        std::string{kWorldTransformStruct}, 1, 3,
        "joint_arena",
        {{
            "xform", orlcomp::HandleViewFieldKind::Matrix, 0,
            "__orlrig_world_read_matrix",
            "__orlrig_world_write_matrix",
            "matrix", 0, 128,
        }}});
}

bool validate_handle_context(
    const HandleViewContext& context,
    std::uint64_t expected_revision,
    std::string* error)
{
    if (context.topology_revision == 0
        || context.topology_revision != expected_revision)
    {
        set_error(error, "Handle context topology revision is stale");
        return false;
    }
    return true;
}

bool read_joint_int(const HandleViewContext& context,
    orlcomp::HandleValue handle, std::uint32_t field,
    std::int64_t* value, std::string* error)
{
    if (value == nullptr
        || !valid_handle(context, handle,
            orlcomp::HandleTypeIdFor(kJointHandleCanonical),
            context.joints, sizeof(Joint), error))
    {
        return false;
    }
    const auto* joint = joint_at(context.joints, handle.slot);
    switch (field) {
    case 0: *value = joint->parent; return true;
    case 1: *value = joint->selected; return true;
    case 2: *value = joint->pad0; return true;
    case 3: *value = joint->pad1; return true;
    default:
        set_error(error, "Unknown Joint integer field");
        return false;
    }
}

bool write_joint_int(const HandleViewContext& context,
    orlcomp::HandleValue handle, std::uint32_t field,
    std::int64_t value, std::string* error)
{
    if (!valid_handle(context, handle,
            orlcomp::HandleTypeIdFor(kJointHandleCanonical),
            context.joints, sizeof(Joint), error)
        || !writable(context.joints, error))
    {
        return false;
    }
    auto* joint = joint_at(context.joints, handle.slot);
    switch (field) {
    case 0: joint->parent = value; return true;
    case 1: joint->selected = value; return true;
    case 2: joint->pad0 = value; return true;
    case 3: joint->pad1 = value; return true;
    default:
        set_error(error, "Unknown Joint integer field");
        return false;
    }
}

bool read_joint_vec4(const HandleViewContext& context,
    orlcomp::HandleValue handle, std::uint32_t field,
    double value[4], std::string* error)
{
    if (value == nullptr
        || !valid_handle(context, handle,
            orlcomp::HandleTypeIdFor(kJointHandleCanonical),
            context.joints, sizeof(Joint), error))
    {
        return false;
    }
    const auto* joint = joint_at(context.joints, handle.slot);
    const double* source = field == 4 ? joint->translation
        : field == 5 ? joint->rotation
        : field == 6 ? joint->scale : nullptr;
    if (source == nullptr) {
        set_error(error, "Unknown Joint vector field");
        return false;
    }
    std::copy(source, source + 4, value);
    return true;
}

bool write_joint_vec4(const HandleViewContext& context,
    orlcomp::HandleValue handle, std::uint32_t field,
    const double value[4], std::string* error)
{
    if (value == nullptr
        || !valid_handle(context, handle,
            orlcomp::HandleTypeIdFor(kJointHandleCanonical),
            context.joints, sizeof(Joint), error)
        || !writable(context.joints, error))
    {
        return false;
    }
    auto* joint = joint_at(context.joints, handle.slot);
    double* destination = field == 4 ? joint->translation
        : field == 5 ? joint->rotation
        : field == 6 ? joint->scale : nullptr;
    if (destination == nullptr) {
        set_error(error, "Unknown Joint vector field");
        return false;
    }
    std::copy(value, value + 4, destination);
    return true;
}

bool read_locator_matrix(const HandleViewContext& context,
    orlcomp::HandleValue handle, double value[16],
    std::string* error)
{
    if (value == nullptr
        || !valid_handle(context, handle,
            orlcomp::HandleTypeIdFor(kLocatorHandleCanonical),
            context.locators, kLocatorStride, error))
    {
        return false;
    }
    std::copy_n(locator_at_const(context.locators, handle.slot), 16, value);
    return true;
}

bool write_locator_matrix(const HandleViewContext& context,
    orlcomp::HandleValue handle, const double value[16],
    std::string* error)
{
    if (value == nullptr
        || !valid_handle(context, handle,
            orlcomp::HandleTypeIdFor(kLocatorHandleCanonical),
            context.locators, kLocatorStride, error)
        || !writable(context.locators, error))
    {
        return false;
    }
    std::copy_n(value, 16, locator_at(context.locators, handle.slot));
    return true;
}

bool read_world_matrix(const HandleViewContext& context,
    orlcomp::HandleValue handle, double value[16],
    std::string* error)
{
    if (value == nullptr
        || !valid_handle(context, handle,
            orlcomp::HandleTypeIdFor(kJointHandleCanonical),
            context.joints, sizeof(Joint), error))
    {
        return false;
    }
    std::vector<Joint> joints;
    joints.resize(context.joints.count);
    for (std::size_t index = 0; index < joints.size(); ++index) {
        joints[index] = *joint_at(context.joints,
            static_cast<std::int64_t>(index));
    }
    const auto world = joint_world_matrix(joints, handle.slot);
    for (int row = 0; row < 4; ++row) {
        for (int column = 0; column < 4; ++column) {
            value[row * 4 + column] = world[column][row];
        }
    }
    return true;
}

bool write_world_matrix(const HandleViewContext& context,
    orlcomp::HandleValue handle, const double value[16],
    std::string* error)
{
    if (value == nullptr
        || !valid_handle(context, handle,
            orlcomp::HandleTypeIdFor(kJointHandleCanonical),
            context.joints, sizeof(Joint), error)
        || !writable(context.joints, error))
    {
        return false;
    }
    std::vector<Joint> joints;
    joints.resize(context.joints.count);
    for (std::size_t index = 0; index < joints.size(); ++index) {
        joints[index] = *joint_at(context.joints,
            static_cast<std::int64_t>(index));
    }
    glm::mat4 world{1.0f};
    for (int row = 0; row < 4; ++row) {
        for (int column = 0; column < 4; ++column) {
            world[column][row] = static_cast<float>(
                value[row * 4 + column]);
        }
    }
    auto* destination = joint_at(context.joints, handle.slot);
    return write_joint_world_matrix(
        joints, handle.slot, world, *destination);
}

extern "C" std::int64_t __orlrig_joint_read_i64(
    std::uint64_t type_id, std::int64_t slot, std::int64_t field)
{
    if (active_context == nullptr) {
        return 0;
    }
    std::int64_t value = 0;
    read_joint_int(*active_context, {type_id, slot},
        static_cast<std::uint32_t>(field), &value);
    return value;
}

extern "C" void __orlrig_joint_write_i64(
    std::uint64_t type_id, std::int64_t slot, std::int64_t field,
    std::int64_t value)
{
    if (active_context != nullptr) {
        write_joint_int(*active_context, {type_id, slot},
            static_cast<std::uint32_t>(field), value);
    }
}

extern "C" void __orlrig_joint_read_vec4(
    double* value, std::uint64_t type_id, std::int64_t slot,
    std::int64_t field)
{
    if (active_context != nullptr && value != nullptr) {
        read_joint_vec4(*active_context, {type_id, slot},
            static_cast<std::uint32_t>(field), value);
    }
}

extern "C" void __orlrig_joint_write_vec4(
    const double* value, std::uint64_t type_id, std::int64_t slot,
    std::int64_t field)
{
    if (active_context != nullptr && value != nullptr) {
        write_joint_vec4(*active_context, {type_id, slot},
            static_cast<std::uint32_t>(field), value);
    }
}

extern "C" void __orlrig_locator_read_matrix(
    double* value, std::uint64_t type_id, std::int64_t slot,
    std::int64_t)
{
    if (active_context != nullptr && value != nullptr) {
        read_locator_matrix(
            *active_context, {type_id, slot}, value);
    }
}

extern "C" void __orlrig_locator_write_matrix(
    const double* value, std::uint64_t type_id, std::int64_t slot,
    std::int64_t)
{
    if (active_context != nullptr && value != nullptr) {
        write_locator_matrix(
            *active_context, {type_id, slot}, value);
    }
}

extern "C" void __orlrig_world_read_matrix(
    double* value, std::uint64_t type_id, std::int64_t slot,
    std::int64_t)
{
    if (active_context != nullptr && value != nullptr) {
        read_world_matrix(
            *active_context, {type_id, slot}, value);
    }
}

extern "C" void __orlrig_world_write_matrix(
    const double* value, std::uint64_t type_id, std::int64_t slot,
    std::int64_t)
{
    if (active_context != nullptr && value != nullptr) {
        write_world_matrix(
            *active_context, {type_id, slot}, value);
    }
}

} // namespace orlrig
