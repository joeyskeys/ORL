#pragma once

#include "orl_ast.h"

#include <cstddef>
#include <functional>
#include <optional>
#include <cstdint>
#include <string>
#include <string_view>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

namespace orlcomp
{

inline constexpr std::string_view kSolverContextParameterName =
    "__orl_solver_context";
inline constexpr std::string_view kSolverContextTypeName = "SolverContext";
inline constexpr std::string_view kHierarchyContextParameterName =
    "__orl_hierarchy_context";
inline constexpr std::string_view kHierarchyContextTypeName =
    "HierarchyContext";
inline constexpr std::string_view kHierarchyDataParameterName =
    "__orl_hierarchy_data";
inline constexpr std::string_view kHierarchyDataTypeName = "int";
inline constexpr std::string_view kHandleAbiVersion = "orl-handle-v1";
inline constexpr std::string_view kHandleContextParameterName =
    "__orl_handle_context";

struct HandleValue {
    std::uint64_t type_id = 0;
    std::int64_t slot = -1;

    friend bool operator==(const HandleValue&, const HandleValue&) = default;
};

static_assert(std::is_standard_layout_v<HandleValue>);
static_assert(sizeof(HandleValue) == sizeof(std::uint64_t) * 2);
static_assert(alignof(HandleValue) == alignof(std::uint64_t));
static_assert(offsetof(HandleValue, type_id) == 0);
static_assert(offsetof(HandleValue, slot) == sizeof(std::uint64_t));
static_assert(sizeof(HandleValue::type_id) == sizeof(std::uint64_t));
static_assert(sizeof(HandleValue::slot) == sizeof(std::int64_t));

// GPU-visible handle context. The arena fields are device addresses encoded
// as 64-bit lanes; no host pointer or C++ object is embedded here.
struct HandleDeviceContext {
    std::uint64_t joint_arena = 0;
    std::uint64_t locator_arena = 0;
    std::uint64_t joint_count = 0;
    std::uint64_t locator_count = 0;
    std::uint64_t joint_stride = 0;
    std::uint64_t locator_stride = 0;
    std::uint64_t topology_revision = 0;
    std::uint64_t flags = 0;
};

static_assert(std::is_standard_layout_v<HandleDeviceContext>);
static_assert(sizeof(HandleDeviceContext) == sizeof(std::uint64_t) * 8);
static_assert(offsetof(HandleDeviceContext, joint_arena) == 0);
static_assert(offsetof(HandleDeviceContext, locator_arena)
    == sizeof(std::uint64_t));
static_assert(offsetof(HandleDeviceContext, topology_revision)
    == sizeof(std::uint64_t) * 6);

inline constexpr std::uint64_t kInvalidHandleTypeId = 0;
inline constexpr std::int64_t kInvalidHandleSlot = -1;

inline bool IsValidHandleValue(const HandleValue& value) {
    return value.type_id != kInvalidHandleTypeId && value.slot >= 0;
}

inline std::uint64_t HandleTypeIdFor(std::string_view canonical_name) {
    // FNV-1a over a versioned domain keeps the identity deterministic across
    // processes and toolchains while leaving room for a future hash revision.
    constexpr std::uint64_t offset = 14695981039346656037ull;
    constexpr std::uint64_t prime = 1099511628211ull;
    std::uint64_t hash = offset;
    for (const char character : kHandleAbiVersion) {
        hash ^= static_cast<unsigned char>(character);
        hash *= prime;
    }
    hash ^= static_cast<unsigned char>(':');
    hash *= prime;
    for (const char character : canonical_name) {
        hash ^= static_cast<unsigned char>(character);
        hash *= prime;
    }
    return hash == kInvalidHandleTypeId ? 1 : hash;
}

class HandleTypeRegistry {
public:
    bool register_type(std::string canonical_name,
        std::string* error = nullptr)
    {
        const auto id = HandleTypeIdFor(canonical_name);
        const auto found = ids_.find(id);
        if (found != ids_.end() && found->second != canonical_name) {
            if (error != nullptr) {
                *error = "Handle type ID collision between '"
                    + found->second + "' and '" + canonical_name + "'";
            }
            return false;
        }
        ids_.emplace(id, std::move(canonical_name));
        return true;
    }

    std::optional<std::uint64_t> id_for(
        std::string_view canonical_name) const
    {
        const auto id = HandleTypeIdFor(canonical_name);
        const auto found = ids_.find(id);
        if (found == ids_.end() || found->second != canonical_name) {
            return std::nullopt;
        }
        return id;
    }

private:
    std::unordered_map<std::uint64_t, std::string> ids_;
};

enum class OrlRuntimeParameterKind {
    Buffer,
    HandleBuffer,
    Int64,
    Float64,
    Handle,
    Unsupported,
};

struct OrlRuntimeParameter {
    std::string name;
    std::string type_name;
    OrlRuntimeParameterKind kind = OrlRuntimeParameterKind::Unsupported;
    std::string canonical_type_name;
    std::uint64_t handle_type_id = 0;
    bool hidden_scratch = false;
    std::string scratch_size_symbol;
    std::size_t scratch_size_multiplier = 1;
};

struct OrlRuntimeFunctionSignature {
    std::string name;
    std::string return_type;
    std::vector<OrlRuntimeParameter> parameters;
    bool uses_solver_context = false;
    bool uses_hierarchy_context = false;
};

inline OrlRuntimeParameterKind RuntimeParameterKindFor(const Parameter& parameter) {
    if (parameter.is_buffer) {
        return OrlRuntimeParameterKind::Buffer;
    }
    if (parameter.type_name == "int") {
        return OrlRuntimeParameterKind::Int64;
    }
    if (parameter.type_name == "float") {
        return OrlRuntimeParameterKind::Float64;
    }
    return OrlRuntimeParameterKind::Unsupported;
}

inline std::optional<OrlRuntimeFunctionSignature> DescribeRuntimeFunction(
    const Program& program, std::string_view name,
    const std::unordered_map<std::string, std::string>&
        handle_type_identities = {})
{
    for (const auto& item : program.items) {
        const auto* function = dynamic_cast<const FunctionDefinitionStatement*>(item.get());
        if (function == nullptr || function->name != name) {
            continue;
        }

        OrlRuntimeFunctionSignature signature;
        signature.name = function->name;
        signature.return_type = function->return_type;
        signature.uses_solver_context = program.uses_solver_context;
        signature.uses_hierarchy_context = program.uses_hierarchy_context;
        signature.parameters.reserve(function->parameters.size());
        std::unordered_map<std::string, std::string> handle_types;
        for (const auto& declaration : program.items) {
            const auto* handle =
                dynamic_cast<const HandleDefinitionStatement*>(
                    declaration.get());
            if (handle != nullptr) {
                handle_types.emplace(handle->name, handle->canonical_name);
            }
        }
        for (const Parameter& parameter : function->parameters) {
            OrlRuntimeParameter runtime{
                parameter.name,
                parameter.type_name,
                RuntimeParameterKindFor(parameter),
                {},
                0,
            };
            const auto found = handle_types.find(parameter.type_name);
            if (found != handle_types.end()) {
                runtime.kind = parameter.is_buffer
                    ? OrlRuntimeParameterKind::HandleBuffer
                    : OrlRuntimeParameterKind::Handle;
                runtime.canonical_type_name = found->second;
                if (const auto override = handle_type_identities.find(
                        parameter.type_name);
                    override != handle_type_identities.end())
                {
                    runtime.canonical_type_name = override->second;
                }
                runtime.handle_type_id = HandleTypeIdFor(found->second);
                runtime.handle_type_id = HandleTypeIdFor(
                    runtime.canonical_type_name);
            } else if (parameter.type_name == "handle") {
                const auto override = handle_type_identities.find("handle");
                if (override != handle_type_identities.end()
                    && override->second != "handle")
                {
                    runtime.kind = OrlRuntimeParameterKind::Handle;
                    runtime.canonical_type_name = override->second;
                    runtime.handle_type_id = HandleTypeIdFor(
                        runtime.canonical_type_name);
                }
            }
            signature.parameters.push_back(std::move(runtime));
        }
        std::vector<OrlRuntimeParameter> scratch_parameters;
        std::function<void(const Statement&)> collect_scratch;
        collect_scratch = [&](const Statement& statement) {
            if (const auto* declaration =
                    dynamic_cast<const DeclarationStatement*>(&statement))
            {
                if (declaration->array_size_expression == nullptr) {
                    return;
                }
                const auto bound = dispatch_array_bound(
                    *declaration->array_size_expression);
                if (!bound.has_value()) {
                    return;
                }
                if (bound->symbol == "joint_count") {
                    signature.uses_solver_context = true;
                }
                OrlRuntimeParameter scratch{
                    "__orl_scratch_" + declaration->variable_name,
                    declaration->type_name,
                    OrlRuntimeParameterKind::Buffer,
                    {},
                    0,
                    true,
                    bound->symbol,
                    bound->multiplier,
                };
                if (const auto found = handle_types.find(
                        declaration->type_name);
                    found != handle_types.end())
                {
                    scratch.canonical_type_name = found->second;
                    if (const auto override = handle_type_identities.find(
                            declaration->type_name);
                        override != handle_type_identities.end())
                    {
                        scratch.canonical_type_name = override->second;
                    }
                    scratch.handle_type_id = HandleTypeIdFor(
                        scratch.canonical_type_name);
                }
                scratch_parameters.push_back(std::move(scratch));
                return;
            }
            if (const auto* block =
                    dynamic_cast<const BlockStatement*>(&statement))
            {
                for (const auto& child : block->statements) {
                    if (child != nullptr) {
                        collect_scratch(*child);
                    }
                }
            } else if (const auto* conditional =
                    dynamic_cast<const IfStatement*>(&statement))
            {
                collect_scratch(*conditional->then_branch);
                if (conditional->else_branch != nullptr) {
                    collect_scratch(*conditional->else_branch);
                }
            } else if (const auto* loop =
                    dynamic_cast<const WhileStatement*>(&statement))
            {
                collect_scratch(*loop->body);
            } else if (const auto* loop =
                    dynamic_cast<const DoWhileStatement*>(&statement))
            {
                collect_scratch(*loop->body);
            } else if (const auto* loop =
                    dynamic_cast<const ForStatement*>(&statement))
            {
                collect_scratch(*loop->body);
            } else if (const auto* loop =
                    dynamic_cast<const ParallelForStatement*>(&statement))
            {
                collect_scratch(*loop->body);
            }
        };
        collect_scratch(*function->body);
        if (signature.uses_solver_context) {
            signature.parameters.push_back({
                std::string{kSolverContextParameterName},
                std::string{kSolverContextTypeName},
                OrlRuntimeParameterKind::Buffer,
            });
        }
        if (signature.uses_hierarchy_context) {
            signature.parameters.push_back({
                std::string{kHierarchyContextParameterName},
                std::string{kHierarchyContextTypeName},
                OrlRuntimeParameterKind::Buffer,
            });
            signature.parameters.push_back({
                std::string{kHierarchyDataParameterName},
                std::string{kHierarchyDataTypeName},
                OrlRuntimeParameterKind::Buffer,
            });
        }
        for (auto& scratch : scratch_parameters) {
            signature.parameters.push_back(std::move(scratch));
        }
        return signature;
    }
    return std::nullopt;
}

} // namespace orlcomp
