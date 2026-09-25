#pragma once

#include "orl_runtime_signature.h"

#include <cstdint>
#include <map>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

namespace orlcomp
{

enum class HandleViewFieldKind : std::uint8_t {
    Int64,
    Float64,
    Vec4,
    Matrix,
};

struct HandleViewFieldDescriptor {
    std::string name;
    HandleViewFieldKind kind = HandleViewFieldKind::Float64;
    std::uint32_t field_id = 0;
    std::string read_symbol;
    std::string write_symbol;
    std::string type_name;
    std::uint32_t byte_offset = 0;
    std::uint32_t byte_size = 0;
};

struct HandleViewDescriptor {
    std::string source_handle;
    std::string destination_struct;
    std::uint32_t abi_version = 1;
    std::uint32_t backend_mask = 1u;
    std::string storage_identity;
    std::vector<HandleViewFieldDescriptor> fields;
};

class HandleViewRegistry {
public:
    bool register_view(HandleViewDescriptor descriptor,
        std::string* error = nullptr)
    {
        if (descriptor.source_handle.empty()
            || descriptor.destination_struct.empty()
            || descriptor.storage_identity.empty()
            || descriptor.backend_mask == 0)
        {
            set_error(error, "Handle view identity is incomplete");
            return false;
        }
        const auto type_id =
            HandleTypeIdFor(descriptor.source_handle);
        if (const auto found = type_names_.find(type_id);
            found != type_names_.end()
            && found->second != descriptor.source_handle)
        {
            set_error(error, "Handle view type ID collision between '"
                + found->second + "' and '"
                + descriptor.source_handle + "'");
            return false;
        }
        const auto key = descriptor.source_handle + "->"
            + descriptor.destination_struct;
        if (views_.contains(key)) {
            set_error(error, "Duplicate handle view: " + key);
            return false;
        }
        std::unordered_map<std::string, bool> fields;
        for (const auto& field : descriptor.fields) {
            if (field.name.empty() || field.read_symbol.empty()
                || field.write_symbol.empty()
                || !fields.emplace(field.name, true).second)
            {
                set_error(error,
                    "Handle view has incomplete or duplicate field: " + key);
                return false;
            }
        }
        type_names_[type_id] = descriptor.source_handle;
        views_.emplace(key, std::move(descriptor));
        return true;
    }

    const HandleViewDescriptor* find(
        std::string_view source_handle,
        std::string_view destination_struct) const
    {
        const auto found = views_.find(
            std::string{source_handle} + "->"
                + std::string{destination_struct});
        return found == views_.end() ? nullptr : &found->second;
    }

    bool has_destination(std::string_view destination_struct) const {
        for (const auto& [_, view] : views_) {
            if (view.destination_struct == destination_struct) {
                return true;
            }
        }
        return false;
    }

    const HandleViewFieldDescriptor* field(
        const HandleViewDescriptor& view, std::string_view name) const
    {
        for (const auto& candidate : view.fields) {
            if (candidate.name == name) {
                return &candidate;
            }
        }
        return nullptr;
    }

    void clear() {
        views_.clear();
        type_names_.clear();
    }

    std::string fingerprint() const {
        std::string result = "orl-handle-views-v1";
        for (const auto& [key, descriptor] : views_) {
            result += "|" + key + ":" + descriptor.storage_identity
                + ":" + std::to_string(descriptor.abi_version)
                + ":" + std::to_string(descriptor.backend_mask);
            for (const auto& field : descriptor.fields) {
                result += "|" + field.name + ":"
                    + std::to_string(field.field_id) + ":"
                    + field.read_symbol + ":" + field.write_symbol;
            }
        }
        return result;
    }

private:
    static void set_error(std::string* error, std::string message) {
        if (error != nullptr) {
            *error = std::move(message);
        }
    }

    std::unordered_map<std::string, HandleViewDescriptor> views_;
    std::map<std::uint64_t, std::string> type_names_;
};

inline HandleViewRegistry& global_handle_view_registry() {
    static HandleViewRegistry registry;
    return registry;
}

} // namespace orlcomp
