#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <string_view>
#include <utility>

namespace orlgraph
{

// StableId is serialized graph identity. It is deliberately independent of
// process-local handles such as orlrig::ComponentId.
struct StableId {
    std::string value;

    StableId() = default;
    explicit StableId(std::string text) : value(std::move(text)) {}

    static StableId from(std::string_view domain, std::string_view name);

    bool empty() const { return value.empty(); }
    explicit operator bool() const { return !empty(); }

    friend bool operator==(const StableId&, const StableId&) = default;
    friend bool operator<(const StableId& left, const StableId& right) {
        return left.value < right.value;
    }
};

struct StableIdHash {
    std::size_t operator()(const StableId& id) const noexcept {
        return std::hash<std::string>{}(id.value);
    }
};

struct Version {
    std::uint32_t major = 1;
    std::uint32_t minor = 0;
    std::uint32_t patch = 0;

    friend bool operator==(const Version&, const Version&) = default;
    friend bool operator<(const Version& left, const Version& right) {
        if (left.major != right.major) {
            return left.major < right.major;
        }
        if (left.minor != right.minor) {
            return left.minor < right.minor;
        }
        return left.patch < right.patch;
    }
};

} // namespace orlgraph
