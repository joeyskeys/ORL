#include "graph_ids.hpp"

#include <cstdint>
#include <iomanip>
#include <sstream>
#include <utility>

namespace orlgraph
{

StableId StableId::from(std::string_view domain, std::string_view name) {
    // FNV-1a gives deterministic IDs without depending on a platform hash
    // implementation. Keep the readable seed in the serialized identity.
    std::uint64_t hash = 14695981039346656037ull;
    for (const char value : domain) {
        hash ^= static_cast<unsigned char>(value);
        hash *= 1099511628211ull;
    }
    hash ^= 0xffu;
    hash *= 1099511628211ull;
    for (const char value : name) {
        hash ^= static_cast<unsigned char>(value);
        hash *= 1099511628211ull;
    }

    std::ostringstream stream;
    stream << domain << ":" << name << ":"
           << std::hex << std::setw(16) << std::setfill('0') << hash;
    return StableId{stream.str()};
}

} // namespace orlgraph
