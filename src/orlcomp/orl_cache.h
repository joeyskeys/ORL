#pragma once

#include <cstdint>
#include <filesystem>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace orlcomp
{

enum class OrlBinaryKind : std::uint32_t {
    CpuObject = 1,
    CudaPtx = 2,
    CudaCubin = 3,
    RocmObject = 4,
};

struct OrlBinaryCacheOptions {
    std::filesystem::path directory;
    bool force_recompile = false;
};

// Content-addressed, versioned storage for backend machine-code artifacts.
// LLVM IR is not cached: callers compile oro -> LLVM IR -> machine code only
// when load() misses or force_recompile is set. The key must include every
// compiler, backend, target, and ABI input that affects the artifact.
class OrlBinaryCache final {
public:
    explicit OrlBinaryCache(OrlBinaryCacheOptions options = {});

    bool enabled() const;
    bool force_recompile() const {
        return options_.force_recompile;
    }

    std::filesystem::path path(
        OrlBinaryKind kind, std::string_view key) const;
    bool load(OrlBinaryKind kind, std::string_view key,
        std::vector<std::uint8_t>& data) const;
    bool save(OrlBinaryKind kind, std::string_view key,
        std::span<const std::uint8_t> data) const;

private:
    OrlBinaryCacheOptions options_;
};

// Produces the stable filename key used by OrlBinaryCache. This is not a
// cryptographic identity; the cache file also stores and validates the full
// material key before accepting a hit.
std::string make_binary_cache_key(std::string_view material);

bool save_binary_cache(const OrlBinaryCacheOptions& options,
    OrlBinaryKind kind, std::string_view key,
    std::span<const std::uint8_t> data);
bool load_binary_cache(const OrlBinaryCacheOptions& options,
    OrlBinaryKind kind, std::string_view key,
    std::vector<std::uint8_t>& data);

} // namespace orlcomp
