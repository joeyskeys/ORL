#include "orl_cache.h"

#include <array>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <system_error>
#include <utility>

namespace orlcomp
{
namespace
{

constexpr std::array<char, 8> kCacheMagic{
    'O', 'R', 'L', 'B', 'I', 'N', '1', '\0'};
constexpr std::uint32_t kCacheVersion = 1;
constexpr std::uint64_t kMaxCachePayload =
    static_cast<std::uint64_t>(1) << 34;

std::string kind_directory(OrlBinaryKind kind) {
    switch (kind) {
    case OrlBinaryKind::CpuObject:
        return "cpu";
    case OrlBinaryKind::CudaPtx:
        return "cuda_ptx";
    case OrlBinaryKind::CudaCubin:
        return "cuda_cubin";
    case OrlBinaryKind::RocmObject:
        return "rocm";
    }
    return "unknown";
}

std::string kind_extension(OrlBinaryKind kind) {
    switch (kind) {
    case OrlBinaryKind::CpuObject:
        return ".obj";
    case OrlBinaryKind::CudaPtx:
        return ".ptx";
    case OrlBinaryKind::CudaCubin:
        return ".cubin";
    case OrlBinaryKind::RocmObject:
        return ".hsaco";
    }
    return ".bin";
}

template <typename T>
bool write_value(std::ofstream& output, T value) {
    output.write(
        reinterpret_cast<const char*>(&value),
        static_cast<std::streamsize>(sizeof(value)));
    return static_cast<bool>(output);
}

template <typename T>
bool read_value(std::ifstream& input, T& value) {
    input.read(
        reinterpret_cast<char*>(&value),
        static_cast<std::streamsize>(sizeof(value)));
    return static_cast<bool>(input);
}

std::string path_key(std::string_view key) {
    return make_binary_cache_key(key);
}

} // namespace

OrlBinaryCache::OrlBinaryCache(OrlBinaryCacheOptions options)
    : options_(std::move(options))
{
}

bool OrlBinaryCache::enabled() const {
    return !options_.directory.empty();
}

std::filesystem::path OrlBinaryCache::path(
    OrlBinaryKind kind, std::string_view key) const
{
    return options_.directory / kind_directory(kind)
        / (path_key(key) + kind_extension(kind));
}

bool OrlBinaryCache::load(OrlBinaryKind kind, std::string_view key,
    std::vector<std::uint8_t>& data) const
{
    data.clear();
    if (!enabled() || force_recompile()) {
        return false;
    }

    std::ifstream input(path(kind, key), std::ios::binary);
    if (!input) {
        return false;
    }

    std::array<char, kCacheMagic.size()> magic{};
    input.read(magic.data(),
        static_cast<std::streamsize>(magic.size()));
    if (!input || magic != kCacheMagic) {
        return false;
    }

    std::uint32_t version = 0;
    std::uint32_t stored_kind = 0;
    std::uint64_t key_size = 0;
    std::uint64_t payload_size = 0;
    if (!read_value(input, version)
        || !read_value(input, stored_kind)
        || !read_value(input, key_size)
        || !read_value(input, payload_size)
        || version != kCacheVersion
        || stored_kind != static_cast<std::uint32_t>(kind)
        || key_size != key.size()
        || payload_size > kMaxCachePayload)
    {
        return false;
    }

    std::string stored_key(static_cast<std::size_t>(key_size), '\0');
    input.read(
        stored_key.data(),
        static_cast<std::streamsize>(stored_key.size()));
    if (!input || stored_key != key) {
        return false;
    }

    if (payload_size
        > static_cast<std::uint64_t>(
            std::numeric_limits<std::size_t>::max()))
    {
        return false;
    }
    data.resize(static_cast<std::size_t>(payload_size));
    if (!data.empty()) {
        input.read(
            reinterpret_cast<char*>(data.data()),
            static_cast<std::streamsize>(data.size()));
    }
    return static_cast<bool>(input);
}

bool OrlBinaryCache::save(OrlBinaryKind kind, std::string_view key,
    std::span<const std::uint8_t> data) const
{
    if (!enabled()
        || data.size() > static_cast<std::size_t>(kMaxCachePayload))
    {
        return false;
    }

    const auto destination = path(kind, key);
    std::error_code error;
    std::filesystem::create_directories(
        destination.parent_path(), error);
    if (error) {
        return false;
    }

    const auto temporary = destination.string() + ".tmp";
    {
        std::ofstream output(temporary, std::ios::binary | std::ios::trunc);
        if (!output) {
            return false;
        }
        output.write(
            kCacheMagic.data(),
            static_cast<std::streamsize>(kCacheMagic.size()));
        if (!write_value(output, kCacheVersion)
            || !write_value(
                output, static_cast<std::uint32_t>(kind))
            || !write_value(
                output, static_cast<std::uint64_t>(key.size()))
            || !write_value(
                output, static_cast<std::uint64_t>(data.size())))
        {
            return false;
        }
        output.write(
            key.data(),
            static_cast<std::streamsize>(key.size()));
        if (!data.empty()) {
            output.write(
                reinterpret_cast<const char*>(data.data()),
                static_cast<std::streamsize>(data.size()));
        }
        if (!output) {
            return false;
        }
    }

    std::filesystem::remove(destination, error);
    error.clear();
    std::filesystem::rename(temporary, destination, error);
    if (error) {
        std::filesystem::remove(temporary, error);
        return false;
    }
    return true;
}

std::string make_binary_cache_key(std::string_view material) {
    // FNV-1a is sufficient for a filename key because the complete material
    // is stored in the artifact header and checked on load.
    std::uint64_t hash = 14695981039346656037ull;
    for (const unsigned char byte : material) {
        hash ^= byte;
        hash *= 1099511628211ull;
    }
    std::ostringstream result;
    result << std::hex << std::setfill('0') << std::setw(16) << hash;
    return result.str();
}

bool save_binary_cache(const OrlBinaryCacheOptions& options,
    OrlBinaryKind kind, std::string_view key,
    std::span<const std::uint8_t> data)
{
    return OrlBinaryCache(options).save(kind, key, data);
}

bool load_binary_cache(const OrlBinaryCacheOptions& options,
    OrlBinaryKind kind, std::string_view key,
    std::vector<std::uint8_t>& data)
{
    return OrlBinaryCache(options).load(kind, key, data);
}

} // namespace orlcomp
