#pragma once

#include "graph_ir.hpp"
#include "graph_validation.hpp"

#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

namespace orlgraph
{

inline constexpr const char* kOroMagic = "ORO";
inline constexpr std::uint32_t kOroFormatVersion = 1;
inline constexpr const char* kOroLanguageVersion = "orl-0";
inline constexpr const char* kOroLogicalAbiVersion = "orlgraph-1";

// Graph JSON is the editable graph format. It stores one GraphModule and
// references node definitions by stable ID; the definitions remain owned by
// the caller's NodeRegistry.
inline constexpr const char* kGraphJsonMagic = "ORL_GRAPH";
inline constexpr std::uint32_t kGraphJsonFormatVersion = 1;
inline constexpr const char* kGraphStagesJsonMagic = "ORL_GRAPH_STAGES";
inline constexpr std::uint32_t kGraphStagesJsonFormatVersion = 1;

struct OroSerializationResult {
    bool ok = false;
    std::string text;
    std::string content_hash;
    std::vector<Diagnostic> diagnostics;
};

struct OroDocument {
    bool ok = false;
    GraphModule module;
    NodeRegistry registry;
    std::string content_hash;
    std::vector<Diagnostic> diagnostics;
};

struct GraphJsonSerializationResult {
    bool ok = false;
    std::string text;
    std::string content_hash;
    std::vector<Diagnostic> diagnostics;
};

struct GraphJsonDocument {
    bool ok = false;
    GraphModule module;
    std::string content_hash;
    std::vector<Diagnostic> diagnostics;
};

struct GraphStagesJsonSerializationResult {
    bool ok = false;
    std::string text;
    std::string content_hash;
    std::vector<Diagnostic> diagnostics;
};

struct GraphStagesJsonDocument {
    bool ok = false;
    GraphModule solver;
    GraphModule deformer;
    std::string content_hash;
    std::vector<Diagnostic> diagnostics;
};

OroSerializationResult serialize_oro(const GraphModule& module,
    const NodeRegistry& registry);

OroDocument deserialize_oro(std::string_view text);

bool save_oro(const std::string& path, const GraphModule& module,
    const NodeRegistry& registry, std::vector<Diagnostic>* diagnostics = nullptr);

OroDocument load_oro(const std::string& path);

// Content-addressed cache helpers for complete graph IR documents. The cache
// uses the existing versioned .oro representation and therefore preserves the
// graph, registry definitions, ABI metadata, and content hash together.
// LLVM IR is not stored here: a cache miss still runs oro -> LLVM IR ->
// machine code through the ordinary compile path.
struct GraphIrCacheOptions {
    std::filesystem::path directory;
    bool force_recompile = false;
};

class OrlGraphIrCache final {
public:
    explicit OrlGraphIrCache(GraphIrCacheOptions options = {});

    bool enabled() const;
    bool force_recompile() const {
        return options_.force_recompile;
    }

    std::filesystem::path path(std::string_view content_hash) const;
    bool save(const GraphModule& module, const NodeRegistry& registry,
        std::string* content_hash = nullptr,
        std::vector<Diagnostic>* diagnostics = nullptr) const;
    OroDocument load(std::string_view content_hash) const;

private:
    GraphIrCacheOptions options_;
};

std::filesystem::path graph_ir_cache_path(
    const std::filesystem::path& directory,
    std::string_view content_hash);
bool save_graph_ir_cache(const std::filesystem::path& directory,
    const GraphModule& module, const NodeRegistry& registry,
    std::string* content_hash = nullptr,
    std::vector<Diagnostic>* diagnostics = nullptr);
OroDocument load_graph_ir_cache(
    const std::filesystem::path& directory,
    std::string_view content_hash);
bool save_graph_ir_cache(const GraphIrCacheOptions& options,
    const GraphModule& module, const NodeRegistry& registry,
    std::string* content_hash = nullptr,
    std::vector<Diagnostic>* diagnostics = nullptr);
OroDocument load_graph_ir_cache(const GraphIrCacheOptions& options,
    std::string_view content_hash);

std::string oro_content_hash(std::string_view canonical_text);

GraphJsonSerializationResult serialize_graph_json(const GraphModule& module);
GraphJsonDocument deserialize_graph_json(std::string_view text);
bool save_graph_json(const std::string& path, const GraphModule& module,
    std::vector<Diagnostic>* diagnostics = nullptr);
GraphJsonDocument load_graph_json(const std::string& path);

GraphStagesJsonSerializationResult serialize_graph_stages_json(
    const GraphModule& solver, const GraphModule& deformer);
GraphStagesJsonDocument deserialize_graph_stages_json(std::string_view text);
bool save_graph_stages_json(const std::string& path,
    const GraphModule& solver, const GraphModule& deformer,
    std::vector<Diagnostic>* diagnostics = nullptr);
GraphStagesJsonDocument load_graph_stages_json(const std::string& path);

} // namespace orlgraph
