#pragma once

#include "graph_ir.hpp"
#include "graph_validation.hpp"

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace orlgraph
{

inline constexpr const char* kOroMagic = "ORO";
inline constexpr std::uint32_t kOroFormatVersion = 1;
inline constexpr const char* kOroLanguageVersion = "orl-0";
inline constexpr const char* kOroLogicalAbiVersion = "orlgraph-0";

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

OroSerializationResult serialize_oro(const GraphModule& module,
    const NodeRegistry& registry);

OroDocument deserialize_oro(std::string_view text);

bool save_oro(const std::string& path, const GraphModule& module,
    const NodeRegistry& registry, std::vector<Diagnostic>* diagnostics = nullptr);

OroDocument load_oro(const std::string& path);

std::string oro_content_hash(std::string_view canonical_text);

} // namespace orlgraph
