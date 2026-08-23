#include "orl_preprocessor.h"

#include <cctype>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string_view>

namespace orlcomp {

namespace {

std::string TrimLeft(const std::string &text) {
    std::size_t index = 0;
    while (index < text.size() && (text[index] == ' ' || text[index] == '\t')) {
        ++index;
    }
    return text.substr(index);
}

bool IsIdentifierChar(char c, bool first) {
    const unsigned char uc = static_cast<unsigned char>(c);
    return (std::isalpha(uc) != 0 || c == '_') || (!first && std::isdigit(uc) != 0);
}

bool ParseUseLine(const std::string &line, std::string *name) {
    const std::string trimmed = TrimLeft(line);
    constexpr std::string_view prefix = "use";
    if (trimmed.size() < prefix.size() || trimmed.compare(0, prefix.size(), prefix) != 0) {
        return false;
    }
    if (trimmed.size() > prefix.size() && IsIdentifierChar(trimmed[prefix.size()], false)) {
        return false;
    }

    std::size_t index = prefix.size();
    while (index < trimmed.size() && (trimmed[index] == ' ' || trimmed[index] == '\t')) {
        ++index;
    }
    if (index >= trimmed.size() || !IsIdentifierChar(trimmed[index], true)) {
        return false;
    }

    const std::size_t start = index;
    ++index;
    while (index < trimmed.size() && IsIdentifierChar(trimmed[index], false)) {
        ++index;
    }
    *name = trimmed.substr(start, index - start);

    while (index < trimmed.size() && (trimmed[index] == ' ' || trimmed[index] == '\t')) {
        ++index;
    }
    return index < trimmed.size() && trimmed[index] == ';';
}

bool LooksLikeUse(const std::string &line) {
    const std::string trimmed = TrimLeft(line);
    constexpr std::string_view prefix = "use";
    if (trimmed.size() < prefix.size() || trimmed.compare(0, prefix.size(), prefix) != 0) {
        return false;
    }
    return trimmed.size() == prefix.size() ||
           !IsIdentifierChar(trimmed[prefix.size()], false);
}

std::string CanonicalPath(const std::filesystem::path &path) {
    std::error_code error;
    const std::filesystem::path canonical = std::filesystem::weakly_canonical(path, error);
    return error ? path.lexically_normal().string() : canonical.string();
}

bool ReadFileText(const std::string &path, std::string *contents, std::string *error) {
    std::ifstream in(path, std::ios::in);
    if (!in.is_open()) {
        *error = "Unable to open used module: " + path;
        return false;
    }

    std::ostringstream buffer;
    buffer << in.rdbuf();
    if (!in && !in.eof()) {
        *error = "Failed to read used module: " + path;
        return false;
    }
    *contents = buffer.str();
    return true;
}

} // namespace

void Preprocessor::AddIncludePath(std::string path) {
    if (path.empty()) {
        return;
    }
    include_paths_.push_back(std::move(path));
}

bool Preprocessor::Process(const std::string &source, std::string *output) {
    errors_.clear();
    include_stack_.clear();
    processed_files_.clear();
    return ProcessText(source, "<source>", output);
}

bool Preprocessor::ProcessFile(const std::string &path, std::string *output) {
    errors_.clear();
    include_stack_.clear();
    processed_files_.clear();

    std::string contents;
    std::string error;
    if (!ReadFileText(path, &contents, &error)) {
        AddError(error);
        return false;
    }
    return ProcessText(contents, CanonicalPath(path), output);
}

const std::vector<std::string> &Preprocessor::Errors() const {
    return errors_;
}

bool Preprocessor::ProcessText(const std::string &source, const std::string &origin, std::string *output) {
    if (!origin.empty() && origin != "<source>") {
        for (const std::string &frame : include_stack_) {
            if (frame == origin) {
                AddError("Circular use: " + origin);
                return false;
            }
        }
        if (!processed_files_.insert(origin).second) {
            return true;
        }
        include_stack_.push_back(origin);
    }

    std::istringstream input(source);
    std::ostringstream expanded;
    std::string line;
    while (std::getline(input, line)) {
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }

        if (LooksLikeUse(line)) {
            std::string name;
            if (!ParseUseLine(line, &name)) {
                AddError("Invalid use statement in " + origin);
                return false;
            }

            std::string resolved;
            if (!ResolveUse(name, &resolved)) {
                AddError("Cannot resolve use '" + name + "' from " + origin);
                return false;
            }

            std::string used_source;
            std::string error;
            if (!ReadFileText(resolved, &used_source, &error)) {
                AddError(error);
                return false;
            }
            std::string used;
            if (!ProcessText(used_source, CanonicalPath(resolved), &used)) {
                return false;
            }
            expanded << used;
            if (!used.empty() && used.back() != '\n') {
                expanded << '\n';
            }
            continue;
        }

        expanded << line << '\n';
    }

    if (!origin.empty() && origin != "<source>" && !include_stack_.empty()) {
        include_stack_.pop_back();
    }

    *output += expanded.str();
    return errors_.empty();
}

bool Preprocessor::ResolveUse(const std::string &name, std::string *resolved_path) const {
    const std::filesystem::path file_name = name + ".orl";
    for (const std::string &include_path : include_paths_) {
        const std::filesystem::path candidate = std::filesystem::path(include_path) / file_name;
        std::error_code error;
        if (std::filesystem::is_regular_file(candidate, error)) {
            *resolved_path = CanonicalPath(candidate);
            return true;
        }
    }
    return false;
}

void Preprocessor::AddError(const std::string &message) {
    errors_.push_back(message);
}

} // namespace orlcomp
