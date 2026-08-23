#pragma once

#include <string>
#include <unordered_set>
#include <vector>

namespace orlcomp {

class Preprocessor {
public:
    void AddIncludePath(std::string path);

    bool Process(const std::string &source, std::string *output);
    bool ProcessFile(const std::string &path, std::string *output);

    const std::vector<std::string> &Errors() const;

private:
    bool ProcessText(const std::string &source, const std::string &origin, std::string *output);
    bool ResolveUse(const std::string &name, std::string *resolved_path) const;
    void AddError(const std::string &message);

    std::vector<std::string> include_paths_;
    std::vector<std::string> include_stack_;
    std::unordered_set<std::string> processed_files_;
    std::vector<std::string> errors_;
};

} // namespace orlcomp
