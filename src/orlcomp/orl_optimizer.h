#pragma once

#include <memory>
#include <string>
#include <vector>

namespace llvm {
class Module;
}

namespace orlcomp {

enum class OrlOptimizationLevel {
    O0,
    O1,
    O2,
    O3
};

class LlvmOptimizer {
public:
    explicit LlvmOptimizer(OrlOptimizationLevel level = OrlOptimizationLevel::O2);
    ~LlvmOptimizer();

    bool Optimize(llvm::Module &module);
    const std::vector<std::string> &Errors() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace orlcomp
