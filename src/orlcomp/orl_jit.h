#pragma once

#include "orl_optimizer.h"

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace llvm {
class LLVMContext;
class Module;
}

namespace orlcomp {

enum class OrlJitTarget : std::uint8_t {
    Native = 0,
    Cuda = 1,
    Rocm = 2
};

class OrlJitEngine {
public:
    explicit OrlJitEngine(OrlJitTarget target = OrlJitTarget::Native);
    ~OrlJitEngine();

    bool LoadModule(std::unique_ptr<llvm::Module> module, std::unique_ptr<llvm::LLVMContext> context);
    bool LoadModuleWithOptimization(std::unique_ptr<llvm::Module> module,
                                    std::unique_ptr<llvm::LLVMContext> context,
                                    OrlOptimizationLevel level = OrlOptimizationLevel::O2);

    std::optional<int64_t> InvokeInt64(const std::string &name);
    std::optional<int64_t> InvokeInt64(const std::string &name, int64_t arg);
    OrlJitTarget Target() const;

    const std::vector<std::string> &Errors() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace orlcomp
