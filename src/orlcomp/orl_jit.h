#pragma once

#include "orl_optimizer.h"

#include <array>
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
    std::optional<int64_t> InvokeInt64WithBufferArgs(const std::string &name,
                                                      const std::array<void *, 5> &buffers,
                                                      int64_t scalar_arg);
    // Calls the canonical wrapper emitted by codegen for application-facing
    // execution: int64_t(void* const* buffers, const int64_t* integers,
    //                     const double* floats).
    std::optional<int64_t> InvokeInt64WithRuntimeArgs(const std::string &name,
                                                       void *const *buffers,
                                                       const int64_t *integers,
                                                       const double *floats);
    OrlJitTarget Target() const;

    const std::vector<std::string> &Errors() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace orlcomp
