#pragma once

#include "orl_cache.h"
#include "orl_optimizer.h"

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
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
    explicit OrlJitEngine(OrlJitTarget target = OrlJitTarget::Native,
        OrlBinaryCacheOptions cache = {});
    ~OrlJitEngine();

    bool LoadModule(std::unique_ptr<llvm::Module> module,
        std::unique_ptr<llvm::LLVMContext> context,
        std::string cache_key = {});
    bool LoadModuleWithOptimization(std::unique_ptr<llvm::Module> module,
                                    std::unique_ptr<llvm::LLVMContext> context,
                                    OrlOptimizationLevel level = OrlOptimizationLevel::O2,
                                    std::string cache_key = {});
    // Loads a previously compiled host object without generating LLVM IR.
    bool LoadObject(std::span<const std::uint8_t> object);

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
    // Context-aware variant used when the ORL source references the implicit
    // solver_context global.
    std::optional<int64_t> InvokeInt64WithRuntimeArgsAndContext(
        const std::string &name,
        void *const *buffers,
        const int64_t *integers,
        const double *floats,
        void *solver_context);
    std::optional<int64_t> InvokeInt64WithRuntimeArgsAndHierarchyContext(
        const std::string &name,
        void *const *buffers,
        const int64_t *integers,
        const double *floats,
        void *hierarchy_context,
        void *hierarchy_data);
    std::optional<int64_t> InvokeInt64WithRuntimeArgsAndContexts(
        const std::string &name,
        void *const *buffers,
        const int64_t *integers,
        const double *floats,
        void *solver_context,
        void *hierarchy_context,
        void *hierarchy_data);
    OrlJitTarget Target() const;

    const std::vector<std::string> &Errors() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace orlcomp
