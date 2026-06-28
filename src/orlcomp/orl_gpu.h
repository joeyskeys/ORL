#pragma once

#include "orl_optimizer.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace llvm {
class LLVMContext;
class Module;
}

namespace orlcomp {

enum class OrlGpuBackend : std::uint8_t {
    Cuda = 0,
    Rocm = 1
};

class OrlGpuEngine {
public:
    explicit OrlGpuEngine(OrlGpuBackend backend = OrlGpuBackend::Cuda);
    ~OrlGpuEngine();

    bool CompileModule(std::unique_ptr<llvm::Module> module, std::unique_ptr<llvm::LLVMContext> context);
    bool CompileModuleWithOptimization(std::unique_ptr<llvm::Module> module,
                                       std::unique_ptr<llvm::LLVMContext> context,
                                       OrlOptimizationLevel level = OrlOptimizationLevel::O2);

    void SetDeviceCode(std::string device_code);
    bool LoadToDriver();
    void UnloadDriverModule();
    bool RunCudaInt32AddKernel(const std::string &kernel_name,
                               std::vector<std::int32_t> *values,
                               std::int32_t addend,
                               std::uint32_t threads_per_block = 128);
    bool RunCudaKernelNoArgs(const std::string &kernel_name,
                             std::uint32_t blocks = 1,
                             std::uint32_t threads_per_block = 1);
    bool ReadCudaGlobalInt32(const std::string &symbol_name, std::int32_t *value);

    OrlGpuBackend Backend() const;
    bool IsDriverModuleLoaded() const;
    const std::string &DeviceCode() const;
    const std::vector<std::string> &Errors() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace orlcomp
