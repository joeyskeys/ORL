#pragma once

#include "orl_optimizer.h"

#include <cstddef>
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

enum class OrlGpuBackend : std::uint8_t {
    Cuda = 0,
    Rocm = 1
};

using OrlGpuBuffer = std::uint64_t;

class OrlGpuEngine {
public:
    static constexpr const char *CudaEntryKernelName = "orl_cuda_entry";
    static constexpr const char *CudaResultSymbolName = "orl_cuda_result";

    explicit OrlGpuEngine(OrlGpuBackend backend = OrlGpuBackend::Cuda);
    ~OrlGpuEngine();

    bool CompileModule(std::unique_ptr<llvm::Module> module, std::unique_ptr<llvm::LLVMContext> context);
    bool CompileModuleWithOptimization(std::unique_ptr<llvm::Module> module,
                                       std::unique_ptr<llvm::LLVMContext> context,
                                       OrlOptimizationLevel level = OrlOptimizationLevel::O2);

    void SetDeviceCode(std::string device_code);
    bool LoadToDriver();
    void UnloadDriverModule();
    std::optional<OrlGpuBuffer> AllocateBuffer(std::size_t bytes);
    bool UploadBuffer(OrlGpuBuffer buffer, const void *source, std::size_t bytes);
    bool DownloadBuffer(OrlGpuBuffer buffer, void *destination, std::size_t bytes);
    bool FreeBuffer(OrlGpuBuffer buffer);
    bool Synchronize();
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
