#pragma once

#include "orl_optimizer.h"
#include "orl_runtime_signature.h"

#include <cstddef>
#include <cstring>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <type_traits>
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

struct OrlGpuBufferView {
    std::uint64_t device_ptr = 0;
    std::size_t bytes = 0;
};

enum class OrlGpuKernelParameterType : std::uint8_t {
    Buffer,
    Int64,
    Float64,
    Handle,
    Unsupported
};

struct OrlGpuKernelParameter {
    std::string name;
    OrlGpuKernelParameterType type = OrlGpuKernelParameterType::Unsupported;
    std::size_t byte_size = 0;
    std::size_t alignment = 0;
    std::uint32_t lane_count = 0;
    std::string lane_order;
};

struct OrlGpuBufferBinding {
    OrlGpuBuffer buffer = 0;
};

inline OrlGpuBufferBinding BindGpuBuffer(OrlGpuBuffer buffer) {
    return OrlGpuBufferBinding{buffer};
}

struct OrlGpuKernelArgument {
    bool is_buffer = false;
    OrlGpuBuffer buffer = 0;
    std::size_t buffer_offset = 0;
    OrlGpuKernelParameterType scalar_type = OrlGpuKernelParameterType::Unsupported;
    std::vector<std::uint8_t> scalar_bytes;
    std::size_t scalar_alignment = 0;
    std::uint32_t lane_count = 0;
    std::string lane_order;
};

class OrlGpuEngine {
public:
    static constexpr const char *CudaEntryKernelName = "orl_cuda_entry";
    static constexpr const char *CudaResultSymbolName = "orl_cuda_result";

    explicit OrlGpuEngine(OrlGpuBackend backend = OrlGpuBackend::Cuda);
    ~OrlGpuEngine();

    void SetCudaEntryFunction(std::string function_name);
    bool CompileModule(std::unique_ptr<llvm::Module> module, std::unique_ptr<llvm::LLVMContext> context);
    bool CompileModuleWithOptimization(std::unique_ptr<llvm::Module> module,
                                       std::unique_ptr<llvm::LLVMContext> context,
                                       OrlOptimizationLevel level = OrlOptimizationLevel::O2);

    void SetDeviceCode(std::string device_code);
    // Sets a driver-loadable CUDA image, such as a cubin returned by
    // cuLinkComplete. This bypasses PTX generation and driver linking.
    void SetDeviceBinary(std::vector<std::uint8_t> device_binary);
    bool LoadToDriver();
    void UnloadDriverModule();
    std::optional<OrlGpuBuffer> AllocateBuffer(std::size_t bytes);
    std::optional<OrlGpuBuffer> ImportBuffer(std::uint64_t device_ptr, std::size_t bytes);
    std::optional<OrlGpuBufferView> DeviceBufferView(OrlGpuBuffer buffer);
    std::optional<std::uint64_t> DeviceBufferPointer(OrlGpuBuffer buffer);
    bool UploadBuffer(OrlGpuBuffer buffer, const void *source, std::size_t bytes);
    bool DownloadBuffer(OrlGpuBuffer buffer, void *destination, std::size_t bytes);
    bool FreeBuffer(OrlGpuBuffer buffer);
    bool Synchronize();
    template <typename... Args>
    bool SetupCudaKernel(const std::string &kernel_name, const Args &... args) {
        std::vector<OrlGpuKernelArgument> arguments;
        arguments.reserve(sizeof...(Args));
        (arguments.push_back(MakeKernelArgument(args)), ...);
        return SetupCudaKernelArguments(kernel_name, std::move(arguments));
    }
    // Runtime integrations that bind arguments by parameter name can construct
    // the reflected argument list dynamically rather than at a call site.
    bool SetupCudaKernelArguments(const std::string &kernel_name,
                                  std::vector<OrlGpuKernelArgument> arguments);
    bool LaunchCudaKernel(std::uint32_t block_count, std::uint32_t threads_per_block);
    bool SetDefaultThreadsPerBlock(std::uint32_t threads_per_block);
    std::uint32_t DefaultThreadsPerBlock() const;
    bool LaunchCudaKernelForElements(std::uint32_t element_count,
                                     std::uint32_t suggested_threads_per_block = 0);
    bool RunCudaInt32AddKernel(const std::string &kernel_name,
                               std::vector<std::int32_t> *values,
                               std::int32_t addend,
                               std::uint32_t threads_per_block = 128);
    bool RunCudaKernelNoArgs(const std::string &kernel_name,
                             std::uint32_t blocks = 1,
                             std::uint32_t threads_per_block = 1);
    bool ReadCudaGlobalInt32(const std::string &symbol_name, std::int32_t *value);
    void SetCudaEntryParameters(
        std::vector<OrlGpuKernelParameter> parameters);

    OrlGpuBackend Backend() const;
    bool IsDriverModuleLoaded() const;
    const std::vector<OrlGpuKernelParameter> &CudaEntryParameters() const;
    const std::string &DeviceCode() const;
    const std::vector<std::uint8_t> &DeviceBinary() const;
    const std::vector<std::string> &Errors() const;

private:
    static OrlGpuKernelArgument MakeKernelArgument(OrlGpuBufferBinding binding) {
        OrlGpuKernelArgument argument;
        argument.is_buffer = true;
        argument.buffer = binding.buffer;
        argument.scalar_type = OrlGpuKernelParameterType::Buffer;
        return argument;
    }

    static OrlGpuKernelArgument MakeKernelArgument(
        const HandleValue& value) {
        OrlGpuKernelArgument argument;
        argument.scalar_type = OrlGpuKernelParameterType::Handle;
        argument.scalar_bytes.resize(sizeof(std::uint64_t) * 2);
        argument.scalar_alignment = alignof(HandleValue);
        argument.lane_count = 2;
        argument.lane_order = "type_id,slot";
        std::memcpy(argument.scalar_bytes.data(),
            &value.type_id, sizeof(std::uint64_t));
        const auto slot = static_cast<std::uint64_t>(value.slot);
        std::memcpy(argument.scalar_bytes.data() + sizeof(std::uint64_t),
            &slot, sizeof(std::uint64_t));
        return argument;
    }

    template <typename T>
    static OrlGpuKernelArgument MakeKernelArgument(const T &value) {
        static_assert(std::is_trivially_copyable_v<T>,
                      "CUDA kernel scalar arguments must be trivially copyable");
        OrlGpuKernelArgument argument;
        if constexpr (std::is_same_v<std::remove_cv_t<T>, std::int64_t>) {
            argument.scalar_type = OrlGpuKernelParameterType::Int64;
        } else if constexpr (std::is_same_v<std::remove_cv_t<T>, double>) {
            argument.scalar_type = OrlGpuKernelParameterType::Float64;
        }
        argument.scalar_bytes.resize(sizeof(T));
        std::memcpy(argument.scalar_bytes.data(), &value, sizeof(T));
        return argument;
    }

    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace orlcomp
