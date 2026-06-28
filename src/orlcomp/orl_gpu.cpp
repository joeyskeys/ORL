#include "orl_gpu.h"

#if __has_include(<llvm/MC/TargetRegistry.h>) && \
    __has_include(<llvm/Target/TargetMachine.h>) && \
    __has_include(<llvm/Support/CodeGen.h>)

#include <llvm/IR/LegacyPassManager.h>
#include <llvm/IR/LLVMContext.h>
#include <llvm/IR/Module.h>
#include <llvm/MC/TargetRegistry.h>
#include <llvm/Support/Error.h>
#include <llvm/Support/TargetSelect.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/Target/TargetMachine.h>

#if !__has_include(<cuda.h>)
#error "CUDA Toolkit header 'cuda.h' not found. Install CUDA Toolkit and ensure include paths are configured."
#endif

#define ORL_HAS_CUDA_HEADERS 1
#include <cuda.h>

#if defined(_WIN32)
#include <windows.h>
#else
#include <dlfcn.h>
#endif

#include <optional>
#include <utility>

namespace orlcomp {

namespace {

std::string FormatLlvmError(const llvm::Error &error) {
    std::string message;
    llvm::raw_string_ostream stream(message);
    stream << error;
    return stream.str();
}

const char *BackendName(OrlGpuBackend backend) {
    switch (backend) {
    case OrlGpuBackend::Cuda:
        return "cuda";
    case OrlGpuBackend::Rocm:
        return "rocm";
    default:
        return "unknown";
    }
}

std::string TargetTripleFor(OrlGpuBackend backend) {
    return backend == OrlGpuBackend::Cuda ? "nvptx64-nvidia-cuda" : "amdgcn-amd-amdhsa";
}

std::string TargetCpuFor(OrlGpuBackend backend) {
    // Conservative defaults; callers can extend this later with user-selected arch.
    return backend == OrlGpuBackend::Cuda ? "sm_52" : "gfx900";
}

} // namespace

struct OrlGpuEngine::Impl {
    explicit Impl(OrlGpuBackend backend) : backend_(backend) {
        llvm::InitializeAllTargetInfos();
        llvm::InitializeAllTargets();
        llvm::InitializeAllTargetMCs();
        llvm::InitializeAllAsmPrinters();
        llvm::InitializeAllAsmParsers();
    }

    ~Impl() = default;

    bool CompileModule(std::unique_ptr<llvm::Module> module, std::unique_ptr<llvm::LLVMContext> context) {
        errors_.clear();
        device_code_.clear();
        UnloadDriverModule();

        if (module == nullptr || context == nullptr) {
            errors_.push_back("CompileModule requires non-null LLVM module and context");
            return false;
        }

        std::string target_error;
        const std::string target_triple = TargetTripleFor(backend_);
        const std::string target_cpu = TargetCpuFor(backend_);
        const llvm::Target *target = llvm::TargetRegistry::lookupTarget(target_triple, target_error);
        if (target == nullptr) {
            errors_.push_back("Failed to find LLVM target for " + std::string(BackendName(backend_)) + ": " + target_error);
            return false;
        }

        const llvm::TargetOptions options;
        auto target_machine = std::unique_ptr<llvm::TargetMachine>(
            target->createTargetMachine(target_triple, target_cpu, "", options, std::nullopt, std::nullopt,
                                        llvm::CodeGenOptLevel::Aggressive));
        if (!target_machine) {
            errors_.push_back("Failed to create target machine for backend " + std::string(BackendName(backend_)));
            return false;
        }

        module->setTargetTriple(target_triple);
        module->setDataLayout(target_machine->createDataLayout());

        llvm::SmallVector<char, 0> output_buffer;
        llvm::raw_svector_ostream output_stream(output_buffer);
        llvm::legacy::PassManager pass_manager;

        const llvm::CodeGenFileType file_type =
            backend_ == OrlGpuBackend::Cuda ? llvm::CodeGenFileType::AssemblyFile : llvm::CodeGenFileType::ObjectFile;

        if (target_machine->addPassesToEmitFile(pass_manager, output_stream, nullptr, file_type)) {
            errors_.push_back("LLVM target backend cannot emit requested device file type");
            return false;
        }

        pass_manager.run(*module);
        device_code_.assign(output_buffer.begin(), output_buffer.end());

        module_ = std::move(module);
        context_ = std::move(context);
        return true;
    }

    bool CompileModuleWithOptimization(std::unique_ptr<llvm::Module> module,
                                       std::unique_ptr<llvm::LLVMContext> context,
                                       OrlOptimizationLevel level) {
        errors_.clear();
        if (module == nullptr || context == nullptr) {
            errors_.push_back("CompileModuleWithOptimization requires non-null LLVM module and context");
            return false;
        }

        LlvmOptimizer optimizer(level);
        if (!optimizer.Optimize(*module)) {
            errors_ = optimizer.Errors();
            return false;
        }

        return CompileModule(std::move(module), std::move(context));
    }

    bool LoadToDriver() {
        errors_.clear();
        if (device_code_.empty()) {
            errors_.push_back("No compiled device code. Call CompileModule first.");
            return false;
        }
        if (backend_ == OrlGpuBackend::Rocm) {
            errors_.push_back("ROCm driver loading is not implemented yet in OrlGpuEngine");
            return false;
        }

#if ORL_HAS_CUDA_HEADERS
        return LoadCudaDriverModule();
#else
        errors_.push_back("CUDA headers not available; cannot load CUDA module");
        return false;
#endif
    }

    void UnloadDriverModule() {
#if ORL_HAS_CUDA_HEADERS
        if (!cuda_loaded_) {
            return;
        }

        if (cuda_module_ != nullptr && cuModuleUnload_ != nullptr) {
            cuModuleUnload_(cuda_module_);
        }
        cuda_module_ = nullptr;

        if (cuda_context_ != nullptr && cuCtxDestroy_ != nullptr) {
            cuCtxDestroy_(cuda_context_);
        }
        cuda_context_ = nullptr;

        CloseCudaLibrary();
#endif
    }

#if ORL_HAS_CUDA_HEADERS
    using CuInitFn = CUresult(CUDAAPI *)(unsigned int);
    using CuDeviceGetFn = CUresult(CUDAAPI *)(CUdevice *, int);
    using CuCtxCreateFn = CUresult(CUDAAPI *)(CUcontext *, unsigned int, CUdevice);
    using CuCtxDestroyFn = CUresult(CUDAAPI *)(CUcontext);
    using CuModuleLoadDataExFn = CUresult(CUDAAPI *)(CUmodule *, const void *, unsigned int, CUjit_option *, void **);
    using CuModuleUnloadFn = CUresult(CUDAAPI *)(CUmodule);
    using CuModuleGetFunctionFn = CUresult(CUDAAPI *)(CUfunction *, CUmodule, const char *);
    using CuLaunchKernelFn = CUresult(CUDAAPI *)(CUfunction,
                                                 unsigned int,
                                                 unsigned int,
                                                 unsigned int,
                                                 unsigned int,
                                                 unsigned int,
                                                 unsigned int,
                                                 unsigned int,
                                                 CUstream,
                                                 void **,
                                                 void **);
    using CuCtxSynchronizeFn = CUresult(CUDAAPI *)(void);
    using CuMemAllocFn = CUresult(CUDAAPI *)(CUdeviceptr *, size_t);
    using CuMemFreeFn = CUresult(CUDAAPI *)(CUdeviceptr);
    using CuMemcpyHtoDFn = CUresult(CUDAAPI *)(CUdeviceptr, const void *, size_t);
    using CuMemcpyDtoHFn = CUresult(CUDAAPI *)(void *, CUdeviceptr, size_t);
    using CuModuleGetGlobalFn = CUresult(CUDAAPI *)(CUdeviceptr *, size_t *, CUmodule, const char *);
    using CuGetErrorNameFn = CUresult(CUDAAPI *)(CUresult, const char **);
    using CuGetErrorStringFn = CUresult(CUDAAPI *)(CUresult, const char **);

    bool LoadCudaDriverModule() {
        if (!EnsureCudaApiLoaded()) {
            return false;
        }

        CUresult rc = cuInit_(0);
        if (rc != CUDA_SUCCESS) {
            AddCudaError("cuInit failed", rc);
            return false;
        }

        CUdevice device = 0;
        rc = cuDeviceGet_(&device, 0);
        if (rc != CUDA_SUCCESS) {
            AddCudaError("cuDeviceGet failed", rc);
            return false;
        }

        rc = cuCtxCreate_(&cuda_context_, 0, device);
        if (rc != CUDA_SUCCESS) {
            AddCudaError("cuCtxCreate failed", rc);
            cuda_context_ = nullptr;
            return false;
        }

        rc = cuModuleLoadDataEx_(&cuda_module_, device_code_.data(), 0, nullptr, nullptr);
        if (rc != CUDA_SUCCESS) {
            AddCudaError("cuModuleLoadDataEx failed", rc);
            cuCtxDestroy_(cuda_context_);
            cuda_context_ = nullptr;
            cuda_module_ = nullptr;
            return false;
        }

        return true;
    }

    void AddCudaError(const std::string &prefix, CUresult rc) {
        std::string message = prefix + " (" + std::to_string(static_cast<int>(rc)) + ")";
        if (cuGetErrorName_ != nullptr) {
            const char *name = nullptr;
            if (cuGetErrorName_(rc, &name) == CUDA_SUCCESS && name != nullptr) {
                message += " " + std::string(name);
            }
        }
        if (cuGetErrorString_ != nullptr) {
            const char *desc = nullptr;
            if (cuGetErrorString_(rc, &desc) == CUDA_SUCCESS && desc != nullptr) {
                message += ": " + std::string(desc);
            }
        }
        errors_.push_back(std::move(message));
    }

    bool EnsureCudaApiLoaded() {
        if (cuda_loaded_) {
            return true;
        }

#if defined(_WIN32)
        cuda_library_ = LoadLibraryA("nvcuda.dll");
#else
        cuda_library_ = dlopen("libcuda.so.1", RTLD_NOW);
        if (!cuda_library_) {
            cuda_library_ = dlopen("libcuda.so", RTLD_NOW);
        }
#endif
        if (cuda_library_ == nullptr) {
            errors_.push_back("Failed to load CUDA driver library");
            return false;
        }

        if (!LoadCudaSymbol(cuInit_, "cuInit") ||
            !LoadCudaSymbol(cuDeviceGet_, "cuDeviceGet") ||
            !LoadCudaSymbol(cuCtxCreate_, "cuCtxCreate_v2") ||
            !LoadCudaSymbol(cuCtxDestroy_, "cuCtxDestroy_v2") ||
            !LoadCudaSymbol(cuModuleLoadDataEx_, "cuModuleLoadDataEx") ||
            !LoadCudaSymbol(cuModuleUnload_, "cuModuleUnload") ||
            !LoadCudaSymbol(cuModuleGetFunction_, "cuModuleGetFunction") ||
            !LoadCudaSymbol(cuLaunchKernel_, "cuLaunchKernel") ||
            !LoadCudaSymbol(cuCtxSynchronize_, "cuCtxSynchronize") ||
            !LoadCudaSymbol(cuMemAlloc_, "cuMemAlloc_v2") ||
            !LoadCudaSymbol(cuMemFree_, "cuMemFree_v2") ||
            !LoadCudaSymbol(cuMemcpyHtoD_, "cuMemcpyHtoD_v2") ||
            !LoadCudaSymbol(cuMemcpyDtoH_, "cuMemcpyDtoH_v2") ||
            !LoadCudaSymbol(cuModuleGetGlobal_, "cuModuleGetGlobal_v2") ||
            !LoadCudaSymbol(cuGetErrorName_, "cuGetErrorName") ||
            !LoadCudaSymbol(cuGetErrorString_, "cuGetErrorString")) {
            CloseCudaLibrary();
            errors_.push_back("Failed to resolve required CUDA driver symbols");
            return false;
        }

        cuda_loaded_ = true;
        return true;
    }

    template <typename T>
    bool LoadCudaSymbol(T &function, const char *name) {
#if defined(_WIN32)
        FARPROC proc = GetProcAddress(static_cast<HMODULE>(cuda_library_), name);
        if (proc == nullptr) {
            return false;
        }
        function = reinterpret_cast<T>(proc);
#else
        void *proc = dlsym(cuda_library_, name);
        if (proc == nullptr) {
            return false;
        }
        function = reinterpret_cast<T>(proc);
#endif
        return true;
    }

    void CloseCudaLibrary() {
        if (cuda_library_ == nullptr) {
            return;
        }
#if defined(_WIN32)
        FreeLibrary(static_cast<HMODULE>(cuda_library_));
#else
        dlclose(cuda_library_);
#endif
        cuda_library_ = nullptr;
        cuda_loaded_ = false;
    }
#endif

    OrlGpuBackend backend_ = OrlGpuBackend::Cuda;
    std::unique_ptr<llvm::LLVMContext> context_;
    std::unique_ptr<llvm::Module> module_;
    std::string device_code_;
    std::vector<std::string> errors_;

#if ORL_HAS_CUDA_HEADERS
    void *cuda_library_ = nullptr;
    bool cuda_loaded_ = false;
    CUcontext cuda_context_ = nullptr;
    CUmodule cuda_module_ = nullptr;

    CuInitFn cuInit_ = nullptr;
    CuDeviceGetFn cuDeviceGet_ = nullptr;
    CuCtxCreateFn cuCtxCreate_ = nullptr;
    CuCtxDestroyFn cuCtxDestroy_ = nullptr;
    CuModuleLoadDataExFn cuModuleLoadDataEx_ = nullptr;
    CuModuleUnloadFn cuModuleUnload_ = nullptr;
    CuModuleGetFunctionFn cuModuleGetFunction_ = nullptr;
    CuLaunchKernelFn cuLaunchKernel_ = nullptr;
    CuCtxSynchronizeFn cuCtxSynchronize_ = nullptr;
    CuMemAllocFn cuMemAlloc_ = nullptr;
    CuMemFreeFn cuMemFree_ = nullptr;
    CuMemcpyHtoDFn cuMemcpyHtoD_ = nullptr;
    CuMemcpyDtoHFn cuMemcpyDtoH_ = nullptr;
    CuModuleGetGlobalFn cuModuleGetGlobal_ = nullptr;
    CuGetErrorNameFn cuGetErrorName_ = nullptr;
    CuGetErrorStringFn cuGetErrorString_ = nullptr;
#endif
};

OrlGpuEngine::OrlGpuEngine(OrlGpuBackend backend) : impl_(std::make_unique<Impl>(backend)) {}
OrlGpuEngine::~OrlGpuEngine() = default;

bool OrlGpuEngine::CompileModule(std::unique_ptr<llvm::Module> module, std::unique_ptr<llvm::LLVMContext> context) {
    return impl_->CompileModule(std::move(module), std::move(context));
}

bool OrlGpuEngine::CompileModuleWithOptimization(std::unique_ptr<llvm::Module> module,
                                                 std::unique_ptr<llvm::LLVMContext> context,
                                                 OrlOptimizationLevel level) {
    return impl_->CompileModuleWithOptimization(std::move(module), std::move(context), level);
}

void OrlGpuEngine::SetDeviceCode(std::string device_code) {
    impl_->errors_.clear();
    impl_->UnloadDriverModule();
    impl_->device_code_ = std::move(device_code);
}

bool OrlGpuEngine::LoadToDriver() {
    return impl_->LoadToDriver();
}

void OrlGpuEngine::UnloadDriverModule() {
    impl_->UnloadDriverModule();
}

OrlGpuBackend OrlGpuEngine::Backend() const {
    return impl_->backend_;
}

bool OrlGpuEngine::IsDriverModuleLoaded() const {
#if ORL_HAS_CUDA_HEADERS
    if (impl_->backend_ == OrlGpuBackend::Cuda) {
        return impl_->cuda_module_ != nullptr;
    }
#endif
    return false;
}

bool OrlGpuEngine::RunCudaInt32AddKernel(const std::string &kernel_name,
                                         std::vector<std::int32_t> *values,
                                         std::int32_t addend,
                                         std::uint32_t threads_per_block) {
    impl_->errors_.clear();
    if (impl_->backend_ != OrlGpuBackend::Cuda) {
        impl_->errors_.push_back("RunCudaInt32AddKernel requires CUDA backend");
        return false;
    }
    if (values == nullptr) {
        impl_->errors_.push_back("RunCudaInt32AddKernel requires non-null values vector");
        return false;
    }
    if (values->empty()) {
        return true;
    }
    if (threads_per_block == 0) {
        impl_->errors_.push_back("threads_per_block must be greater than zero");
        return false;
    }

#if ORL_HAS_CUDA_HEADERS
    if (!IsDriverModuleLoaded() && !LoadToDriver()) {
        return false;
    }

    CUfunction kernel = nullptr;
    CUresult rc = impl_->cuModuleGetFunction_(&kernel, impl_->cuda_module_, kernel_name.c_str());
    if (rc != CUDA_SUCCESS) {
        impl_->AddCudaError("cuModuleGetFunction failed", rc);
        return false;
    }

    CUdeviceptr device_buffer = 0;
    const std::size_t bytes = values->size() * sizeof(std::int32_t);
    rc = impl_->cuMemAlloc_(&device_buffer, bytes);
    if (rc != CUDA_SUCCESS) {
        impl_->AddCudaError("cuMemAlloc failed", rc);
        return false;
    }

    rc = impl_->cuMemcpyHtoD_(device_buffer, values->data(), bytes);
    if (rc != CUDA_SUCCESS) {
        impl_->AddCudaError("cuMemcpyHtoD failed", rc);
        impl_->cuMemFree_(device_buffer);
        return false;
    }

    std::uint32_t element_count = static_cast<std::uint32_t>(values->size());
    void *kernel_args[] = {&device_buffer, &element_count, &addend};
    const std::uint32_t blocks = (element_count + threads_per_block - 1) / threads_per_block;
    rc = impl_->cuLaunchKernel_(kernel,
                                blocks,
                                1,
                                1,
                                threads_per_block,
                                1,
                                1,
                                0,
                                nullptr,
                                kernel_args,
                                nullptr);
    if (rc != CUDA_SUCCESS) {
        impl_->AddCudaError("cuLaunchKernel failed", rc);
        impl_->cuMemFree_(device_buffer);
        return false;
    }

    rc = impl_->cuCtxSynchronize_();
    if (rc != CUDA_SUCCESS) {
        impl_->AddCudaError("cuCtxSynchronize failed", rc);
        impl_->cuMemFree_(device_buffer);
        return false;
    }

    rc = impl_->cuMemcpyDtoH_(values->data(), device_buffer, bytes);
    if (rc != CUDA_SUCCESS) {
        impl_->AddCudaError("cuMemcpyDtoH failed", rc);
        impl_->cuMemFree_(device_buffer);
        return false;
    }

    rc = impl_->cuMemFree_(device_buffer);
    if (rc != CUDA_SUCCESS) {
        impl_->AddCudaError("cuMemFree failed", rc);
        return false;
    }

    return true;
#else
    (void)kernel_name;
    (void)addend;
    (void)threads_per_block;
    impl_->errors_.push_back("CUDA headers not available; cannot run CUDA kernels");
    return false;
#endif
}

bool OrlGpuEngine::RunCudaKernelNoArgs(const std::string &kernel_name,
                                       std::uint32_t blocks,
                                       std::uint32_t threads_per_block) {
    impl_->errors_.clear();
    if (impl_->backend_ != OrlGpuBackend::Cuda) {
        impl_->errors_.push_back("RunCudaKernelNoArgs requires CUDA backend");
        return false;
    }
    if (blocks == 0 || threads_per_block == 0) {
        impl_->errors_.push_back("blocks and threads_per_block must be greater than zero");
        return false;
    }

#if ORL_HAS_CUDA_HEADERS
    if (!IsDriverModuleLoaded() && !LoadToDriver()) {
        return false;
    }

    CUfunction kernel = nullptr;
    CUresult rc = impl_->cuModuleGetFunction_(&kernel, impl_->cuda_module_, kernel_name.c_str());
    if (rc != CUDA_SUCCESS) {
        impl_->AddCudaError("cuModuleGetFunction failed", rc);
        return false;
    }

    rc = impl_->cuLaunchKernel_(kernel,
                                blocks,
                                1,
                                1,
                                threads_per_block,
                                1,
                                1,
                                0,
                                nullptr,
                                nullptr,
                                nullptr);
    if (rc != CUDA_SUCCESS) {
        impl_->AddCudaError("cuLaunchKernel failed", rc);
        return false;
    }

    rc = impl_->cuCtxSynchronize_();
    if (rc != CUDA_SUCCESS) {
        impl_->AddCudaError("cuCtxSynchronize failed", rc);
        return false;
    }

    return true;
#else
    (void)kernel_name;
    (void)blocks;
    (void)threads_per_block;
    impl_->errors_.push_back("CUDA headers not available; cannot run CUDA kernels");
    return false;
#endif
}

bool OrlGpuEngine::ReadCudaGlobalInt32(const std::string &symbol_name, std::int32_t *value) {
    impl_->errors_.clear();
    if (impl_->backend_ != OrlGpuBackend::Cuda) {
        impl_->errors_.push_back("ReadCudaGlobalInt32 requires CUDA backend");
        return false;
    }
    if (value == nullptr) {
        impl_->errors_.push_back("ReadCudaGlobalInt32 requires non-null output pointer");
        return false;
    }

#if ORL_HAS_CUDA_HEADERS
    if (!IsDriverModuleLoaded() && !LoadToDriver()) {
        return false;
    }

    CUdeviceptr device_symbol = 0;
    std::size_t bytes = 0;
    CUresult rc = impl_->cuModuleGetGlobal_(&device_symbol, &bytes, impl_->cuda_module_, symbol_name.c_str());
    if (rc != CUDA_SUCCESS) {
        impl_->AddCudaError("cuModuleGetGlobal failed", rc);
        return false;
    }
    if (bytes < sizeof(std::int32_t)) {
        impl_->errors_.push_back("CUDA global symbol '" + symbol_name + "' is smaller than int32");
        return false;
    }

    rc = impl_->cuMemcpyDtoH_(value, device_symbol, sizeof(std::int32_t));
    if (rc != CUDA_SUCCESS) {
        impl_->AddCudaError("cuMemcpyDtoH failed", rc);
        return false;
    }
    return true;
#else
    (void)symbol_name;
    impl_->errors_.push_back("CUDA headers not available; cannot read CUDA globals");
    return false;
#endif
}

const std::string &OrlGpuEngine::DeviceCode() const {
    return impl_->device_code_;
}

const std::vector<std::string> &OrlGpuEngine::Errors() const {
    return impl_->errors_;
}

} // namespace orlcomp

#else

namespace orlcomp {

struct OrlGpuEngine::Impl {
    explicit Impl(OrlGpuBackend backend) : backend_(backend) {}

    bool CompileModule(std::unique_ptr<llvm::Module>, std::unique_ptr<llvm::LLVMContext>) {
        errors_.clear();
        errors_.push_back("LLVM target-machine headers are unavailable; GPU code generation disabled");
        return false;
    }

    bool CompileModuleWithOptimization(std::unique_ptr<llvm::Module>,
                                       std::unique_ptr<llvm::LLVMContext>,
                                       OrlOptimizationLevel) {
        return CompileModule(nullptr, nullptr);
    }

    bool LoadToDriver() {
        errors_.clear();
        errors_.push_back("GPU driver loading unavailable in this build");
        return false;
    }

    void UnloadDriverModule() {}

    OrlGpuBackend backend_ = OrlGpuBackend::Cuda;
    std::string device_code_;
    std::vector<std::string> errors_;
};

OrlGpuEngine::OrlGpuEngine(OrlGpuBackend backend) : impl_(std::make_unique<Impl>(backend)) {}
OrlGpuEngine::~OrlGpuEngine() = default;

bool OrlGpuEngine::CompileModule(std::unique_ptr<llvm::Module> module, std::unique_ptr<llvm::LLVMContext> context) {
    return impl_->CompileModule(std::move(module), std::move(context));
}

bool OrlGpuEngine::CompileModuleWithOptimization(std::unique_ptr<llvm::Module> module,
                                                 std::unique_ptr<llvm::LLVMContext> context,
                                                 OrlOptimizationLevel level) {
    return impl_->CompileModuleWithOptimization(std::move(module), std::move(context), level);
}

void OrlGpuEngine::SetDeviceCode(std::string device_code) {
    impl_->errors_.clear();
    impl_->device_code_ = std::move(device_code);
}

bool OrlGpuEngine::LoadToDriver() {
    return impl_->LoadToDriver();
}

void OrlGpuEngine::UnloadDriverModule() {
    impl_->UnloadDriverModule();
}

OrlGpuBackend OrlGpuEngine::Backend() const {
    return impl_->backend_;
}

bool OrlGpuEngine::IsDriverModuleLoaded() const {
    return false;
}

bool OrlGpuEngine::RunCudaInt32AddKernel(const std::string &,
                                         std::vector<std::int32_t> *,
                                         std::int32_t,
                                         std::uint32_t) {
    impl_->errors_.clear();
    impl_->errors_.push_back("CUDA kernel execution unavailable in this build");
    return false;
}

bool OrlGpuEngine::RunCudaKernelNoArgs(const std::string &, std::uint32_t, std::uint32_t) {
    impl_->errors_.clear();
    impl_->errors_.push_back("CUDA kernel execution unavailable in this build");
    return false;
}

bool OrlGpuEngine::ReadCudaGlobalInt32(const std::string &, std::int32_t *) {
    impl_->errors_.clear();
    impl_->errors_.push_back("CUDA global read unavailable in this build");
    return false;
}

const std::string &OrlGpuEngine::DeviceCode() const {
    return impl_->device_code_;
}

const std::vector<std::string> &OrlGpuEngine::Errors() const {
    return impl_->errors_;
}

} // namespace orlcomp

#endif
