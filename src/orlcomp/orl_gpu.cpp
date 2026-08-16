#include "orl_gpu.h"

#if __has_include(<llvm/MC/TargetRegistry.h>) && \
    __has_include(<llvm/Target/TargetMachine.h>) && \
    __has_include(<llvm/Support/CodeGen.h>)

#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/Constants.h>
#include <llvm/IR/DerivedTypes.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/GlobalVariable.h>
#include <llvm/IR/IRBuilder.h>
#include <llvm/IR/Instructions.h>
#include <llvm/IR/LegacyPassManager.h>
#include <llvm/IR/LLVMContext.h>
#include <llvm/IR/Metadata.h>
#include <llvm/IR/Module.h>
#include <llvm/IR/Type.h>
#include <llvm/MC/TargetRegistry.h>
#include <llvm/Support/Error.h>
#include <llvm/Support/TargetSelect.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/Target/TargetMachine.h>
#include <llvm/Transforms/Utils/Cloning.h>

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
#include <cstring>
#include <unordered_map>
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

    bool LowerCudaGlobalId(llvm::Module &module) {
        llvm::Function *global_id = module.getFunction("__orl_global_id");
        if (global_id == nullptr) {
            return true;
        }

        std::vector<llvm::CallInst *> calls;
        for (llvm::User *user : global_id->users()) {
            auto *call = llvm::dyn_cast<llvm::CallInst>(user);
            if (call == nullptr || call->getCalledFunction() != global_id || call->arg_size() != 0) {
                errors_.push_back("Invalid use of global_id builtin");
                return false;
            }
            calls.push_back(call);
        }

        const auto register_type = llvm::FunctionType::get(llvm::Type::getInt32Ty(module.getContext()), false);
        llvm::FunctionCallee thread_id = module.getOrInsertFunction("llvm.nvvm.read.ptx.sreg.tid.x", register_type);
        llvm::FunctionCallee block_id = module.getOrInsertFunction("llvm.nvvm.read.ptx.sreg.ctaid.x", register_type);
        llvm::FunctionCallee block_size = module.getOrInsertFunction("llvm.nvvm.read.ptx.sreg.ntid.x", register_type);
        for (llvm::CallInst *call : calls) {
            llvm::IRBuilder<> builder(call);
            llvm::Value *thread_id_i64 = builder.CreateZExt(builder.CreateCall(thread_id), builder.getInt64Ty(), "thread_id");
            llvm::Value *block_id_i64 = builder.CreateZExt(builder.CreateCall(block_id), builder.getInt64Ty(), "block_id");
            llvm::Value *block_size_i64 = builder.CreateZExt(builder.CreateCall(block_size), builder.getInt64Ty(), "block_size");
            llvm::Value *global_id_value = builder.CreateAdd(
                builder.CreateMul(block_id_i64, block_size_i64, "global_id_block_offset"),
                thread_id_i64,
                "global_id");
            call->replaceAllUsesWith(global_id_value);
            call->eraseFromParent();
        }
        global_id->eraseFromParent();
        return true;
    }

    bool LowerCudaEntryBufferAddressSpaces(llvm::Module &module) {
        llvm::Function *entry_function = module.getFunction(cuda_entry_function_);
        if (entry_function == nullptr) {
            errors_.push_back("CUDA compilation requires ORL entry function '" + cuda_entry_function_ + "'");
            return false;
        }
        // NVPTX accepts generic pointers in kernel parameters. Rebuilding the
        // entry function with address-space-one pointers leaves cloned uses
        // with incompatible opaque-pointer types and corrupts the module.
        // Keep the ABI pointer types unchanged until a complete address-space
        // cast lowering pass is implemented.
        return true;
    }

    bool CreateCudaEntryKernel(llvm::Module &module) {
        cuda_entry_parameters_.clear();
        llvm::Function *entry_function = module.getFunction(cuda_entry_function_);
        if (entry_function == nullptr) {
            errors_.push_back("CUDA compilation requires ORL entry function '" + cuda_entry_function_ + "'");
            return false;
        }
        if (!entry_function->getReturnType()->isIntegerTy()) {
            errors_.push_back("CUDA entry function '" + cuda_entry_function_ + "' must return an integer result");
            return false;
        }
        if (module.getFunction(OrlGpuEngine::CudaEntryKernelName) != nullptr ||
            module.getGlobalVariable(OrlGpuEngine::CudaResultSymbolName) != nullptr) {
            errors_.push_back("CUDA entry kernel or result symbol already exists in the module");
            return false;
        }

        llvm::LLVMContext &context = module.getContext();
        std::vector<llvm::Type *> parameter_types;
        parameter_types.reserve(entry_function->arg_size());
        for (llvm::Argument &argument : entry_function->args()) {
            OrlGpuKernelParameter parameter;
            parameter.name = argument.getName().str();
            if (argument.getType()->isPointerTy()) {
                parameter.type = OrlGpuKernelParameterType::Buffer;
            } else if (argument.getType()->isIntegerTy(64)) {
                parameter.type = OrlGpuKernelParameterType::Int64;
            } else if (argument.getType()->isDoubleTy()) {
                parameter.type = OrlGpuKernelParameterType::Float64;
            } else {
                parameter.type = OrlGpuKernelParameterType::Unsupported;
            }
            cuda_entry_parameters_.push_back(std::move(parameter));
            parameter_types.push_back(argument.getType());
        }

        auto *result_type = llvm::Type::getInt32Ty(context);
        auto *result_global = new llvm::GlobalVariable(module,
                                                        result_type,
                                                        false,
                                                        llvm::GlobalValue::ExternalLinkage,
                                                        llvm::ConstantInt::get(result_type, 0),
                                                        OrlGpuEngine::CudaResultSymbolName);

        auto *kernel_type = llvm::FunctionType::get(llvm::Type::getVoidTy(context), parameter_types, false);
        llvm::Function *kernel = llvm::Function::Create(kernel_type,
                                                        llvm::GlobalValue::ExternalLinkage,
                                                        OrlGpuEngine::CudaEntryKernelName,
                                                        &module);
        llvm::BasicBlock *entry = llvm::BasicBlock::Create(context, "entry", kernel);
        llvm::IRBuilder<> builder(entry);
        std::vector<llvm::Value *> kernel_arguments;
        kernel_arguments.reserve(kernel->arg_size());
        auto entry_argument = entry_function->arg_begin();
        for (llvm::Argument &argument : kernel->args()) {
            kernel_arguments.push_back(&argument);
            ++entry_argument;
        }
        llvm::Value *result = builder.CreateCall(entry_function, kernel_arguments, "compute.call");
        llvm::Value *result_i32 = result->getType()->isIntegerTy(32)
                                      ? result
                                      : builder.CreateTruncOrBitCast(result, result_type, "result.i32");
        builder.CreateStore(result_i32, result_global);
        builder.CreateRetVoid();

        llvm::NamedMDNode *annotations = module.getOrInsertNamedMetadata("nvvm.annotations");
        llvm::Metadata *annotation_ops[] = {
            llvm::ValueAsMetadata::get(kernel),
            llvm::MDString::get(context, "kernel"),
            llvm::ConstantAsMetadata::get(llvm::ConstantInt::get(result_type, 1)),
        };
        annotations->addOperand(llvm::MDNode::get(context, annotation_ops));
        return true;
    }

    bool CompileModule(std::unique_ptr<llvm::Module> incoming_module,
                       std::unique_ptr<llvm::LLVMContext> incoming_context) {
        // LLVM modules retain references to their owning context. Keep the
        // context declared before the module so the module is always destroyed
        // first, including every early-return path below.
        std::unique_ptr<llvm::LLVMContext> context = std::move(incoming_context);
        std::unique_ptr<llvm::Module> module = std::move(incoming_module);
        errors_.clear();
        device_code_.clear();
        UnloadDriverModule();

        const auto fail = [&]() {
            module.reset();
            context.reset();
            return false;
        };
        if (module == nullptr || context == nullptr) {
            errors_.push_back("CompileModule requires non-null LLVM module and context");
            return fail();
        }
        if (backend_ == OrlGpuBackend::Cuda) {
            if (!LowerCudaGlobalId(*module) ||
                !LowerCudaEntryBufferAddressSpaces(*module) ||
                !CreateCudaEntryKernel(*module)) {
                return fail();
            }
        } else if (module->getFunction("__orl_global_id") != nullptr) {
            errors_.push_back("global_id is currently supported only by the CUDA backend");
            return fail();
        }

        std::string target_error;
        const std::string target_triple = TargetTripleFor(backend_);
        const std::string target_cpu = TargetCpuFor(backend_);
        const llvm::Target *target = llvm::TargetRegistry::lookupTarget(target_triple, target_error);
        if (target == nullptr) {
            errors_.push_back("Failed to find LLVM target for " + std::string(BackendName(backend_)) + ": " + target_error);
            return fail();
        }

        const llvm::TargetOptions options;
        auto target_machine = std::unique_ptr<llvm::TargetMachine>(
            target->createTargetMachine(target_triple, target_cpu, "", options, std::nullopt, std::nullopt,
                                        llvm::CodeGenOptLevel::Aggressive));
        if (!target_machine) {
            errors_.push_back("Failed to create target machine for backend " + std::string(BackendName(backend_)));
            return fail();
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
            return fail();
        }

        pass_manager.run(*module);
        device_code_.assign(output_buffer.begin(), output_buffer.end());

        module_ = std::move(module);
        context_ = std::move(context);
        return true;
    }

    bool CompileModuleWithOptimization(std::unique_ptr<llvm::Module> incoming_module,
                                       std::unique_ptr<llvm::LLVMContext> incoming_context,
                                       OrlOptimizationLevel level) {
        std::unique_ptr<llvm::LLVMContext> context = std::move(incoming_context);
        std::unique_ptr<llvm::Module> module = std::move(incoming_module);
        errors_.clear();
        const auto fail = [&]() {
            module.reset();
            context.reset();
            return false;
        };
        if (module == nullptr || context == nullptr) {
            errors_.push_back("CompileModuleWithOptimization requires non-null LLVM module and context");
            return fail();
        }

        LlvmOptimizer optimizer(level);
        if (!optimizer.Optimize(*module)) {
            errors_ = optimizer.Errors();
            return fail();
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

        ReleaseCudaBuffers();
        bound_kernel_ = nullptr;
        bound_argument_values_.clear();
        bound_argument_pointers_.clear();

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

    struct CudaBuffer {
        CUdeviceptr address = 0;
        std::size_t bytes = 0;
    };

    std::optional<OrlGpuBuffer> AllocateCudaBuffer(std::size_t bytes) {
        if (bytes == 0) {
            errors_.push_back("AllocateBuffer requires a non-zero size");
            return std::nullopt;
        }

        CUdeviceptr address = 0;
        const CUresult rc = cuMemAlloc_(&address, bytes);
        if (rc != CUDA_SUCCESS) {
            AddCudaError("cuMemAlloc failed", rc);
            return std::nullopt;
        }

        const OrlGpuBuffer handle = next_cuda_buffer_++;
        cuda_buffers_.emplace(handle, CudaBuffer{address, bytes});
        return handle;
    }

    CudaBuffer *FindCudaBuffer(OrlGpuBuffer handle, std::size_t bytes) {
        const auto buffer = cuda_buffers_.find(handle);
        if (buffer == cuda_buffers_.end()) {
            errors_.push_back("Unknown GPU buffer handle");
            return nullptr;
        }
        if (bytes > buffer->second.bytes) {
            errors_.push_back("GPU buffer transfer exceeds allocation size");
            return nullptr;
        }
        return &buffer->second;
    }

    bool UploadCudaBuffer(OrlGpuBuffer handle, const void *source, std::size_t bytes) {
        if (source == nullptr) {
            errors_.push_back("UploadBuffer requires a non-null source");
            return false;
        }
        CudaBuffer *buffer = FindCudaBuffer(handle, bytes);
        if (buffer == nullptr) {
            return false;
        }
        const CUresult rc = cuMemcpyHtoD_(buffer->address, source, bytes);
        if (rc != CUDA_SUCCESS) {
            AddCudaError("cuMemcpyHtoD failed", rc);
            return false;
        }
        return true;
    }

    bool DownloadCudaBuffer(OrlGpuBuffer handle, void *destination, std::size_t bytes) {
        if (destination == nullptr) {
            errors_.push_back("DownloadBuffer requires a non-null destination");
            return false;
        }
        CudaBuffer *buffer = FindCudaBuffer(handle, bytes);
        if (buffer == nullptr) {
            return false;
        }
        const CUresult rc = cuMemcpyDtoH_(destination, buffer->address, bytes);
        if (rc != CUDA_SUCCESS) {
            AddCudaError("cuMemcpyDtoH failed", rc);
            return false;
        }
        return true;
    }

    bool FreeCudaBuffer(OrlGpuBuffer handle) {
        const auto buffer = cuda_buffers_.find(handle);
        if (buffer == cuda_buffers_.end()) {
            errors_.push_back("Unknown GPU buffer handle");
            return false;
        }
        const CUresult rc = cuMemFree_(buffer->second.address);
        if (rc != CUDA_SUCCESS) {
            AddCudaError("cuMemFree failed", rc);
            return false;
        }
        cuda_buffers_.erase(buffer);
        return true;
    }

    bool SynchronizeCudaContext() {
        const CUresult rc = cuCtxSynchronize_();
        if (rc != CUDA_SUCCESS) {
            AddCudaError("cuCtxSynchronize failed", rc);
            return false;
        }
        return true;
    }

    bool SetupCudaKernelArguments(const std::string &kernel_name,
                                  const std::vector<OrlGpuKernelArgument> &arguments) {
        CUfunction kernel = nullptr;
        const CUresult rc = cuModuleGetFunction_(&kernel, cuda_module_, kernel_name.c_str());
        if (rc != CUDA_SUCCESS) {
            AddCudaError("cuModuleGetFunction failed", rc);
            return false;
        }

        bound_argument_values_.clear();
        bound_argument_values_.reserve(arguments.size());
        for (const OrlGpuKernelArgument &argument : arguments) {
            if (argument.is_buffer) {
                const auto buffer = cuda_buffers_.find(argument.buffer);
                if (buffer == cuda_buffers_.end()) {
                    errors_.push_back("Unknown GPU buffer handle in CUDA kernel binding");
                    bound_argument_values_.clear();
                    return false;
                }

                std::vector<std::uint8_t> bytes(sizeof(CUdeviceptr));
                std::memcpy(bytes.data(), &buffer->second.address, sizeof(CUdeviceptr));
                bound_argument_values_.push_back(std::move(bytes));
            } else {
                bound_argument_values_.push_back(argument.scalar_bytes);
            }
        }

        bound_argument_pointers_.clear();
        bound_argument_pointers_.reserve(bound_argument_values_.size());
        for (auto &argument : bound_argument_values_) {
            bound_argument_pointers_.push_back(argument.data());
        }
        bound_kernel_ = kernel;
        return true;
    }

    bool ValidateCudaKernelArguments(const std::string &kernel_name,
                                     const std::vector<OrlGpuKernelArgument> &arguments) {
        if (kernel_name != OrlGpuEngine::CudaEntryKernelName) {
            return true;
        }
        if (arguments.size() != cuda_entry_parameters_.size()) {
            errors_.push_back("CUDA entry kernel expects " + std::to_string(cuda_entry_parameters_.size()) +
                              " arguments but received " + std::to_string(arguments.size()));
            return false;
        }

        for (std::size_t i = 0; i < arguments.size(); ++i) {
            const OrlGpuKernelArgument &argument = arguments[i];
            const OrlGpuKernelParameter &parameter = cuda_entry_parameters_[i];
            const bool matches = argument.is_buffer
                                     ? parameter.type == OrlGpuKernelParameterType::Buffer
                                     : argument.scalar_type == parameter.type;
            if (!matches) {
                const std::string name = parameter.name.empty() ? std::to_string(i) : parameter.name;
                errors_.push_back("CUDA kernel argument type mismatch for parameter '" + name + "'");
                return false;
            }
        }
        return true;
    }

    bool LaunchCudaKernel(std::uint32_t block_count, std::uint32_t threads_per_block) {
        if (bound_kernel_ == nullptr) {
            errors_.push_back("No CUDA kernel bindings configured");
            return false;
        }
        if (block_count == 0 || threads_per_block == 0) {
            errors_.push_back("block_count and threads_per_block must be greater than zero");
            return false;
        }

        const CUresult rc = cuLaunchKernel_(bound_kernel_,
                                            block_count,
                                            1,
                                            1,
                                            threads_per_block,
                                            1,
                                            1,
                                            0,
                                            nullptr,
                                            bound_argument_pointers_.empty() ? nullptr : bound_argument_pointers_.data(),
                                            nullptr);
        if (rc != CUDA_SUCCESS) {
            AddCudaError("cuLaunchKernel failed", rc);
            return false;
        }
        return true;
    }

    void ReleaseCudaBuffers() {
        if (cuMemFree_ != nullptr) {
            for (const auto &[handle, buffer] : cuda_buffers_) {
                (void)handle;
                cuMemFree_(buffer.address);
            }
        }
        cuda_buffers_.clear();
    }

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
    std::uint32_t default_threads_per_block_ = 128;
    std::string cuda_entry_function_ = "compute";
    std::vector<OrlGpuKernelParameter> cuda_entry_parameters_;
    std::unique_ptr<llvm::LLVMContext> context_;
    std::unique_ptr<llvm::Module> module_;
    std::string device_code_;
    std::vector<std::string> errors_;

#if ORL_HAS_CUDA_HEADERS
    void *cuda_library_ = nullptr;
    bool cuda_loaded_ = false;
    CUcontext cuda_context_ = nullptr;
    CUmodule cuda_module_ = nullptr;
    CUfunction bound_kernel_ = nullptr;

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
    OrlGpuBuffer next_cuda_buffer_ = 1;
    std::unordered_map<OrlGpuBuffer, CudaBuffer> cuda_buffers_;
    std::vector<std::vector<std::uint8_t>> bound_argument_values_;
    std::vector<void *> bound_argument_pointers_;
#endif
};

OrlGpuEngine::OrlGpuEngine(OrlGpuBackend backend) : impl_(std::make_unique<Impl>(backend)) {}
OrlGpuEngine::~OrlGpuEngine() = default;

void OrlGpuEngine::SetCudaEntryFunction(std::string function_name) {
    impl_->cuda_entry_function_ = std::move(function_name);
}

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

std::optional<OrlGpuBuffer> OrlGpuEngine::AllocateBuffer(std::size_t bytes) {
    impl_->errors_.clear();
    if (impl_->backend_ != OrlGpuBackend::Cuda) {
        impl_->errors_.push_back("AllocateBuffer currently requires the CUDA backend");
        return std::nullopt;
    }
#if ORL_HAS_CUDA_HEADERS
    if (!IsDriverModuleLoaded() && !LoadToDriver()) {
        return std::nullopt;
    }
    return impl_->AllocateCudaBuffer(bytes);
#else
    (void)bytes;
    impl_->errors_.push_back("CUDA headers not available; cannot allocate GPU buffers");
    return std::nullopt;
#endif
}

bool OrlGpuEngine::UploadBuffer(OrlGpuBuffer buffer, const void *source, std::size_t bytes) {
    impl_->errors_.clear();
    if (impl_->backend_ != OrlGpuBackend::Cuda) {
        impl_->errors_.push_back("UploadBuffer currently requires the CUDA backend");
        return false;
    }
#if ORL_HAS_CUDA_HEADERS
    if (!IsDriverModuleLoaded() && !LoadToDriver()) {
        return false;
    }
    return impl_->UploadCudaBuffer(buffer, source, bytes);
#else
    (void)buffer;
    (void)source;
    (void)bytes;
    impl_->errors_.push_back("CUDA headers not available; cannot upload GPU buffers");
    return false;
#endif
}

bool OrlGpuEngine::DownloadBuffer(OrlGpuBuffer buffer, void *destination, std::size_t bytes) {
    impl_->errors_.clear();
    if (impl_->backend_ != OrlGpuBackend::Cuda) {
        impl_->errors_.push_back("DownloadBuffer currently requires the CUDA backend");
        return false;
    }
#if ORL_HAS_CUDA_HEADERS
    if (!IsDriverModuleLoaded() && !LoadToDriver()) {
        return false;
    }
    return impl_->DownloadCudaBuffer(buffer, destination, bytes);
#else
    (void)buffer;
    (void)destination;
    (void)bytes;
    impl_->errors_.push_back("CUDA headers not available; cannot download GPU buffers");
    return false;
#endif
}

bool OrlGpuEngine::FreeBuffer(OrlGpuBuffer buffer) {
    impl_->errors_.clear();
    if (impl_->backend_ != OrlGpuBackend::Cuda) {
        impl_->errors_.push_back("FreeBuffer currently requires the CUDA backend");
        return false;
    }
#if ORL_HAS_CUDA_HEADERS
    if (!IsDriverModuleLoaded()) {
        impl_->errors_.push_back("CUDA driver module is not loaded");
        return false;
    }
    return impl_->FreeCudaBuffer(buffer);
#else
    (void)buffer;
    impl_->errors_.push_back("CUDA headers not available; cannot free GPU buffers");
    return false;
#endif
}

bool OrlGpuEngine::Synchronize() {
    impl_->errors_.clear();
    if (impl_->backend_ != OrlGpuBackend::Cuda) {
        impl_->errors_.push_back("Synchronize currently requires the CUDA backend");
        return false;
    }
#if ORL_HAS_CUDA_HEADERS
    if (!IsDriverModuleLoaded() && !LoadToDriver()) {
        return false;
    }
    return impl_->SynchronizeCudaContext();
#else
    impl_->errors_.push_back("CUDA headers not available; cannot synchronize GPU work");
    return false;
#endif
}

bool OrlGpuEngine::SetupCudaKernelArguments(const std::string &kernel_name,
                                            std::vector<OrlGpuKernelArgument> arguments) {
    impl_->errors_.clear();
    if (impl_->backend_ != OrlGpuBackend::Cuda) {
        impl_->errors_.push_back("SetupCudaKernel currently requires the CUDA backend");
        return false;
    }
#if ORL_HAS_CUDA_HEADERS
    if (!impl_->ValidateCudaKernelArguments(kernel_name, arguments)) {
        return false;
    }
    if (!IsDriverModuleLoaded() && !LoadToDriver()) {
        return false;
    }
    return impl_->SetupCudaKernelArguments(kernel_name, arguments);
#else
    (void)kernel_name;
    (void)arguments;
    impl_->errors_.push_back("CUDA headers not available; cannot bind CUDA kernel arguments");
    return false;
#endif
}

bool OrlGpuEngine::LaunchCudaKernel(std::uint32_t block_count, std::uint32_t threads_per_block) {
    impl_->errors_.clear();
    if (impl_->backend_ != OrlGpuBackend::Cuda) {
        impl_->errors_.push_back("LaunchCudaKernel currently requires the CUDA backend");
        return false;
    }
#if ORL_HAS_CUDA_HEADERS
    if (!IsDriverModuleLoaded() && !LoadToDriver()) {
        return false;
    }
    return impl_->LaunchCudaKernel(block_count, threads_per_block);
#else
    (void)block_count;
    (void)threads_per_block;
    impl_->errors_.push_back("CUDA headers not available; cannot launch CUDA kernels");
    return false;
#endif
}

bool OrlGpuEngine::SetDefaultThreadsPerBlock(std::uint32_t threads_per_block) {
    impl_->errors_.clear();
    if (threads_per_block == 0) {
        impl_->errors_.push_back("default threads_per_block must be greater than zero");
        return false;
    }
    impl_->default_threads_per_block_ = threads_per_block;
    return true;
}

std::uint32_t OrlGpuEngine::DefaultThreadsPerBlock() const {
    return impl_->default_threads_per_block_;
}

bool OrlGpuEngine::LaunchCudaKernelForElements(std::uint32_t element_count,
                                               std::uint32_t suggested_threads_per_block) {
    impl_->errors_.clear();
    if (element_count == 0) {
        impl_->errors_.push_back("element_count must be greater than zero");
        return false;
    }
    const std::uint32_t threads_per_block =
        suggested_threads_per_block == 0 ? impl_->default_threads_per_block_ : suggested_threads_per_block;
    const std::uint32_t block_count = (element_count - 1) / threads_per_block + 1;
    return LaunchCudaKernel(block_count, threads_per_block);
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

const std::vector<OrlGpuKernelParameter> &OrlGpuEngine::CudaEntryParameters() const {
    return impl_->cuda_entry_parameters_;
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
    std::uint32_t default_threads_per_block_ = 128;
    std::string cuda_entry_function_ = "compute";
    std::vector<OrlGpuKernelParameter> cuda_entry_parameters_;
    std::string device_code_;
    std::vector<std::string> errors_;
};

OrlGpuEngine::OrlGpuEngine(OrlGpuBackend backend) : impl_(std::make_unique<Impl>(backend)) {}
OrlGpuEngine::~OrlGpuEngine() = default;

void OrlGpuEngine::SetCudaEntryFunction(std::string function_name) {
    impl_->cuda_entry_function_ = std::move(function_name);
}

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

std::optional<OrlGpuBuffer> OrlGpuEngine::AllocateBuffer(std::size_t) {
    impl_->errors_.clear();
    impl_->errors_.push_back("GPU buffer allocation unavailable in this build");
    return std::nullopt;
}

bool OrlGpuEngine::UploadBuffer(OrlGpuBuffer, const void *, std::size_t) {
    impl_->errors_.clear();
    impl_->errors_.push_back("GPU buffer upload unavailable in this build");
    return false;
}

bool OrlGpuEngine::DownloadBuffer(OrlGpuBuffer, void *, std::size_t) {
    impl_->errors_.clear();
    impl_->errors_.push_back("GPU buffer download unavailable in this build");
    return false;
}

bool OrlGpuEngine::FreeBuffer(OrlGpuBuffer) {
    impl_->errors_.clear();
    impl_->errors_.push_back("GPU buffer release unavailable in this build");
    return false;
}

bool OrlGpuEngine::Synchronize() {
    impl_->errors_.clear();
    impl_->errors_.push_back("GPU synchronization unavailable in this build");
    return false;
}

bool OrlGpuEngine::SetupCudaKernelArguments(const std::string &,
                                            std::vector<OrlGpuKernelArgument>) {
    impl_->errors_.clear();
    impl_->errors_.push_back("CUDA kernel binding unavailable in this build");
    return false;
}

bool OrlGpuEngine::LaunchCudaKernel(std::uint32_t, std::uint32_t) {
    impl_->errors_.clear();
    impl_->errors_.push_back("CUDA kernel launch unavailable in this build");
    return false;
}

bool OrlGpuEngine::SetDefaultThreadsPerBlock(std::uint32_t threads_per_block) {
    impl_->errors_.clear();
    if (threads_per_block == 0) {
        impl_->errors_.push_back("default threads_per_block must be greater than zero");
        return false;
    }
    impl_->default_threads_per_block_ = threads_per_block;
    return true;
}

std::uint32_t OrlGpuEngine::DefaultThreadsPerBlock() const {
    return impl_->default_threads_per_block_;
}

bool OrlGpuEngine::LaunchCudaKernelForElements(std::uint32_t element_count,
                                               std::uint32_t suggested_threads_per_block) {
    impl_->errors_.clear();
    if (element_count == 0) {
        impl_->errors_.push_back("element_count must be greater than zero");
        return false;
    }
    const std::uint32_t threads_per_block =
        suggested_threads_per_block == 0 ? impl_->default_threads_per_block_ : suggested_threads_per_block;
    const std::uint32_t block_count = (element_count - 1) / threads_per_block + 1;
    return LaunchCudaKernel(block_count, threads_per_block);
}

OrlGpuBackend OrlGpuEngine::Backend() const {
    return impl_->backend_;
}

bool OrlGpuEngine::IsDriverModuleLoaded() const {
    return false;
}

const std::vector<OrlGpuKernelParameter> &OrlGpuEngine::CudaEntryParameters() const {
    return impl_->cuda_entry_parameters_;
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
