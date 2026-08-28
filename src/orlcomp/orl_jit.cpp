#include "orl_jit.h"

#include "orl_optimizer.h"
#include "orl_parallel_runtime.h"

#if __has_include(<llvm/ExecutionEngine/Orc/LLJIT.h>)

#include <llvm/ExecutionEngine/Orc/Core.h>
#include <llvm/ExecutionEngine/Orc/LLJIT.h>
#include <llvm/ExecutionEngine/Orc/Mangling.h>
#include <llvm/ExecutionEngine/Orc/ThreadSafeModule.h>
#include <llvm/IR/LLVMContext.h>
#include <llvm/IR/Module.h>
#include <llvm/Support/Error.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/Support/TargetSelect.h>
#if __has_include(<llvm/TargetParser/Host.h>)
#include <llvm/TargetParser/Host.h>
#else
#include <llvm/Support/Host.h>
#endif

#include <array>
#include <utility>
#include <optional>

namespace orlcomp {

namespace {

std::string FormatLlvmError(const llvm::Error &error) {
    std::string message;
    llvm::raw_string_ostream stream(message);
    stream << error;
    return stream.str();
}

bool IsGpuTarget(OrlJitTarget target) {
    return target == OrlJitTarget::Cuda || target == OrlJitTarget::Rocm;
}

const char *TargetName(OrlJitTarget target) {
    switch (target) {
    case OrlJitTarget::Native:
        return "native";
    case OrlJitTarget::Cuda:
        return "cuda";
    case OrlJitTarget::Rocm:
        return "rocm";
    default:
        return "unknown";
    }
}

} // namespace

struct OrlJitEngine::Impl {
    explicit Impl(OrlJitTarget target_kind) : target_kind_(target_kind) {
        if (target_kind_ == OrlJitTarget::Native) {
            llvm::InitializeNativeTarget();
            llvm::InitializeNativeTargetAsmPrinter();
            llvm::InitializeNativeTargetAsmParser();
            return;
        }

        // CUDA/ROCm selection is explicit. Runtime execution is still host-only
        // because this engine uses LLJIT.
        llvm::InitializeAllTargetInfos();
        llvm::InitializeAllTargets();
        llvm::InitializeAllTargetMCs();
        llvm::InitializeAllAsmPrinters();
        llvm::InitializeAllAsmParsers();
    }

    bool LoadModule(std::unique_ptr<llvm::Module> module, std::unique_ptr<llvm::LLVMContext> context) {
        errors_.clear();
        jit_.reset();

        if (IsGpuTarget(target_kind_)) {
            errors_.push_back(
                std::string("JIT target '") + TargetName(target_kind_) +
                "' requested, but OrlJitEngine currently executes only native host code via LLJIT");
            return false;
        }

        auto jit_or_error = llvm::orc::LLJITBuilder().create();
        if (!jit_or_error) {
            errors_.push_back("Failed to create LLJIT: " + FormatLlvmError(jit_or_error.takeError()));
            return false;
        }

        jit_ = std::move(*jit_or_error);
        if (module->getTargetTriple().empty()) {
            module->setTargetTriple(llvm::sys::getDefaultTargetTriple());
        }
        if (module->getDataLayout().getStringRepresentation().empty()) {
            module->setDataLayout(jit_->getDataLayout());
        }
        llvm::orc::MangleAndInterner mangle(jit_->getExecutionSession(), jit_->getDataLayout());
        llvm::orc::SymbolMap runtime_symbols;
        runtime_symbols[mangle("__orl_parallel_for")] = {
            llvm::orc::ExecutorAddr::fromPtr(&__orl_parallel_for),
            llvm::JITSymbolFlags::Exported,
        };
        if (auto error = jit_->getMainJITDylib().define(llvm::orc::absoluteSymbols(std::move(runtime_symbols)))) {
            errors_.push_back("Failed to register ORL parallel runtime: " + FormatLlvmError(std::move(error)));
            jit_.reset();
            return false;
        }
        llvm::orc::ThreadSafeModule thread_safe_module(std::move(module), std::move(context));
        if (auto error = jit_->addIRModule(std::move(thread_safe_module))) {
            errors_.push_back("Failed to add IR module to JIT: " + FormatLlvmError(std::move(error)));
            jit_.reset();
            return false;
        }

        return true;
    }

    bool LoadModuleWithOptimization(std::unique_ptr<llvm::Module> module,
                                    std::unique_ptr<llvm::LLVMContext> context,
                                    OrlOptimizationLevel level) {
        LlvmOptimizer optimizer(level);
        if (!optimizer.Optimize(*module)) {
            errors_ = optimizer.Errors();
            return false;
        }
        return LoadModule(std::move(module), std::move(context));
    }

    std::optional<int64_t> InvokeInt64(const std::string &name) {
        if (IsGpuTarget(target_kind_)) {
            errors_.push_back(std::string("InvokeInt64 is unsupported for JIT target '") +
                              TargetName(target_kind_) + "'");
            return std::nullopt;
        }
        if (jit_ == nullptr) {
            errors_.push_back("JIT engine has no loaded module");
            return std::nullopt;
        }

        auto symbol_or_error = jit_->lookup(name);
        if (!symbol_or_error) {
            errors_.push_back("Failed to lookup function '" + name + "': " + FormatLlvmError(symbol_or_error.takeError()));
            return std::nullopt;
        }

        using FunctionType = int64_t (*)();
        const auto function = symbol_or_error->toPtr<FunctionType>();
        return function();
    }

    std::optional<int64_t> InvokeInt64(const std::string &name, int64_t arg) {
        if (IsGpuTarget(target_kind_)) {
            errors_.push_back(std::string("InvokeInt64(name, arg) is unsupported for JIT target '") +
                              TargetName(target_kind_) + "'");
            return std::nullopt;
        }
        if (jit_ == nullptr) {
            errors_.push_back("JIT engine has no loaded module");
            return std::nullopt;
        }

        auto symbol_or_error = jit_->lookup(name);
        if (!symbol_or_error) {
            errors_.push_back("Failed to lookup function '" + name + "': " + FormatLlvmError(symbol_or_error.takeError()));
            return std::nullopt;
        }

        using FunctionType = int64_t (*)(int64_t);
        const auto function = symbol_or_error->toPtr<FunctionType>();
        return function(arg);
    }

    std::optional<int64_t> InvokeInt64WithBufferArgs(const std::string &name,
                                                      const std::array<void *, 5> &buffers,
                                                      int64_t scalar_arg) {
        if (IsGpuTarget(target_kind_)) {
            errors_.push_back(std::string("InvokeInt64WithBufferArgs is unsupported for JIT target '") +
                              TargetName(target_kind_) + "'");
            return std::nullopt;
        }
        if (jit_ == nullptr) {
            errors_.push_back("JIT engine has no loaded module");
            return std::nullopt;
        }

        auto symbol_or_error = jit_->lookup(name);
        if (!symbol_or_error) {
            errors_.push_back("Failed to lookup function '" + name + "': " + FormatLlvmError(symbol_or_error.takeError()));
            return std::nullopt;
        }

        using FunctionType = int64_t (*)(void *, void *, void *, void *, void *, int64_t);
        const auto function = symbol_or_error->toPtr<FunctionType>();
        return function(buffers[0], buffers[1], buffers[2], buffers[3], buffers[4], scalar_arg);
    }

    std::optional<int64_t> InvokeInt64WithRuntimeArgs(const std::string &name,
                                                       void *const *buffers,
                                                       const int64_t *integers,
                                                       const double *floats) {
        if (IsGpuTarget(target_kind_)) {
            errors_.push_back(std::string("InvokeInt64WithRuntimeArgs is unsupported for JIT target '") +
                              TargetName(target_kind_) + "'");
            return std::nullopt;
        }
        if (jit_ == nullptr) {
            errors_.push_back("JIT engine has no loaded module");
            return std::nullopt;
        }

        auto symbol_or_error = jit_->lookup(name);
        if (!symbol_or_error) {
            errors_.push_back("Failed to lookup function '" + name + "': " +
                              FormatLlvmError(symbol_or_error.takeError()));
            return std::nullopt;
        }

        using FunctionType = int64_t (*)(void *const *, const int64_t *, const double *);
        const auto function = symbol_or_error->toPtr<FunctionType>();
        return function(buffers, integers, floats);
    }

    std::unique_ptr<llvm::orc::LLJIT> jit_;
    OrlJitTarget target_kind_ = OrlJitTarget::Native;
    std::vector<std::string> errors_;
};

OrlJitEngine::OrlJitEngine(OrlJitTarget target) : impl_(std::make_unique<Impl>(target)) {}
OrlJitEngine::~OrlJitEngine() = default;

bool OrlJitEngine::LoadModule(std::unique_ptr<llvm::Module> module, std::unique_ptr<llvm::LLVMContext> context) {
    return impl_->LoadModule(std::move(module), std::move(context));
}

bool OrlJitEngine::LoadModuleWithOptimization(std::unique_ptr<llvm::Module> module,
                                               std::unique_ptr<llvm::LLVMContext> context,
                                               OrlOptimizationLevel level) {
    return impl_->LoadModuleWithOptimization(std::move(module), std::move(context), level);
}

std::optional<int64_t> OrlJitEngine::InvokeInt64(const std::string &name) {
    return impl_->InvokeInt64(name);
}

std::optional<int64_t> OrlJitEngine::InvokeInt64(const std::string &name, int64_t arg) {
    return impl_->InvokeInt64(name, arg);
}

std::optional<int64_t> OrlJitEngine::InvokeInt64WithBufferArgs(const std::string &name,
                                                                const std::array<void *, 5> &buffers,
                                                                int64_t scalar_arg) {
    return impl_->InvokeInt64WithBufferArgs(name, buffers, scalar_arg);
}

std::optional<int64_t> OrlJitEngine::InvokeInt64WithRuntimeArgs(const std::string &name,
                                                                 void *const *buffers,
                                                                 const int64_t *integers,
                                                                 const double *floats) {
    return impl_->InvokeInt64WithRuntimeArgs(name, buffers, integers, floats);
}

OrlJitTarget OrlJitEngine::Target() const {
    return impl_->target_kind_;
}

const std::vector<std::string> &OrlJitEngine::Errors() const {
    return impl_->errors_;
}

} // namespace orlcomp

#else

namespace orlcomp {

struct OrlJitEngine::Impl {
    explicit Impl(OrlJitTarget target_kind) : target_kind_(target_kind) {}

    bool LoadModule(std::unique_ptr<llvm::Module>, std::unique_ptr<llvm::LLVMContext>) {
        errors_.clear();
        errors_.push_back("LLVM JIT headers are unavailable in this build environment");
        return false;
    }

    bool LoadModuleWithOptimization(std::unique_ptr<llvm::Module>,
                                    std::unique_ptr<llvm::LLVMContext>,
                                    OrlOptimizationLevel) {
        return LoadModule(nullptr, nullptr);
    }

    std::optional<int64_t> InvokeInt64(const std::string &) {
        return std::nullopt;
    }

    std::optional<int64_t> InvokeInt64(const std::string &, int64_t) {
        return std::nullopt;
    }

    std::optional<int64_t> InvokeInt64WithBufferArgs(const std::string &,
                                                      const std::array<void *, 5> &,
                                                      int64_t) {
        return std::nullopt;
    }

    std::optional<int64_t> InvokeInt64WithRuntimeArgs(const std::string &,
                                                       void *const *,
                                                       const int64_t *,
                                                       const double *) {
        return std::nullopt;
    }

    OrlJitTarget target_kind_ = OrlJitTarget::Native;
    std::vector<std::string> errors_;
};

OrlJitEngine::OrlJitEngine(OrlJitTarget target) : impl_(std::make_unique<Impl>(target)) {}
OrlJitEngine::~OrlJitEngine() = default;

bool OrlJitEngine::LoadModule(std::unique_ptr<llvm::Module> module, std::unique_ptr<llvm::LLVMContext> context) {
    return impl_->LoadModule(std::move(module), std::move(context));
}

bool OrlJitEngine::LoadModuleWithOptimization(std::unique_ptr<llvm::Module> module,
                                               std::unique_ptr<llvm::LLVMContext> context,
                                               OrlOptimizationLevel level) {
    return impl_->LoadModuleWithOptimization(std::move(module), std::move(context), level);
}

std::optional<int64_t> OrlJitEngine::InvokeInt64(const std::string &name) {
    return impl_->InvokeInt64(name);
}

std::optional<int64_t> OrlJitEngine::InvokeInt64(const std::string &name, int64_t arg) {
    return impl_->InvokeInt64(name, arg);
}

std::optional<int64_t> OrlJitEngine::InvokeInt64WithBufferArgs(const std::string &name,
                                                                const std::array<void *, 5> &buffers,
                                                                int64_t scalar_arg) {
    return impl_->InvokeInt64WithBufferArgs(name, buffers, scalar_arg);
}

std::optional<int64_t> OrlJitEngine::InvokeInt64WithRuntimeArgs(const std::string &name,
                                                                 void *const *buffers,
                                                                 const int64_t *integers,
                                                                 const double *floats) {
    return impl_->InvokeInt64WithRuntimeArgs(name, buffers, integers, floats);
}

OrlJitTarget OrlJitEngine::Target() const {
    return impl_->target_kind_;
}

const std::vector<std::string> &OrlJitEngine::Errors() const {
    return impl_->errors_;
}

} // namespace orlcomp

#endif
