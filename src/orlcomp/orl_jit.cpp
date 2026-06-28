#include "orl_jit.h"

#include "orl_optimizer.h"

#if __has_include(<llvm/ExecutionEngine/Orc/LLJIT.h>)

#include <llvm/ExecutionEngine/Orc/LLJIT.h>
#include <llvm/ExecutionEngine/Orc/ThreadSafeModule.h>
#include <llvm/IR/LLVMContext.h>
#include <llvm/IR/Module.h>
#include <llvm/Support/Error.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/Support/TargetSelect.h>

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

OrlJitTarget OrlJitEngine::Target() const {
    return impl_->target_kind_;
}

const std::vector<std::string> &OrlJitEngine::Errors() const {
    return impl_->errors_;
}

} // namespace orlcomp

#endif
