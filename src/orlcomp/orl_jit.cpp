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

namespace orlcomp {

namespace {

std::string FormatLlvmError(const llvm::Error &error) {
    std::string message;
    llvm::raw_string_ostream stream(message);
    stream << error;
    return stream.str();
}

} // namespace

struct OrlJitEngine::Impl {
    Impl() {
        llvm::InitializeNativeTarget();
        llvm::InitializeNativeTargetAsmPrinter();
        llvm::InitializeNativeTargetAsmParser();
    }

    bool LoadModule(std::unique_ptr<llvm::Module> module, std::unique_ptr<llvm::LLVMContext> context) {
        errors_.clear();
        jit_.reset();

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
        const auto function = reinterpret_cast<FunctionType>(static_cast<uintptr_t>(symbol_or_error->getAddress()));
        return function();
    }

    std::optional<int64_t> InvokeInt64(const std::string &name, int64_t arg) {
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
        const auto function = reinterpret_cast<FunctionType>(static_cast<uintptr_t>(symbol_or_error->getAddress()));
        return function(arg);
    }

    std::unique_ptr<llvm::orc::LLJIT> jit_;
    std::vector<std::string> errors_;
};

OrlJitEngine::OrlJitEngine() : impl_(std::make_unique<Impl>()) {}
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

const std::vector<std::string> &OrlJitEngine::Errors() const {
    return impl_->errors_;
}

} // namespace orlcomp

#else

namespace orlcomp {

struct OrlJitEngine::Impl {
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

    std::vector<std::string> errors_;
};

OrlJitEngine::OrlJitEngine() : impl_(std::make_unique<Impl>()) {}
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

const std::vector<std::string> &OrlJitEngine::Errors() const {
    return impl_->errors_;
}

} // namespace orlcomp

#endif
