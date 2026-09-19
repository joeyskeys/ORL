#include "orl_jit.h"

#include "orl_optimizer.h"
#include "orl_parallel_runtime.h"

#if __has_include(<llvm/ExecutionEngine/Orc/LLJIT.h>)

#include <llvm/ExecutionEngine/ObjectCache.h>
#include <llvm/ExecutionEngine/Orc/Core.h>
#include <llvm/ExecutionEngine/Orc/CompileUtils.h>
#include <llvm/ExecutionEngine/Orc/LLJIT.h>
#include <llvm/ExecutionEngine/Orc/Mangling.h>
#include <llvm/ExecutionEngine/Orc/ThreadSafeModule.h>
#include <llvm/IR/LLVMContext.h>
#include <llvm/IR/Module.h>
#include <llvm/Support/Error.h>
#include <llvm/Support/MemoryBuffer.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/Support/TargetSelect.h>
#if __has_include(<llvm/TargetParser/Host.h>)
#include <llvm/TargetParser/Host.h>
#else
#include <llvm/Support/Host.h>
#endif

#include <array>
#include <optional>
#include <span>
#include <utility>

namespace orlcomp {

namespace {

std::string FormatLlvmError(const llvm::Error &error) {
    std::string message;
    llvm::raw_string_ostream stream(message);
    stream << error;
    return stream.str();
}

class FileObjectCache final : public llvm::ObjectCache {
public:
    explicit FileObjectCache(OrlBinaryCacheOptions options)
        : cache_(std::move(options))
    {
    }

    void notifyObjectCompiled(
        const llvm::Module* module, llvm::MemoryBufferRef object) override
    {
        if (module == nullptr) {
            return;
        }
        const auto bytes = object.getBuffer();
        cache_.save(
            OrlBinaryKind::CpuObject,
            module->getModuleIdentifier(),
            std::span<const std::uint8_t>(
                reinterpret_cast<const std::uint8_t*>(bytes.data()),
                bytes.size()));
    }

    std::unique_ptr<llvm::MemoryBuffer> getObject(
        const llvm::Module* module) override
    {
        if (module == nullptr) {
            return nullptr;
        }
        std::vector<std::uint8_t> bytes;
        if (!cache_.load(
                OrlBinaryKind::CpuObject,
                module->getModuleIdentifier(),
                bytes))
        {
            return nullptr;
        }
        const llvm::StringRef object(
            reinterpret_cast<const char*>(bytes.data()), bytes.size());
        return llvm::MemoryBuffer::getMemBufferCopy(
            object, module->getModuleIdentifier());
    }

private:
    OrlBinaryCache cache_;
};

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
    explicit Impl(
        OrlJitTarget target_kind, OrlBinaryCacheOptions cache_options)
        : target_kind_(target_kind)
        , cache_options_(std::move(cache_options))
    {
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

    bool CreateHostJit(bool enable_object_cache) {
        jit_.reset();
        object_cache_.reset();

        llvm::orc::LLJITBuilder builder;
        if (enable_object_cache && !cache_options_.directory.empty()) {
            object_cache_ = std::make_unique<FileObjectCache>(
                cache_options_);
            builder.setCompileFunctionCreator(
                [this](llvm::orc::JITTargetMachineBuilder target_machine)
                -> llvm::Expected<
                    std::unique_ptr<llvm::orc::IRCompileLayer::IRCompiler>>
                {
                    auto machine = target_machine.createTargetMachine();
                    if (!machine) {
                        return machine.takeError();
                    }
                    return std::make_unique<
                        llvm::orc::TMOwningSimpleCompiler>(
                            std::move(*machine), object_cache_.get());
                });
        }

        auto jit_or_error = builder.create();
        if (!jit_or_error) {
            errors_.push_back("Failed to create LLJIT: "
                + FormatLlvmError(jit_or_error.takeError()));
            return false;
        }

        jit_ = std::move(*jit_or_error);
        llvm::orc::MangleAndInterner mangle(
            jit_->getExecutionSession(), jit_->getDataLayout());
        llvm::orc::SymbolMap runtime_symbols;
        runtime_symbols[mangle("__orl_parallel_for")] = {
            llvm::orc::ExecutorAddr::fromPtr(&__orl_parallel_for),
            llvm::JITSymbolFlags::Exported,
        };
        if (auto error = jit_->getMainJITDylib().define(
                llvm::orc::absoluteSymbols(std::move(runtime_symbols))))
        {
            errors_.push_back("Failed to register ORL parallel runtime: "
                + FormatLlvmError(std::move(error)));
            jit_.reset();
            return false;
        }
        return true;
    }

    bool LoadModule(std::unique_ptr<llvm::Module> module,
        std::unique_ptr<llvm::LLVMContext> context,
        std::string cache_key) {
        errors_.clear();

        if (IsGpuTarget(target_kind_)) {
            errors_.push_back(
                std::string("JIT target '") + TargetName(target_kind_) +
                "' requested, but OrlJitEngine currently executes only native host code via LLJIT");
            return false;
        }

        if (module == nullptr || context == nullptr) {
            errors_.push_back("LoadModule requires a non-null LLVM module and context");
            return false;
        }

        if (!cache_key.empty()) {
            module->setModuleIdentifier(std::move(cache_key));
        }

        if (!CreateHostJit(true)) {
            return false;
        }

        if (module->getTargetTriple().empty()) {
            module->setTargetTriple(llvm::sys::getDefaultTargetTriple());
        }
        if (module->getDataLayout().getStringRepresentation().empty()) {
            module->setDataLayout(jit_->getDataLayout());
        }
        llvm::orc::ThreadSafeModule thread_safe_module(std::move(module), std::move(context));
        if (auto error = jit_->addIRModule(std::move(thread_safe_module))) {
            errors_.push_back("Failed to add IR module to JIT: " + FormatLlvmError(std::move(error)));
            jit_.reset();
            return false;
        }

        return true;
    }

    bool LoadObject(std::span<const std::uint8_t> object) {
        errors_.clear();

        if (IsGpuTarget(target_kind_)) {
            errors_.push_back(
                std::string("JIT target '") + TargetName(target_kind_) +
                "' requested, but OrlJitEngine currently executes only native host code via LLJIT");
            return false;
        }
        if (object.empty()) {
            errors_.push_back("Cached CPU object is empty");
            return false;
        }
        if (!CreateHostJit(false)) {
            return false;
        }

        auto buffer = llvm::MemoryBuffer::getMemBufferCopy(
            llvm::StringRef(
                reinterpret_cast<const char*>(object.data()),
                object.size()),
            "orl_cached_object");
        if (auto error = jit_->addObjectFile(std::move(buffer))) {
            errors_.push_back("Failed to load cached CPU object: "
                + FormatLlvmError(std::move(error)));
            jit_.reset();
            return false;
        }
        return true;
    }

    bool LoadModuleWithOptimization(std::unique_ptr<llvm::Module> module,
                                    std::unique_ptr<llvm::LLVMContext> context,
                                    OrlOptimizationLevel level,
                                    std::string cache_key) {
        LlvmOptimizer optimizer(level);
        if (!optimizer.Optimize(*module)) {
            errors_ = optimizer.Errors();
            return false;
        }
        return LoadModule(
            std::move(module), std::move(context), std::move(cache_key));
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

    std::optional<int64_t> InvokeInt64WithRuntimeArgsAndContext(
        const std::string &name,
        void *const *buffers,
        const int64_t *integers,
        const double *floats,
        void *solver_context) {
        if (IsGpuTarget(target_kind_)) {
            errors_.push_back(std::string(
                "InvokeInt64WithRuntimeArgsAndContext is unsupported for JIT target '")
                + TargetName(target_kind_) + "'");
            return std::nullopt;
        }
        if (jit_ == nullptr) {
            errors_.push_back("JIT engine has no loaded module");
            return std::nullopt;
        }

        auto symbol_or_error = jit_->lookup(name);
        if (!symbol_or_error) {
            errors_.push_back("Failed to lookup function '" + name + "': "
                + FormatLlvmError(symbol_or_error.takeError()));
            return std::nullopt;
        }

        using FunctionType = int64_t (*)(
            void *const *, const int64_t *, const double *, void *);
        const auto function = symbol_or_error->toPtr<FunctionType>();
        return function(buffers, integers, floats, solver_context);
    }

    std::optional<int64_t> InvokeInt64WithRuntimeArgsAndHierarchyContext(
        const std::string &name,
        void *const *buffers,
        const int64_t *integers,
        const double *floats,
        void *hierarchy_context,
        void *hierarchy_data) {
        if (IsGpuTarget(target_kind_)) {
            errors_.push_back(std::string(
                "InvokeInt64WithRuntimeArgsAndHierarchyContext is unsupported for JIT target '")
                + TargetName(target_kind_) + "'");
            return std::nullopt;
        }
        if (jit_ == nullptr) {
            errors_.push_back("JIT engine has no loaded module");
            return std::nullopt;
        }

        auto symbol_or_error = jit_->lookup(name);
        if (!symbol_or_error) {
            errors_.push_back("Failed to lookup function '" + name + "': "
                + FormatLlvmError(symbol_or_error.takeError()));
            return std::nullopt;
        }

        using FunctionType = int64_t (*)(
            void *const *, const int64_t *, const double *, void *, void *);
        const auto function = symbol_or_error->toPtr<FunctionType>();
        return function(
            buffers, integers, floats, hierarchy_context, hierarchy_data);
    }

    std::optional<int64_t> InvokeInt64WithRuntimeArgsAndContexts(
        const std::string &name,
        void *const *buffers,
        const int64_t *integers,
        const double *floats,
        void *solver_context,
        void *hierarchy_context,
        void *hierarchy_data) {
        if (IsGpuTarget(target_kind_)) {
            errors_.push_back(std::string(
                "InvokeInt64WithRuntimeArgsAndContexts is unsupported for JIT target '")
                + TargetName(target_kind_) + "'");
            return std::nullopt;
        }
        if (jit_ == nullptr) {
            errors_.push_back("JIT engine has no loaded module");
            return std::nullopt;
        }

        auto symbol_or_error = jit_->lookup(name);
        if (!symbol_or_error) {
            errors_.push_back("Failed to lookup function '" + name + "': "
                + FormatLlvmError(symbol_or_error.takeError()));
            return std::nullopt;
        }

        using FunctionType = int64_t (*)(
            void *const *, const int64_t *, const double *,
            void *, void *, void *);
        const auto function = symbol_or_error->toPtr<FunctionType>();
        return function(
            buffers, integers, floats, solver_context,
            hierarchy_context, hierarchy_data);
    }

    OrlJitTarget target_kind_ = OrlJitTarget::Native;
    OrlBinaryCacheOptions cache_options_;
    std::unique_ptr<llvm::ObjectCache> object_cache_;
    std::unique_ptr<llvm::orc::LLJIT> jit_;
    std::vector<std::string> errors_;
};

OrlJitEngine::OrlJitEngine(
    OrlJitTarget target, OrlBinaryCacheOptions cache)
    : impl_(std::make_unique<Impl>(target, std::move(cache)))
{
}
OrlJitEngine::~OrlJitEngine() = default;

bool OrlJitEngine::LoadModule(
    std::unique_ptr<llvm::Module> module,
    std::unique_ptr<llvm::LLVMContext> context,
    std::string cache_key)
{
    return impl_->LoadModule(
        std::move(module), std::move(context), std::move(cache_key));
}

bool OrlJitEngine::LoadModuleWithOptimization(std::unique_ptr<llvm::Module> module,
                                               std::unique_ptr<llvm::LLVMContext> context,
                                               OrlOptimizationLevel level,
                                               std::string cache_key) {
    return impl_->LoadModuleWithOptimization(
        std::move(module), std::move(context), level, std::move(cache_key));
}

bool OrlJitEngine::LoadObject(std::span<const std::uint8_t> object) {
    return impl_->LoadObject(object);
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

std::optional<int64_t> OrlJitEngine::InvokeInt64WithRuntimeArgsAndContext(
    const std::string &name,
    void *const *buffers,
    const int64_t *integers,
    const double *floats,
    void *solver_context) {
    return impl_->InvokeInt64WithRuntimeArgsAndContext(
        name, buffers, integers, floats, solver_context);
}

std::optional<int64_t> OrlJitEngine::InvokeInt64WithRuntimeArgsAndHierarchyContext(
    const std::string &name,
    void *const *buffers,
    const int64_t *integers,
    const double *floats,
    void *hierarchy_context,
    void *hierarchy_data) {
    return impl_->InvokeInt64WithRuntimeArgsAndHierarchyContext(
        name, buffers, integers, floats, hierarchy_context, hierarchy_data);
}

std::optional<int64_t> OrlJitEngine::InvokeInt64WithRuntimeArgsAndContexts(
    const std::string &name,
    void *const *buffers,
    const int64_t *integers,
    const double *floats,
    void *solver_context,
    void *hierarchy_context,
    void *hierarchy_data) {
    return impl_->InvokeInt64WithRuntimeArgsAndContexts(
        name, buffers, integers, floats, solver_context,
        hierarchy_context, hierarchy_data);
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
    explicit Impl(
        OrlJitTarget target_kind, OrlBinaryCacheOptions)
        : target_kind_(target_kind) {}

    bool LoadModule(std::unique_ptr<llvm::Module>,
                    std::unique_ptr<llvm::LLVMContext>,
                    std::string) {
        errors_.clear();
        errors_.push_back("LLVM JIT headers are unavailable in this build environment");
        return false;
    }

    bool LoadModuleWithOptimization(std::unique_ptr<llvm::Module>,
                                    std::unique_ptr<llvm::LLVMContext>,
                                    OrlOptimizationLevel,
                                    std::string) {
        return LoadModule(nullptr, nullptr, {});
    }

    bool LoadObject(std::span<const std::uint8_t>) {
        errors_.clear();
        errors_.push_back("LLVM JIT headers are unavailable in this build environment");
        return false;
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

    std::optional<int64_t> InvokeInt64WithRuntimeArgsAndContext(
        const std::string &,
        void *const *,
        const int64_t *,
        const double *,
        void *) {
        return std::nullopt;
    }

    std::optional<int64_t> InvokeInt64WithRuntimeArgsAndHierarchyContext(
        const std::string &,
        void *const *,
        const int64_t *,
        const double *,
        void *,
        void *) {
        return std::nullopt;
    }

    std::optional<int64_t> InvokeInt64WithRuntimeArgsAndContexts(
        const std::string &,
        void *const *,
        const int64_t *,
        const double *,
        void *,
        void *,
        void *) {
        return std::nullopt;
    }

    OrlJitTarget target_kind_ = OrlJitTarget::Native;
    std::vector<std::string> errors_;
};

OrlJitEngine::OrlJitEngine(
    OrlJitTarget target, OrlBinaryCacheOptions cache)
    : impl_(std::make_unique<Impl>(target, std::move(cache)))
{
}
OrlJitEngine::~OrlJitEngine() = default;

bool OrlJitEngine::LoadModule(
    std::unique_ptr<llvm::Module> module,
    std::unique_ptr<llvm::LLVMContext> context,
    std::string cache_key) {
    return impl_->LoadModule(
        std::move(module), std::move(context), std::move(cache_key));
}

bool OrlJitEngine::LoadModuleWithOptimization(std::unique_ptr<llvm::Module> module,
                                               std::unique_ptr<llvm::LLVMContext> context,
                                               OrlOptimizationLevel level,
                                               std::string cache_key) {
    return impl_->LoadModuleWithOptimization(
        std::move(module), std::move(context), level, std::move(cache_key));
}

bool OrlJitEngine::LoadObject(std::span<const std::uint8_t> object) {
    return impl_->LoadObject(object);
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

std::optional<int64_t> OrlJitEngine::InvokeInt64WithRuntimeArgsAndContext(
    const std::string &name,
    void *const *buffers,
    const int64_t *integers,
    const double *floats,
    void *solver_context) {
    return impl_->InvokeInt64WithRuntimeArgsAndContext(
        name, buffers, integers, floats, solver_context);
}

std::optional<int64_t> OrlJitEngine::InvokeInt64WithRuntimeArgsAndHierarchyContext(
    const std::string &name,
    void *const *buffers,
    const int64_t *integers,
    const double *floats,
    void *hierarchy_context,
    void *hierarchy_data) {
    return impl_->InvokeInt64WithRuntimeArgsAndHierarchyContext(
        name, buffers, integers, floats, hierarchy_context, hierarchy_data);
}

std::optional<int64_t> OrlJitEngine::InvokeInt64WithRuntimeArgsAndContexts(
    const std::string &name,
    void *const *buffers,
    const int64_t *integers,
    const double *floats,
    void *solver_context,
    void *hierarchy_context,
    void *hierarchy_data) {
    return impl_->InvokeInt64WithRuntimeArgsAndContexts(
        name, buffers, integers, floats, solver_context,
        hierarchy_context, hierarchy_data);
}

OrlJitTarget OrlJitEngine::Target() const {
    return impl_->target_kind_;
}

const std::vector<std::string> &OrlJitEngine::Errors() const {
    return impl_->errors_;
}

} // namespace orlcomp

#endif
