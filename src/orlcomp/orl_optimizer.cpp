#include "orl_optimizer.h"

#if __has_include(<llvm/IR/Module.h>)

#include <llvm/Analysis/CGSCCPassManager.h>
#include <llvm/Analysis/LoopAnalysisManager.h>
#include <llvm/IR/Module.h>
#include <llvm/IR/PassManager.h>
#include <llvm/IR/Verifier.h>
#include <llvm/Passes/PassBuilder.h>
#include <llvm/Support/raw_ostream.h>

namespace orlcomp {

namespace {

llvm::OptimizationLevel ToLlvmOptimizationLevel(OrlOptimizationLevel level) {
    switch (level) {
    case OrlOptimizationLevel::O0:
        return llvm::OptimizationLevel::O0;
    case OrlOptimizationLevel::O1:
        return llvm::OptimizationLevel::O1;
    case OrlOptimizationLevel::O2:
        return llvm::OptimizationLevel::O2;
    case OrlOptimizationLevel::O3:
        return llvm::OptimizationLevel::O3;
    }
    return llvm::OptimizationLevel::O2;
}

} // namespace

struct LlvmOptimizer::Impl {
    explicit Impl(OrlOptimizationLevel level) : level_(level) {}

    bool Optimize(llvm::Module &module) {
        errors_.clear();

        llvm::LoopAnalysisManager loop_analysis;
        llvm::FunctionAnalysisManager function_analysis;
        llvm::CGSCCAnalysisManager cgscc_analysis;
        llvm::ModuleAnalysisManager module_analysis;

        llvm::PassBuilder pass_builder;
        pass_builder.registerModuleAnalyses(module_analysis);
        pass_builder.registerCGSCCAnalyses(cgscc_analysis);
        pass_builder.registerFunctionAnalyses(function_analysis);
        pass_builder.registerLoopAnalyses(loop_analysis);
        pass_builder.crossRegisterProxies(loop_analysis, function_analysis, cgscc_analysis, module_analysis);

        llvm::ModulePassManager module_passes;
        if (level_ != OrlOptimizationLevel::O0) {
            module_passes.addPass(pass_builder.buildPerModuleDefaultPipeline(ToLlvmOptimizationLevel(level_)));
        }

        module_passes.run(module, module_analysis);

        if (llvm::verifyModule(module, &llvm::errs())) {
            errors_.push_back("LLVM verifier failed after optimization");
            return false;
        }

        return true;
    }

    OrlOptimizationLevel level_;
    std::vector<std::string> errors_;
};

LlvmOptimizer::LlvmOptimizer(OrlOptimizationLevel level) : impl_(std::make_unique<Impl>(level)) {}
LlvmOptimizer::~LlvmOptimizer() = default;

bool LlvmOptimizer::Optimize(llvm::Module &module) {
    return impl_->Optimize(module);
}

const std::vector<std::string> &LlvmOptimizer::Errors() const {
    return impl_->errors_;
}

} // namespace orlcomp

#else

namespace orlcomp {

struct LlvmOptimizer::Impl {
    explicit Impl(OrlOptimizationLevel) {}

    bool Optimize(llvm::Module &) {
        errors_.clear();
        errors_.push_back("LLVM headers are unavailable in this build environment");
        return false;
    }

    std::vector<std::string> errors_;
};

LlvmOptimizer::LlvmOptimizer(OrlOptimizationLevel level) : impl_(std::make_unique<Impl>(level)) {}
LlvmOptimizer::~LlvmOptimizer() = default;

bool LlvmOptimizer::Optimize(llvm::Module &module) {
    return impl_->Optimize(module);
}

const std::vector<std::string> &LlvmOptimizer::Errors() const {
    return impl_->errors_;
}

} // namespace orlcomp

#endif
