#if __has_include(<catch2/catch_all.hpp>)

#include <catch2/catch_all.hpp>

#include "orl_codegen.h"
#include "orl_gpu.h"
#include "orl_jit.h"
#include "orl_parser.h"

#include <llvm/IR/Constants.h>
#include <llvm/IR/IRBuilder.h>
#include <llvm/IR/LLVMContext.h>
#include <llvm/IR/Metadata.h>
#include <llvm/IR/Module.h>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

using namespace orlcomp;

namespace {

std::string LoadTextFileOrFail(const std::string &path) {
    std::ifstream in(path, std::ios::in);
    REQUIRE(in.is_open());

    std::stringstream buffer;
    buffer << in.rdbuf();
    REQUIRE(in.good() || in.eof());
    return buffer.str();
}

std::string LoadLbsOrlSource() {
    namespace fs = std::filesystem;
    const fs::path cwd = fs::current_path();
    const fs::path candidates[] = {
        cwd / "tests" / "skinning_tests" / "lbs.orl",
        cwd / ".." / ".." / "tests" / "skinning_tests" / "lbs.orl",
        cwd / ".." / ".." / ".." / "tests" / "skinning_tests" / "lbs.orl",
        fs::path("E:/repo/ORL/tests/skinning_tests/lbs.orl"),
    };

    for (const auto &candidate : candidates) {
        if (fs::exists(candidate)) {
            return LoadTextFileOrFail(candidate.string());
        }
    }

    FAIL("Unable to locate tests/skinning_tests/lbs.orl from current working directory");
}

bool AddKernelWrapperForCompute(llvm::Module *module) {
    if (module == nullptr) {
        return false;
    }

    llvm::Function *compute = module->getFunction("compute");
    if (compute == nullptr || compute->arg_size() != 0) {
        return false;
    }

    llvm::LLVMContext &context = module->getContext();
    auto *result_global = new llvm::GlobalVariable(*module,
                                                   llvm::Type::getInt32Ty(context),
                                                   false,
                                                   llvm::GlobalValue::ExternalLinkage,
                                                   llvm::ConstantInt::get(llvm::Type::getInt32Ty(context), 0),
                                                   "orl_skinning_result");

    auto *kernel_type = llvm::FunctionType::get(llvm::Type::getVoidTy(context), {}, false);
    llvm::Function *kernel =
        llvm::Function::Create(kernel_type, llvm::GlobalValue::ExternalLinkage, "orl_skinning_kernel", module);

    llvm::BasicBlock *entry = llvm::BasicBlock::Create(context, "entry", kernel);
    llvm::IRBuilder<> builder(entry);
    llvm::Value *result = builder.CreateCall(compute, {}, "compute.call");
    llvm::Value *result_i32 = result->getType()->isIntegerTy(32)
                                  ? result
                                  : builder.CreateTruncOrBitCast(result, llvm::Type::getInt32Ty(context), "result.i32");
    builder.CreateStore(result_i32, result_global);
    builder.CreateRetVoid();

    llvm::NamedMDNode *annotations = module->getOrInsertNamedMetadata("nvvm.annotations");
    llvm::Metadata *annotation_ops[] = {
        llvm::ValueAsMetadata::get(kernel),
        llvm::MDString::get(context, "kernel"),
        llvm::ConstantAsMetadata::get(llvm::ConstantInt::get(llvm::Type::getInt32Ty(context), 1)),
    };
    annotations->addOperand(llvm::MDNode::get(context, annotation_ops));
    return true;
}

constexpr std::int64_t kExpectedScaledLbsValue = 1800;

} // namespace

TEST_CASE("linear blend skinning ORL runs on CPU JIT", "[orl][skinning][cpu]") {
    const std::string src = LoadLbsOrlSource();

    Parser parser(src);
    REQUIRE(parser.Parse());
    REQUIRE(parser.Errors().empty());
    REQUIRE(parser.Ast() != nullptr);

    LlvmIrCodegen codegen("orl_skinning_cpu_module");
    REQUIRE(codegen.Generate(*parser.Ast()));
    REQUIRE(codegen.Errors().empty());

    OrlJitEngine jit(OrlJitTarget::Native);
    REQUIRE(jit.LoadModuleWithOptimization(codegen.ReleaseModule(),
                                           codegen.ReleaseContext(),
                                           OrlOptimizationLevel::O2));
    REQUIRE(jit.Errors().empty());

    const auto result = jit.InvokeInt64("compute");
    REQUIRE(result.has_value());
    REQUIRE(*result == kExpectedScaledLbsValue);
}

TEST_CASE("linear blend skinning ORL runs on CUDA path", "[orl][skinning][gpu][cuda]") {
    const std::string src = LoadLbsOrlSource();

    Parser parser(src);
    REQUIRE(parser.Parse());
    REQUIRE(parser.Errors().empty());
    REQUIRE(parser.Ast() != nullptr);

    LlvmIrCodegen codegen("orl_skinning_cuda_module");
    REQUIRE(codegen.Generate(*parser.Ast()));
    REQUIRE(codegen.Errors().empty());

    std::unique_ptr<llvm::Module> module = codegen.ReleaseModule();
    std::unique_ptr<llvm::LLVMContext> context = codegen.ReleaseContext();
    REQUIRE(module != nullptr);
    REQUIRE(context != nullptr);
    REQUIRE(AddKernelWrapperForCompute(module.get()));

    OrlGpuEngine gpu(OrlGpuBackend::Cuda);
    if (!gpu.CompileModuleWithOptimization(std::move(module),
                                           std::move(context),
                                           OrlOptimizationLevel::O2)) {
        const auto &errors = gpu.Errors();
        std::string reason = errors.empty() ? "CUDA PTX compilation unavailable in this environment" : errors.back();
        WARN(reason);
        return;
    }

    if (!gpu.LoadToDriver()) {
        const auto &errors = gpu.Errors();
        std::string reason = errors.empty() ? "CUDA driver/module load unavailable in this environment"
                                            : errors.back();
        WARN(reason);
        return;
    }

    REQUIRE(gpu.RunCudaKernelNoArgs("orl_skinning_kernel", 1, 1));
    std::int32_t result = 0;
    REQUIRE(gpu.ReadCudaGlobalInt32("orl_skinning_result", &result));
    REQUIRE(result == static_cast<std::int32_t>(kExpectedScaledLbsValue));
}

#endif
