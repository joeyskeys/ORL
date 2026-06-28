#if __has_include(<catch2/catch_all.hpp>)

#include <catch2/catch_all.hpp>

#include "orl_codegen.h"
#include "orl_gpu.h"
#include "orl_parser.h"

#include <llvm/IR/Constants.h>
#include <llvm/IR/IRBuilder.h>
#include <llvm/IR/LLVMContext.h>
#include <llvm/IR/Metadata.h>
#include <llvm/IR/Module.h>

#include <string>
#include <vector>

using namespace orlcomp;

namespace {

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
                                                   "orl_gpu_result");

    auto *kernel_type = llvm::FunctionType::get(llvm::Type::getVoidTy(context), {}, false);
    llvm::Function *kernel =
        llvm::Function::Create(kernel_type, llvm::GlobalValue::ExternalLinkage, "orl_kernel_entry", module);

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

} // namespace

TEST_CASE("cuda backend compiles ORL snippet to PTX and runs kernel", "[orl][gpu][cuda]") {
    const std::string src =
        "int compute() {\n"
        "    int x = 7;\n"
        "    return x + 5;\n"
        "}\n";

    Parser parser(src);
    REQUIRE(parser.Parse());
    REQUIRE(parser.Ast() != nullptr);

    LlvmIrCodegen codegen("orl_gpu_pipeline_module");
    REQUIRE(codegen.Generate(*parser.Ast()));
    REQUIRE(codegen.Errors().empty());

    std::unique_ptr<llvm::Module> module = codegen.ReleaseModule();
    std::unique_ptr<llvm::LLVMContext> context = codegen.ReleaseContext();
    REQUIRE(module != nullptr);
    REQUIRE(context != nullptr);
    REQUIRE(AddKernelWrapperForCompute(module.get()));

    OrlGpuEngine gpu(OrlGpuBackend::Cuda);
    if (!gpu.CompileModule(std::move(module), std::move(context))) {
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

    REQUIRE(gpu.RunCudaKernelNoArgs("orl_kernel_entry", 1, 1));
    std::int32_t result = 0;
    REQUIRE(gpu.ReadCudaGlobalInt32("orl_gpu_result", &result));
    REQUIRE(result == 12);
}

#endif
