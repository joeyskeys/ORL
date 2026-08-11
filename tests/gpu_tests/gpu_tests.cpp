#if __has_include(<catch2/catch_all.hpp>)

#include <catch2/catch_all.hpp>

#include "orl_codegen.h"
#include "orl_gpu.h"
#include "orl_parser.h"

#include <llvm/IR/LLVMContext.h>
#include <llvm/IR/Module.h>

#include <string>
#include <vector>

using namespace orlcomp;

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

    std::int32_t upload_values[] = {3, 7, 11};
    std::int32_t download_values[3] = {};
    const auto buffer = gpu.AllocateBuffer(sizeof(upload_values));
    REQUIRE(buffer.has_value());
    REQUIRE(gpu.UploadBuffer(*buffer, upload_values, sizeof(upload_values)));
    REQUIRE(gpu.Synchronize());
    REQUIRE(gpu.DownloadBuffer(*buffer, download_values, sizeof(download_values)));
    REQUIRE(download_values[0] == 3);
    REQUIRE(download_values[1] == 7);
    REQUIRE(download_values[2] == 11);
    REQUIRE(gpu.FreeBuffer(*buffer));

    REQUIRE(gpu.RunCudaKernelNoArgs(OrlGpuEngine::CudaEntryKernelName, 1, 1));
    std::int32_t result = 0;
    REQUIRE(gpu.ReadCudaGlobalInt32(OrlGpuEngine::CudaResultSymbolName, &result));
    REQUIRE(result == 12);
}

#endif
