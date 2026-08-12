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

TEST_CASE("gpu dispatch defaults are configurable", "[orl][gpu][dispatch]") {
    OrlGpuEngine gpu(OrlGpuBackend::Cuda);

    REQUIRE(gpu.DefaultThreadsPerBlock() == 128);
    REQUIRE(gpu.SetDefaultThreadsPerBlock(64));
    REQUIRE(gpu.DefaultThreadsPerBlock() == 64);
    REQUIRE_FALSE(gpu.SetDefaultThreadsPerBlock(0));
    REQUIRE_FALSE(gpu.Errors().empty());
}

TEST_CASE("cuda backend compiles ORL snippet to PTX and runs kernel", "[orl][gpu][cuda]") {
    const std::string src =
        "int compute() {\n"
        "    int x = 7;\n"
        "    return x + 5;\n"
        "}\n";

    Parser parser(src);
    REQUIRE(parser.Parse());
    REQUIRE(parser.Ast() != nullptr);

    LlvmIrCodegen codegen("orl_gpu_pipeline_module", OrlCodegenTarget::Cuda);
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

    REQUIRE(gpu.SetupCudaKernel(OrlGpuEngine::CudaEntryKernelName));
    REQUIRE(gpu.SetDefaultThreadsPerBlock(64));
    REQUIRE(gpu.LaunchCudaKernelForElements(1));
    REQUIRE(gpu.Synchronize());
    std::int32_t result = 0;
    REQUIRE(gpu.ReadCudaGlobalInt32(OrlGpuEngine::CudaResultSymbolName, &result));
    REQUIRE(result == 12);
}

TEST_CASE("cuda parallel for transforms one buffer element per invocation", "[orl][gpu][cuda][parallel]") {
    const std::string src =
        "int add_work_item(int values[], int element_count) {\n"
        "    parallel for (int index = 0; index < element_count; index = index + 1) {\n"
        "        values[index] = values[index] + index;\n"
        "    }\n"
        "    return 0;\n"
        "}\n";

    Parser parser(src);
    REQUIRE(parser.Parse());

    LlvmIrCodegen codegen("orl_gpu_global_id_module", OrlCodegenTarget::Cuda);
    REQUIRE(codegen.Generate(*parser.Ast()));

    OrlGpuEngine gpu(OrlGpuBackend::Cuda);
    gpu.SetCudaEntryFunction("add_work_item");
    if (!gpu.CompileModule(codegen.ReleaseModule(), codegen.ReleaseContext())) {
        const auto &errors = gpu.Errors();
        WARN((errors.empty() ? "CUDA PTX compilation unavailable in this environment" : errors.back()));
        return;
    }
    if (!gpu.LoadToDriver()) {
        const auto &errors = gpu.Errors();
        WARN((errors.empty() ? "CUDA driver/module load unavailable in this environment" : errors.back()));
        return;
    }

    std::int64_t values[] = {3, 7, 11};
    std::int64_t result_values[3] = {};
    const auto buffer = gpu.AllocateBuffer(sizeof(values));
    REQUIRE(buffer.has_value());
    REQUIRE(gpu.UploadBuffer(*buffer, values, sizeof(values)));
    REQUIRE(gpu.SetupCudaKernel(OrlGpuEngine::CudaEntryKernelName, BindGpuBuffer(*buffer), std::int64_t{3}));
    REQUIRE(gpu.LaunchCudaKernelForElements(3));
    REQUIRE(gpu.Synchronize());
    REQUIRE(gpu.DownloadBuffer(*buffer, result_values, sizeof(result_values)));
    REQUIRE(gpu.FreeBuffer(*buffer));

    REQUIRE(result_values[0] == 3);
    REQUIRE(result_values[1] == 8);
    REQUIRE(result_values[2] == 13);
}

TEST_CASE("cuda entry reflects ORL parameters and rejects mismatched bindings", "[orl][gpu][cuda]") {
    const std::string src =
        "int deform(point input_positions[], point output_positions[], int vertex_count) {\n"
        "    output_positions[0] = input_positions[0];\n"
        "    return vertex_count;\n"
        "}\n";

    Parser parser(src);
    REQUIRE(parser.Parse());

    LlvmIrCodegen codegen("orl_gpu_reflection_module", OrlCodegenTarget::Cuda);
    REQUIRE(codegen.Generate(*parser.Ast()));

    OrlGpuEngine gpu(OrlGpuBackend::Cuda);
    gpu.SetCudaEntryFunction("deform");
    const bool compiled = gpu.CompileModule(codegen.ReleaseModule(), codegen.ReleaseContext());

    const auto &parameters = gpu.CudaEntryParameters();
    REQUIRE(parameters.size() == 3);
    REQUIRE(parameters[0].name == "input_positions");
    REQUIRE(parameters[0].type == OrlGpuKernelParameterType::Buffer);
    REQUIRE(parameters[1].name == "output_positions");
    REQUIRE(parameters[1].type == OrlGpuKernelParameterType::Buffer);
    REQUIRE(parameters[2].name == "vertex_count");
    REQUIRE(parameters[2].type == OrlGpuKernelParameterType::Int64);

    REQUIRE_FALSE(gpu.SetupCudaKernel(OrlGpuEngine::CudaEntryKernelName,
                                      BindGpuBuffer(1),
                                      std::int64_t{2},
                                      std::int64_t{1}));
    REQUIRE_FALSE(gpu.Errors().empty());

    if (!compiled) {
        WARN("CUDA PTX compilation unavailable in this environment");
    }
}

#endif
