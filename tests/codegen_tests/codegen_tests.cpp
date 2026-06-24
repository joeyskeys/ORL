#if __has_include(<catch2/catch_all.hpp>)

#include <catch2/catch_all.hpp>

#include "orl_codegen.h"
#include "orl_jit.h"
#include "orl_optimizer.h"
#include "orl_parser.h"
#include <llvm/IR/LLVMContext.h>
#include <llvm/IR/Module.h>

using namespace orlcomp;

TEST_CASE("llvm codegen emits IR for arithmetic and control flow", "[orl][codegen]") {
    const std::string src =
        "int addloop(int n) {\n"
        "    int acc = 0;\n"
        "    int i = 0;\n"
        "    while (i < n) {\n"
        "        acc = acc + i;\n"
        "        i = i + 1;\n"
        "    }\n"
        "    return acc;\n"
        "}\n";

    Parser parser(src);
    REQUIRE(parser.Parse());
    REQUIRE(parser.Ast() != nullptr);

    LlvmIrCodegen codegen("orl_test_module");
    REQUIRE(codegen.Generate(*parser.Ast()));
    REQUIRE(codegen.Errors().empty());

    const std::string ir = codegen.DumpIR();
    REQUIRE(ir.find("define i64 @addloop") != std::string::npos);
    REQUIRE(ir.find("while.cond") != std::string::npos);
    REQUIRE(ir.find("while.body") != std::string::npos);
    REQUIRE(ir.find("ret i64") != std::string::npos);
}

TEST_CASE("llvm codegen lowers vector constructor and calls", "[orl][codegen]") {
    const std::string src =
        "float main() {\n"
        "    vector v1(1, 0, 0);\n"
        "    vector v2(0, 1, 1);\n"
        "    print(dot(v1, v2));\n"
        "    return dot(v1, v2);\n"
        "}\n";

    Parser parser(src);
    REQUIRE(parser.Parse());
    REQUIRE(parser.Ast() != nullptr);

    LlvmIrCodegen codegen("orl_test_module_vec");
    REQUIRE(codegen.Generate(*parser.Ast()));
    REQUIRE(codegen.Errors().empty());

    const std::string ir = codegen.DumpIR();
    REQUIRE(ir.find("<3 x double>") != std::string::npos);
    REQUIRE(ir.find("@dot(") != std::string::npos);
    REQUIRE(ir.find("@print(") != std::string::npos);
}

TEST_CASE("llvm optimizer runs default pipeline", "[orl][optimizer]") {
    const std::string src =
        "int addloop(int n) {\n"
        "    int acc = 0;\n"
        "    int i = 0;\n"
        "    while (i < n) {\n"
        "        acc = acc + i;\n"
        "        i = i + 1;\n"
        "    }\n"
        "    return acc;\n"
        "}\n";

    Parser parser(src);
    REQUIRE(parser.Parse());

    LlvmIrCodegen codegen("orl_opt_module");
    REQUIRE(codegen.Generate(*parser.Ast()));

    LlvmOptimizer optimizer(OrlOptimizationLevel::O2);
    llvm::Module *module = const_cast<llvm::Module *>(codegen.GetModule());
    REQUIRE(module != nullptr);
    REQUIRE(optimizer.Optimize(*module));
    REQUIRE(optimizer.Errors().empty());
    REQUIRE(codegen.DumpIR().find("define i64 @addloop") != std::string::npos);
}

TEST_CASE("orl jit executes optimized function", "[orl][jit]") {
    const std::string src =
        "int addloop(int n) {\n"
        "    int acc = 0;\n"
        "    int i = 0;\n"
        "    while (i < n) {\n"
        "        acc = acc + i;\n"
        "        i = i + 1;\n"
        "    }\n"
        "    return acc;\n"
        "}\n";

    Parser parser(src);
    REQUIRE(parser.Parse());

    LlvmIrCodegen codegen("orl_jit_module");
    REQUIRE(codegen.Generate(*parser.Ast()));

    OrlJitEngine jit;
    REQUIRE(jit.LoadModuleWithOptimization(codegen.ReleaseModule(),
                                           codegen.ReleaseContext(),
                                           OrlOptimizationLevel::O2));
    REQUIRE(jit.Errors().empty());

    const auto result = jit.InvokeInt64("addloop", 5);
    REQUIRE(result.has_value());
    REQUIRE(*result == 10);
}

TEST_CASE("orl jit exposes selectable target mode", "[orl][jit]") {
    OrlJitEngine native_jit(OrlJitTarget::Native);
    REQUIRE(native_jit.Target() == OrlJitTarget::Native);

    OrlJitEngine cuda_jit(OrlJitTarget::Cuda);
    REQUIRE(cuda_jit.Target() == OrlJitTarget::Cuda);

    OrlJitEngine rocm_jit(OrlJitTarget::Rocm);
    REQUIRE(rocm_jit.Target() == OrlJitTarget::Rocm);
}

#endif
