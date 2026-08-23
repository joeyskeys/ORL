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

TEST_CASE("llvm CUDA codegen lowers parallel for", "[orl][codegen][cuda][parallel]") {
    const std::string src =
        "int deform(int values[], int count) {\n"
        "    parallel for (int index = 0; index < count; index = index + 1) {\n"
        "        values[index] = values[index] + 1;\n"
        "    }\n"
        "    return count;\n"
        "}\n";

    Parser parser(src);
    REQUIRE(parser.Parse());

    LlvmIrCodegen codegen("orl_cuda_parallel_module", OrlCodegenTarget::Cuda);
    REQUIRE(codegen.Generate(*parser.Ast()));
    REQUIRE(codegen.Errors().empty());

    const std::string ir = codegen.DumpIR();
    REQUIRE(ir.find("__orl_global_id") != std::string::npos);
    REQUIRE(ir.find("parallel.body") != std::string::npos);
}

TEST_CASE("llvm CUDA codegen lowers skinning parallel for", "[orl][codegen][cuda][parallel][skinning]") {
    const std::string src =
        "int deform(point input_positions[], point output_positions[], matrix bone_matrices[], "
        "float weights[], int bone_indices[], int vertex_count) {\n"
        "    parallel for (int vertex = 0; vertex < vertex_count; vertex = vertex + 1) {\n"
        "        int influence = vertex * 2;\n"
        "        int bone0 = bone_indices[influence];\n"
        "        int bone1 = bone_indices[influence + 1];\n"
        "        float weight0 = weights[influence];\n"
        "        float weight1 = weights[influence + 1];\n"
        "        point bind_position = input_positions[vertex];\n"
        "        point transformed0 = bone_matrices[bone0] * bind_position;\n"
        "        point transformed1 = bone_matrices[bone1] * bind_position;\n"
        "        output_positions[vertex] = weight0 * transformed0 + weight1 * transformed1;\n"
        "    }\n"
        "    return vertex_count;\n"
        "}\n";

    Parser parser(src);
    REQUIRE(parser.Parse());

    LlvmIrCodegen codegen("orl_lbs_deformer_cuda_module", OrlCodegenTarget::Cuda);
    REQUIRE(codegen.Generate(*parser.Ast()));
    REQUIRE(codegen.Errors().empty());
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
    REQUIRE(ir.find("dotmul") != std::string::npos);
    REQUIRE(ir.find("@print(") != std::string::npos);
}

TEST_CASE("llvm codegen lowers vector math intrinsics", "[orl][codegen][intrinsic]") {
    const std::string src =
        "float vector_intrinsics() {\n"
        "    vector x_axis(1, 0, 0);\n"
        "    vector y_axis(0, 1, 0);\n"
        "    vector perpendicular = cross(x_axis, y_axis);\n"
        "    vector unit = normalize(perpendicular);\n"
        "    vector limited = clamp(unit, -0.5, 0.5);\n"
        "    vector blended = lerp(x_axis, y_axis, 0.25);\n"
        "    float magnitude = length(limited);\n"
        "    return lerp(dot(limited, x_axis) + clamp(magnitude, 0.0, 1.0), blended.x, 0.5);\n"
        "}\n";

    Parser parser(src);
    REQUIRE(parser.Parse());

    LlvmIrCodegen codegen("orl_vector_intrinsic_module");
    REQUIRE(codegen.Generate(*parser.Ast()));
    REQUIRE(codegen.Errors().empty());

    const std::string ir = codegen.DumpIR();
    REQUIRE(ir.find("crossx") != std::string::npos);
    REQUIRE(ir.find("vecnormalize") != std::string::npos);
    REQUIRE(ir.find("veclength") != std::string::npos);
    REQUIRE(ir.find("clamp") != std::string::npos);
    REQUIRE(ir.find("lerp") != std::string::npos);
}

TEST_CASE("llvm codegen preserves global_id for GPU lowering", "[orl][codegen][intrinsic][gpu]") {
    const std::string src =
        "int work_item() {\n"
        "    return global_id();\n"
        "}\n";

    Parser parser(src);
    REQUIRE(parser.Parse());

    LlvmIrCodegen codegen("orl_global_id_module");
    REQUIRE(codegen.Generate(*parser.Ast()));
    REQUIRE(codegen.Errors().empty());

    const std::string ir = codegen.DumpIR();
    REQUIRE(ir.find("call i64 @__orl_global_id()") != std::string::npos);
    REQUIRE(ir.find("declare i64 @__orl_global_id()") != std::string::npos);
}

TEST_CASE("llvm codegen rejects global_id arguments", "[orl][codegen][intrinsic][gpu]") {
    const std::string src =
        "int invalid_work_item() {\n"
        "    return global_id(1);\n"
        "}\n";

    Parser parser(src);
    REQUIRE(parser.Parse());

    LlvmIrCodegen codegen("orl_invalid_global_id_module");
    REQUIRE_FALSE(codegen.Generate(*parser.Ast()));
    REQUIRE_FALSE(codegen.Errors().empty());
}

TEST_CASE("llvm codegen lowers parallel for for host and CUDA targets", "[orl][codegen][parallel]") {
    const std::string src =
        "int transform(int values[], int count) {\n"
        "    parallel for (int index = 0; index < count; index = index + 1) {\n"
        "        values[index] = values[index] + index;\n"
        "    }\n"
        "    return count;\n"
        "}\n";

    Parser parser(src);
    REQUIRE(parser.Parse());

    LlvmIrCodegen host_codegen("orl_parallel_host_module");
    REQUIRE(host_codegen.Generate(*parser.Ast()));
    const std::string host_ir = host_codegen.DumpIR();
    REQUIRE(host_ir.find("@__orl_parallel_for") != std::string::npos);
    REQUIRE(host_ir.find("@orl.parallel.body.0") != std::string::npos);
    REQUIRE(host_ir.find("parallel.for.inc") == std::string::npos);
    REQUIRE(host_ir.find("__orl_global_id") == std::string::npos);

    LlvmIrCodegen cuda_codegen("orl_parallel_cuda_module", OrlCodegenTarget::Cuda);
    REQUIRE(cuda_codegen.Generate(*parser.Ast()));
    const std::string cuda_ir = cuda_codegen.DumpIR();
    REQUIRE(cuda_ir.find("@__orl_global_id") != std::string::npos);
    REQUIRE(cuda_ir.find("parallel.for.inc") == std::string::npos);
    REQUIRE(cuda_ir.find("parallel.body") != std::string::npos);
}

TEST_CASE("llvm codegen lowers matrix math intrinsics", "[orl][codegen][intrinsic][matrix]") {
    const std::string src =
        "vector matrix_intrinsics() {\n"
        "    matrix identity = mat_identity();\n"
        "    matrix translation(1, 0, 0, 2,\n"
        "                       0, 1, 0, 3,\n"
        "                       0, 0, 1, 4,\n"
        "                       0, 0, 0, 1);\n"
        "    matrix combined = mat_mul(identity, translation);\n"
        "    matrix transpose = mat_transpose(combined);\n"
        "    matrix inverse = mat_inverse(transpose);\n"
        "    vector position(1, 2, 3);\n"
        "    return mat_mul(inverse, position);\n"
        "}\n";

    Parser parser(src);
    REQUIRE(parser.Parse());

    LlvmIrCodegen codegen("orl_matrix_intrinsic_module");
    REQUIRE(codegen.Generate(*parser.Ast()));
    REQUIRE(codegen.Errors().empty());

    const std::string ir = codegen.DumpIR();
    REQUIRE(ir.find("[16 x double]") != std::string::npos);
    REQUIRE(ir.find("matmul") != std::string::npos);
    REQUIRE(ir.find("mattranspose") != std::string::npos);
    REQUIRE(ir.find("matinverse") != std::string::npos);
    REQUIRE(ir.find("matvecmul") != std::string::npos);
}

TEST_CASE("llvm codegen lowers custom structs and struct buffers", "[orl][codegen][struct]") {
    const std::string src =
        "struct Vertex {\n"
        "    point position;\n"
        "    float weight;\n"
        "}\n"
        "Vertex copy_vertex(Vertex source) {\n"
        "    Vertex copy = source;\n"
        "    copy.weight = source.weight + 0.25;\n"
        "    return copy;\n"
        "}\n"
        "float update_vertices(Vertex vertices[]) {\n"
        "    point origin(0, 0, 0);\n"
        "    Vertex local(origin, 0.5);\n"
        "    Vertex cache[2];\n"
        "    cache[0] = local;\n"
        "    vertices[0].weight = local.weight;\n"
        "    return cache[0].position.x + vertices[0].position.x;\n"
        "}\n";

    Parser parser(src);
    REQUIRE(parser.Parse());
    REQUIRE(parser.Errors().empty());

    LlvmIrCodegen codegen("orl_struct_module");
    REQUIRE(codegen.Generate(*parser.Ast()));
    REQUIRE(codegen.Errors().empty());

    const std::string ir = codegen.DumpIR();
    REQUIRE(ir.find("Vertex = type") != std::string::npos);
    REQUIRE(ir.find("structins") != std::string::npos);
    REQUIRE(ir.find("fieldaddr") != std::string::npos);
    REQUIRE(ir.find("getelementptr inbounds %Vertex") != std::string::npos);
}

TEST_CASE("llvm codegen lowers stdlib joint helpers", "[orl][codegen][stdlib][joint]") {
    const std::string src =
        "use joint;\n"
        "matrix skin_from_joints(Joint joints[], matrix inverse_binds[], int index) {\n"
        "    matrix world = joint_world_matrix(joints, index);\n"
        "    return joint_skin_matrix(world, inverse_binds[index]);\n"
        "}\n";

    Parser parser(src);
    REQUIRE(parser.Parse());
    REQUIRE(parser.Errors().empty());

    LlvmIrCodegen codegen("orl_stdlib_joint_module");
    REQUIRE(codegen.Generate(*parser.Ast()));
    REQUIRE(codegen.Errors().empty());

    const std::string ir = codegen.DumpIR();
    REQUIRE(ir.find("Joint = type") != std::string::npos);
    REQUIRE(ir.find("define [16 x double] @joint_local_matrix") != std::string::npos);
    REQUIRE(ir.find("define [16 x double] @joint_world_matrix") != std::string::npos);
    REQUIRE(ir.find("define [16 x double] @joint_skin_matrix") != std::string::npos);
}

TEST_CASE("llvm codegen rejects unknown custom struct fields", "[orl][codegen][struct][error]") {
    const std::string src =
        "struct Weight {\n"
        "    float value;\n"
        "}\n"
        "float invalid_field() {\n"
        "    Weight item(0.5);\n"
        "    return item.missing;\n"
        "}\n";

    Parser parser(src);
    REQUIRE(parser.Parse());

    LlvmIrCodegen codegen("orl_invalid_struct_module");
    REQUIRE_FALSE(codegen.Generate(*parser.Ast()));
    REQUIRE_FALSE(codegen.Errors().empty());
}

TEST_CASE("llvm codegen lowers fixed arrays and indexed access", "[orl][codegen]") {
    const std::string src =
        "int sum_buffer() {\n"
        "    int values[3];\n"
        "    values[0] = 4;\n"
        "    values[1] = 5;\n"
        "    values[2] = 6;\n"
        "    int i = 0;\n"
        "    int sum = 0;\n"
        "    while (i < 3) {\n"
        "        sum = sum + values[i];\n"
        "        i = i + 1;\n"
        "    }\n"
        "    return sum;\n"
        "}\n";

    Parser parser(src);
    REQUIRE(parser.Parse());
    REQUIRE(parser.Ast() != nullptr);

    LlvmIrCodegen codegen("orl_array_module");
    REQUIRE(codegen.Generate(*parser.Ast()));
    REQUIRE(codegen.Errors().empty());

    const std::string ir = codegen.DumpIR();
    REQUIRE(ir.find("[3 x i64]") != std::string::npos);
    REQUIRE(ir.find("getelementptr inbounds") != std::string::npos);
}

TEST_CASE("llvm codegen lowers typed mesh buffers to pointers", "[orl][codegen][buffer]") {
    const std::string src =
        "int deform(point vertices[], matrix bones[], float weights[], int vertex_count) {\n"
        "    int i = 0;\n"
        "    while (i < vertex_count) {\n"
        "        vertices[i] = weights[i] * (bones[0] * vertices[i]);\n"
        "        i = i + 1;\n"
        "    }\n"
        "    return vertex_count;\n"
        "}\n";

    Parser parser(src);
    REQUIRE(parser.Parse());
    REQUIRE(parser.Ast() != nullptr);

    LlvmIrCodegen codegen("orl_buffer_module");
    REQUIRE(codegen.Generate(*parser.Ast()));
    REQUIRE(codegen.Errors().empty());

    const std::string ir = codegen.DumpIR();
    REQUIRE(ir.find("define i64 @deform(ptr %vertices, ptr %bones, ptr %weights, i64 %vertex_count)") !=
            std::string::npos);
    REQUIRE(ir.find("getelementptr inbounds <3 x double>") != std::string::npos);
    REQUIRE(ir.find("getelementptr inbounds [16 x double]") != std::string::npos);
}

TEST_CASE("llvm codegen lowers vector component access", "[orl][codegen][component]") {
    const std::string src =
        "float component_sum() {\n"
        "    vector direction(1, 2, 3);\n"
        "    vec4 homogeneous(4, 5, 6, 1);\n"
        "    return direction.x + direction.y + direction.z + homogeneous.w;\n"
        "}\n";

    Parser parser(src);
    REQUIRE(parser.Parse());

    LlvmIrCodegen codegen("orl_component_module");
    REQUIRE(codegen.Generate(*parser.Ast()));
    REQUIRE(codegen.Errors().empty());

    const std::string ir = codegen.DumpIR();
    REQUIRE(ir.find("<4 x double>") != std::string::npos);
    REQUIRE(ir.find("extractelement") != std::string::npos);
}

TEST_CASE("llvm codegen rejects w access on three-component vectors", "[orl][codegen][component]") {
    const std::string src =
        "float invalid_component() {\n"
        "    vector direction(1, 2, 3);\n"
        "    return direction.w;\n"
        "}\n";

    Parser parser(src);
    REQUIRE(parser.Parse());

    LlvmIrCodegen codegen("orl_invalid_component_module");
    REQUIRE_FALSE(codegen.Generate(*parser.Ast()));
    REQUIRE_FALSE(codegen.Errors().empty());
}

TEST_CASE("llvm codegen lowers quaternion operations", "[orl][codegen][quaternion]") {
    const std::string src =
        "float quaternion_ops() {\n"
        "    quat identity(0, 0, 0, 1);\n"
        "    quat rotation(0, 0.70710678, 0, 0.70710678);\n"
        "    quat combined = quat_normalize(quat_mul(identity, rotation));\n"
        "    quat inverse = quat_conjugate(combined);\n"
        "    vector direction(1, 0, 0);\n"
        "    vector restored = quat_rotate(inverse, quat_rotate(combined, direction));\n"
        "    return restored.x + combined.w;\n"
        "}\n";

    Parser parser(src);
    REQUIRE(parser.Parse());

    LlvmIrCodegen codegen("orl_quaternion_module");
    REQUIRE(codegen.Generate(*parser.Ast()));
    REQUIRE(codegen.Errors().empty());

    const std::string ir = codegen.DumpIR();
    REQUIRE(ir.find("<4 x double>") != std::string::npos);
    REQUIRE(ir.find("qnormalize") != std::string::npos);
    REQUIRE(ir.find("quatrot") != std::string::npos);
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

TEST_CASE("orl jit executes oneTBB-backed parallel for", "[orl][jit][parallel]") {
    const std::string src =
        "int transform(int values[], int unused0[], int unused1[], int unused2[], int unused3[], int count) {\n"
        "    parallel for (int index = 0; index < count; index = index + 1) {\n"
        "        values[index] = values[index] + index;\n"
        "    }\n"
        "    return count;\n"
        "}\n";

    Parser parser(src);
    REQUIRE(parser.Parse());

    LlvmIrCodegen codegen("orl_tbb_parallel_jit_module");
    REQUIRE(codegen.Generate(*parser.Ast()));

    std::int64_t values[] = {3, 7, 11};
    std::int64_t unused[1] = {};
    const std::array<void *, 5> buffers = {values, unused, unused, unused, unused};

    OrlJitEngine jit;
    REQUIRE(jit.LoadModuleWithOptimization(codegen.ReleaseModule(),
                                           codegen.ReleaseContext(),
                                           OrlOptimizationLevel::O2));
    const auto result = jit.InvokeInt64WithBufferArgs("transform", buffers, 3);
    REQUIRE(result.has_value());
    REQUIRE(*result == 3);
    REQUIRE(values[0] == 3);
    REQUIRE(values[1] == 8);
    REQUIRE(values[2] == 13);
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
