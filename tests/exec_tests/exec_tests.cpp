#if __has_include(<catch2/catch_all.hpp>)

#include <catch2/catch_all.hpp>

#include "orl_exec.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <span>
#include <string>

#include <glm/vec3.hpp>

#include "orlrig/abi.hpp"
#include "orlrig/handle_registry.hpp"
#include "orlrig/joint.hpp"
#include "orlrig/locator.hpp"

using namespace ORL::exec;

namespace
{

constexpr const char* kAddSource = R"(
int add_one(int input[], int output[], int count) {
    parallel for (int index = 0; index < count; index = index + 1) {
        output[index] = input[index] + 1;
    }
    return count;
}
)";

OrlProgram RequireProgram() {
    auto program = OrlProgram::Compile(kAddSource, {.entry_function = "add_one"});
    REQUIRE(program.valid());
    return program;
}

} // namespace

TEST_CASE("CPU execution transports and validates exact handles",
    "[orl][exec][handle][cpu]")
{
    const auto program = OrlProgram::Compile(R"(
        handle joint_handle;
        int mixed(int count, joint_handle value, float scale) {
            if (value == value) { return count; }
            return 0;
        }
    )", {
        .entry_function = "mixed",
        .source_name = "orlrig",
    });
    REQUIRE(program.valid());
    REQUIRE(program.parameters().size() == 3);
    REQUIRE(program.parameters()[1].kind == ParameterKind::Handle);
    REQUIRE(program.parameters()[1].canonical_type_name
        == "orlrig::joint_handle");
    REQUIRE(program.parameters()[1].handle_type_id
        == orlcomp::HandleTypeIdFor("orlrig::joint_handle"));

    auto execution = OrlExecution::Create(program, Backend::Cpu);
    REQUIRE(execution.valid());
    REQUIRE_FALSE(execution.bind_handle(
        "value", HandleValue{0, 3}));
    REQUIRE_FALSE(execution.bind_handle(
        "value", HandleValue{
            orlcomp::HandleTypeIdFor("orlrig::locator_handle"), 3}));
    REQUIRE(execution.bind_int("count", 17));
    REQUIRE(execution.bind_float("scale", 1.0));
    REQUIRE(execution.bind_handle(
        "value", HandleValue{
            orlcomp::HandleTypeIdFor("orlrig::joint_handle"), 3}));
    const auto result = execution.evaluate();
    REQUIRE(result.has_value());
    REQUIRE(*result == 17);
    REQUIRE_FALSE(execution.evaluate().has_value());
}

TEST_CASE("runtime compilation applies semantic handle diagnostics first",
    "[orl][exec][handle][semantic]")
{
    const auto program = OrlProgram::Compile(R"(
        handle joint_handle;
        int invalid(joint_handle value) {
            return value + value;
        }
    )", {.entry_function = "invalid", .source_name = "orlrig"});

    REQUIRE_FALSE(program.valid());
    REQUIRE(std::any_of(program.errors().begin(), program.errors().end(),
        [](const std::string& error) {
            return error.find("ORL_ANALYSIS_HANDLE_OPERATION")
                != std::string::npos;
        }));
}

TEST_CASE("CPU execution binds storage-backed Joint views",
    "[orl][exec][handle][view][cpu]")
{
    const auto program = OrlProgram::Compile(R"(
        handle joint_handle;
        struct Joint {
            int parent;
            int selected;
            int pad0;
            int pad1;
            vec4 translation;
            quat rotation;
            vec4 scale;
        }
        int edit(joint_handle value) {
            Joint data = Joint(value);
            int parent = data.parent;
            vec4 translation = data.translation;
            translation = vec4(
                translation.x + 2.0,
                translation.y, translation.z, translation.w);
            data.translation = translation;
            return parent;
        }
    )", {.entry_function = "edit", .source_name = "orlrig"});

    auto execution = OrlExecution::Create(program, Backend::Cpu);
    REQUIRE(execution.valid());
    orlrig::Joint joint = orlrig::make_identity_joint();
    joint.parent = 7;
    joint.translation[0] = 1.0;
    orlrig::HandleViewContext context{
        {&joint, 1, sizeof(orlrig::Joint), true}, {}, 1};
    REQUIRE(execution.bind_handle_view_context(context));
    REQUIRE(execution.bind_handle("value", HandleValue{
        orlcomp::HandleTypeIdFor("orlrig::joint_handle"), 0}));
    const auto result = execution.evaluate();
    REQUIRE(result.has_value());
    REQUIRE(*result == 7);
    REQUIRE(joint.translation[0] == Catch::Approx(3.0));
}

TEST_CASE("ORL binary cache round trips backend artifacts",
    "[orl][cache]")
{
    const auto directory =
        std::filesystem::temp_directory_path() / "orl_binary_cache_test";
    std::error_code error;
    std::filesystem::remove_all(directory, error);

    orlcomp::OrlBinaryCache cache({directory, false});
    const std::string key = "cpu-cache-key";
    const std::vector<std::uint8_t> expected{0, 1, 2, 127, 255};
    REQUIRE(cache.save(
        orlcomp::OrlBinaryKind::CpuObject,
        key,
        std::span<const std::uint8_t>(expected.data(), expected.size())));

    std::vector<std::uint8_t> actual;
    REQUIRE(cache.load(orlcomp::OrlBinaryKind::CpuObject, key, actual));
    REQUIRE(actual == expected);

    orlcomp::OrlBinaryCache forced({directory, true});
    REQUIRE_FALSE(forced.load(
        orlcomp::OrlBinaryKind::CpuObject, key, actual));

    std::filesystem::remove_all(directory, error);
}

TEST_CASE("CPU execution stores and reloads its object cache",
    "[orl][cache][cpu]")
{
    const auto directory =
        std::filesystem::temp_directory_path() / "orl_cpu_object_cache_test";
    std::error_code error;
    std::filesystem::remove_all(directory, error);

    const auto program = RequireProgram();
    const orlcomp::OrlBinaryCacheOptions options{directory, false};
    auto execution = OrlExecution::Create(program, Backend::Cpu, options);
    REQUIRE(execution.valid());

    OrlBuffer input("int", sizeof(std::int64_t));
    OrlBuffer output("int", sizeof(std::int64_t));
    REQUIRE(input.resize(1));
    REQUIRE(output.resize(1));
    const std::int64_t input_value = 3;
    REQUIRE(input.write(0, input_value));
    REQUIRE(execution.bind_buffer("input", input));
    REQUIRE(execution.bind_buffer("output", output));
    REQUIRE(execution.bind_int("count", 1));
    REQUIRE(execution.evaluate(1).has_value());
    std::int64_t output_value = 0;
    REQUIRE(output.read(0, &output_value));
    REQUIRE(output_value == 4);

    bool object_exists = false;
    if (std::filesystem::exists(directory / "cpu")) {
        for (const auto& entry :
            std::filesystem::directory_iterator(directory / "cpu"))
        {
            object_exists |= entry.path().extension() == ".obj";
        }
    }
    REQUIRE(object_exists);

    auto cached = OrlExecution::Create(program, Backend::Cpu, options);
    REQUIRE(cached.valid());
    REQUIRE(cached.ir().empty());
    REQUIRE(cached.bind_buffer("input", input));
    REQUIRE(cached.bind_buffer("output", output));
    REQUIRE(cached.bind_int("count", 1));
    REQUIRE(cached.evaluate(1).has_value());
    REQUIRE(output.read(0, &output_value));
    REQUIRE(output_value == 4);

    orlcomp::OrlBinaryCacheOptions forced{directory, true};
    auto recompiled = OrlExecution::Create(program, Backend::Cpu, forced);
    REQUIRE(recompiled.valid());
    REQUIRE_FALSE(recompiled.ir().empty());

    std::filesystem::remove_all(directory, error);
}

TEST_CASE("CUDA execution persists a device binary or PTX cache",
    "[orl][cache][cuda]")
{
    const auto directory =
        std::filesystem::temp_directory_path() / "orl_cuda_binary_cache_test";
    std::error_code error;
    std::filesystem::remove_all(directory, error);

    const auto program = RequireProgram();
    const orlcomp::OrlBinaryCacheOptions options{directory, false};
    auto execution = OrlExecution::Create(program, Backend::Cuda, options);
    if (!execution.valid()) {
        if (execution.errors().empty()) {
            WARN("CUDA execution runtime unavailable in this environment");
        } else {
            WARN(execution.errors().back());
        }
        return;
    }

    OrlBuffer input("int", sizeof(std::int64_t));
    OrlBuffer output("int", sizeof(std::int64_t));
    REQUIRE(input.resize(1));
    REQUIRE(output.resize(1));
    const std::int64_t input_value = 5;
    REQUIRE(input.write(0, input_value));
    REQUIRE(execution.bind_buffer("input", input));
    REQUIRE(execution.bind_buffer("output", output));
    REQUIRE(execution.bind_int("count", 1));
    REQUIRE(execution.evaluate(1).has_value());

    bool device_cache_exists = false;
    for (const auto kind :
        {orlcomp::OrlBinaryKind::CudaCubin,
         orlcomp::OrlBinaryKind::CudaPtx})
    {
        const auto kind_directory =
            kind == orlcomp::OrlBinaryKind::CudaCubin
                ? directory / "cuda_cubin"
                : directory / "cuda_ptx";
        if (std::filesystem::exists(kind_directory)) {
            device_cache_exists |= !std::filesystem::is_empty(kind_directory);
        }
    }
    REQUIRE(device_cache_exists);

    auto cached = OrlExecution::Create(program, Backend::Cuda, options);
    REQUIRE(cached.valid());
    REQUIRE(cached.ir().empty());
    REQUIRE(cached.bind_buffer("input", input));
    REQUIRE(cached.bind_buffer("output", output));
    REQUIRE(cached.bind_int("count", 1));
    REQUIRE(cached.evaluate(1).has_value());

    std::filesystem::remove_all(directory, error);
}

#if 0 // Numeric index constraint fixture awaits typed graph specialization.
TEST_CASE("orlexec evaluates locator-fed aim constraint on CPU",
    "[orl][exec][constraint][locator][cpu]")
{
    const auto program = OrlProgram::Compile(R"(
use locator;
use constraint/aim_locator;
int apply(Locator targets[], matrix subjects[], vector axes[],
    int target_index, int subject_index, int target_count, int subject_count) {
    return constraint_aim_locator(targets, subjects, axes,
        target_index, subject_index, target_count, subject_count);
}
)", {.entry_function = "apply"});
    REQUIRE(program.valid());
    auto execution = OrlExecution::Create(program, Backend::Cpu);
    REQUIRE(execution.valid());

    OrlBuffer targets(orlrig::kLocatorOrlType, orlrig::kLocatorStride);
    OrlBuffer subjects(orlrig::kMatrixOrlType, orlrig::kMatrixStride);
    OrlBuffer axes("vector", sizeof(double) * 4);
    REQUIRE(targets.resize(1));
    REQUIRE(subjects.resize(1));
    REQUIRE(axes.resize(1));
    orlrig::pack_xform(
        orlrig::make_locator(glm::vec3{0.0f, 1.0f, 0.0f}),
        static_cast<double*>(targets.data()));
    auto* subject = static_cast<double*>(subjects.data());
    for (int index = 0; index < 16; ++index) {
        subject[index] = index % 5 == 0 ? 1.0 : 0.0;
    }
    auto* axis = static_cast<double*>(axes.data());
    axis[0] = 1.0;
    axis[1] = 0.0;
    axis[2] = 0.0;
    axis[3] = 0.0;

    REQUIRE(execution.bind_buffer("targets", targets));
    REQUIRE(execution.bind_buffer("subjects", subjects));
    REQUIRE(execution.bind_buffer("axes", axes));
    REQUIRE(execution.bind_int("target_index", 0));
    REQUIRE(execution.bind_int("subject_index", 0));
    REQUIRE(execution.bind_int("target_count", 1));
    REQUIRE(execution.bind_int("subject_count", 1));
    REQUIRE(execution.evaluate(1).has_value());
    REQUIRE(subject[4] == Catch::Approx(1.0));
    REQUIRE(subject[3] == Catch::Approx(0.0));
    REQUIRE(subject[7] == Catch::Approx(0.0));

    auto gpu = OrlExecution::Create(program, Backend::Cuda);
    if (!gpu.valid()) {
        const std::string reason = gpu.errors().empty()
            ? "CUDA execution runtime unavailable in this environment"
            : gpu.errors().back();
        WARN(reason);
        return;
    }
    OrlBuffer gpu_targets(orlrig::kLocatorOrlType, orlrig::kLocatorStride);
    OrlBuffer gpu_subjects(orlrig::kMatrixOrlType, orlrig::kMatrixStride);
    OrlBuffer gpu_axes("vector", sizeof(double) * 4);
    REQUIRE(gpu_targets.resize(1));
    REQUIRE(gpu_subjects.resize(1));
    REQUIRE(gpu_axes.resize(1));
    orlrig::pack_xform(
        orlrig::make_locator(glm::vec3{0.0f, 1.0f, 0.0f}),
        static_cast<double*>(gpu_targets.data()));
    auto* gpu_subject_data = static_cast<double*>(gpu_subjects.data());
    for (int index = 0; index < 16; ++index) {
        gpu_subject_data[index] = index % 5 == 0 ? 1.0 : 0.0;
    }
    std::memcpy(gpu_axes.data(), axis, sizeof(double) * 4);
    REQUIRE(gpu.bind_buffer("targets", gpu_targets));
    REQUIRE(gpu.bind_buffer("subjects", gpu_subjects));
    REQUIRE(gpu.bind_buffer("axes", gpu_axes));
    REQUIRE(gpu.bind_int("target_index", 0));
    REQUIRE(gpu.bind_int("subject_index", 0));
    REQUIRE(gpu.bind_int("target_count", 1));
    REQUIRE(gpu.bind_int("subject_count", 1));
    REQUIRE(gpu.evaluate(1).has_value());
    const auto* gpu_subject = static_cast<const double*>(gpu_subjects.data());
    REQUIRE(gpu_subject[4] == Catch::Approx(subject[4]));
}

#endif

TEST_CASE("orlexec buffers grow without losing active elements", "[orl][exec][buffer]") {
    OrlBuffer buffer("int", sizeof(std::int64_t));
    REQUIRE(buffer.resize(2));
    REQUIRE(buffer.write<std::int64_t>(0, 7));
    REQUIRE(buffer.write<std::int64_t>(1, 11));
    const auto old_capacity = buffer.capacity();

    REQUIRE(buffer.reserve(old_capacity + 8));
    REQUIRE(buffer.capacity() >= old_capacity + 8);
    std::int64_t value = 0;
    REQUIRE(buffer.read(0, &value));
    REQUIRE(value == 7);
    REQUIRE(buffer.read(1, &value));
    REQUIRE(value == 11);

    REQUIRE(buffer.resize(4));
    REQUIRE(buffer.read(2, &value));
    REQUIRE(value == 0);
    REQUIRE(buffer.read(3, &value));
    REQUIRE(value == 0);
    buffer.clear();
    REQUIRE(buffer.count() == 0);
    REQUIRE(buffer.capacity() >= old_capacity + 8);
}

TEST_CASE("orlexec evaluates named CPU buffer bindings after growth", "[orl][exec][cpu]") {
    OrlProgram program = RequireProgram();
    auto execution = OrlExecution::Create(program, Backend::Cpu);
    REQUIRE(execution.valid());

    OrlBuffer input("int", sizeof(std::int64_t));
    OrlBuffer output("int", sizeof(std::int64_t));
    REQUIRE(input.resize(3));
    REQUIRE(output.resize(3));
    REQUIRE(input.write<std::int64_t>(0, 3));
    REQUIRE(input.write<std::int64_t>(1, 7));
    REQUIRE(input.write<std::int64_t>(2, 11));

    REQUIRE(execution.bind_buffer("input", input));
    REQUIRE(execution.bind_buffer("output", output));
    REQUIRE(execution.bind_int("count", 3));
    const auto result = execution.evaluate();
    REQUIRE(result.has_value());
    REQUIRE(*result == 3);

    std::int64_t value = 0;
    REQUIRE(output.read(0, &value));
    REQUIRE(value == 4);
    REQUIRE(output.read(1, &value));
    REQUIRE(value == 8);
    REQUIRE(output.read(2, &value));
    REQUIRE(value == 12);

    REQUIRE(input.resize(5));
    REQUIRE(output.resize(5));
    REQUIRE(input.write<std::int64_t>(3, 13));
    REQUIRE(input.write<std::int64_t>(4, 17));
    REQUIRE(execution.bind_int("count", 5));
    const auto grown_result = execution.evaluate();
    REQUIRE(grown_result.has_value());
    REQUIRE(*grown_result == 5);
    REQUIRE(output.read(4, &value));
    REQUIRE(value == 18);
}

TEST_CASE("orlexec binds implicit solver context without a graph socket",
    "[orl][exec][solver_context][cpu]")
{
    const auto program = OrlProgram::Compile(R"(
        int read_context() {
            return solver_context.joint_count
                + solver_context.controller_count;
        }
    )", {.entry_function = "read_context"});
    REQUIRE(program.valid());
    REQUIRE(program.parameters().size() == 1);
    REQUIRE(program.parameters().front().name == "__orl_solver_context");
    REQUIRE(program.parameters().front().orl_type == "SolverContext");
    REQUIRE(program.parameters().front().kind == ParameterKind::Buffer);

    auto execution = OrlExecution::Create(program, Backend::Cpu);
    REQUIRE(execution.valid());
    REQUIRE(execution.set_solver_context(3, 4));
    const auto first_result = execution.evaluate();
    REQUIRE(first_result.has_value());
    REQUIRE(*first_result == 7);

    REQUIRE(execution.set_solver_context(9, 2));
    const auto result = execution.evaluate();
    REQUIRE(result.has_value());
    REQUIRE(*result == 11);

    auto gpu = OrlExecution::Create(program, Backend::Cuda);
    if (!gpu.valid()) {
        if (gpu.errors().empty()) {
            WARN("CUDA execution runtime unavailable in this environment");
        } else {
            WARN(gpu.errors().back());
        }
        return;
    }
    REQUIRE(gpu.set_solver_context(5, 6));
    const auto gpu_result = gpu.evaluate(1);
    REQUIRE(gpu_result.has_value());
    REQUIRE(*gpu_result == 11);
}

TEST_CASE("orlexec exposes packed rig buffers through solver context",
    "[orl][exec][solver_context][buffers][cpu]")
{
    const auto program = OrlProgram::Compile(R"(
        use joint;
        use locator;
        int read_context() {
            Joint joint = solver_context.joints[0];
            Locator locator = solver_context.locators[0];
            matrix controller = solver_context.controllers[0];
            point locator_position = locator.xform * point(0.0, 0.0, 0.0);
            point controller_position = controller * point(0.0, 0.0, 0.0);
            int result = joint.parent;
            if (locator_position.x > 1.0) {
                result = result + 10;
            }
            if (controller_position.x > 2.0) {
                result = result + 100;
            }
            return result;
        }
    )", {.entry_function = "read_context"});
    REQUIRE(program.valid());

    constexpr std::size_t joints_offset = sizeof(orlrig::SolverContext);
    constexpr std::size_t locators_offset =
        joints_offset + orlrig::kJointStride;
    constexpr std::size_t controllers_offset =
        locators_offset + orlrig::kLocatorStride;
    OrlBuffer storage(orlrig::kSolverContextOrlType, 1);
    REQUIRE(storage.resize(
        controllers_offset + orlrig::kMatrixStride));

    std::int64_t parent = 7;
    std::memcpy(
        static_cast<std::byte*>(storage.data()) + joints_offset,
        &parent, sizeof(parent));
    double locator[16] = {};
    locator[0] = locator[5] = locator[10] = locator[15] = 1.0;
    locator[3] = 2.0;
    std::memcpy(
        static_cast<std::byte*>(storage.data()) + locators_offset,
        locator, sizeof(locator));
    double controller[16] = {};
    controller[0] = controller[5] = controller[10] = controller[15] = 1.0;
    controller[3] = 3.0;
    std::memcpy(
        static_cast<std::byte*>(storage.data()) + controllers_offset,
        controller, sizeof(controller));

    const orlrig::SolverContext context{
        1, 1, 1,
        static_cast<std::int64_t>(joints_offset),
        static_cast<std::int64_t>(controllers_offset),
        static_cast<std::int64_t>(locators_offset)};
    std::memcpy(storage.data(), &context, sizeof(context));

    auto execution = OrlExecution::Create(program, Backend::Cpu);
    REQUIRE(execution.valid());
    REQUIRE(execution.bind_solver_context(PackedBufferView{
        storage.data(), storage.byte_size(), 0, storage.byte_size(),
        storage.version()}));
    REQUIRE(execution.set_solver_context(context));
    const auto result = execution.evaluate();
    REQUIRE(result.has_value());
    REQUIRE(*result == 117);
}

TEST_CASE("orlexec binds implicit hierarchy context and data",
    "[orl][exec][hierarchy][cpu]")
{
    const auto program = OrlProgram::Compile(R"(
        use hierarchy;
        int read_hierarchy() {
            return hierarchy_preorder_id(1)
                + hierarchy_depth(1)
                + hierarchy_context.joint_count;
        }
    )", {.entry_function = "read_hierarchy"});
    REQUIRE(program.valid());
    REQUIRE(program.parameters().size() == 2);
    REQUIRE(program.parameters()[0].name == "__orl_hierarchy_context");
    REQUIRE(program.parameters()[1].name == "__orl_hierarchy_data");
    REQUIRE(program.parameters()[0].element_stride
        == sizeof(orlrig::HierarchyContext));
    REQUIRE(program.parameters()[1].element_stride
        == orlrig::kHierarchyDataStride);

    auto execution = OrlExecution::Create(program, Backend::Cpu);
    REQUIRE(execution.valid());
    orlrig::HierarchyContext context;
    context.joint_count = 2;
    context.preorder_offset = 0;
    context.depth_offset = 2;
    OrlBuffer data(orlrig::kHierarchyDataOrlType,
        orlrig::kHierarchyDataStride);
    REQUIRE(data.resize(4));
    REQUIRE(data.write<std::int64_t>(0, 10));
    REQUIRE(data.write<std::int64_t>(1, 20));
    REQUIRE(data.write<std::int64_t>(2, 0));
    REQUIRE(data.write<std::int64_t>(3, 1));

    REQUIRE(execution.set_hierarchy_context(context));
    REQUIRE(execution.bind_hierarchy_data(data));
    const auto result = execution.evaluate();
    REQUIRE(result.has_value());
    REQUIRE(*result == 23);

    auto gpu = OrlExecution::Create(program, Backend::Cuda);
    if (!gpu.valid()) {
        WARN((gpu.errors().empty()
            ? "CUDA execution runtime unavailable in this environment"
            : gpu.errors().back()));
        return;
    }
    REQUIRE(gpu.set_hierarchy_context(context));
    REQUIRE(gpu.bind_hierarchy_data(data));
    const auto gpu_result = gpu.evaluate(1);
    REQUIRE(gpu_result.has_value());
    REQUIRE(*gpu_result == 23);
}

TEST_CASE("orlexec appends hierarchy ABI after solver context",
    "[orl][exec][hierarchy][solver_context][cpu]")
{
    const auto program = OrlProgram::Compile(R"(
        use hierarchy;
        int read_both() {
            return solver_context.joint_count
                + hierarchy_context.joint_count
                + hierarchy_data[0];
        }
    )", {.entry_function = "read_both"});
    REQUIRE(program.valid());
    REQUIRE(program.parameters().size() == 3);
    REQUIRE(program.parameters()[0].name == "__orl_solver_context");
    REQUIRE(program.parameters()[1].name == "__orl_hierarchy_context");
    REQUIRE(program.parameters()[2].name == "__orl_hierarchy_data");

    auto execution = OrlExecution::Create(program, Backend::Cpu);
    REQUIRE(execution.valid());
    orlrig::HierarchyContext context;
    context.joint_count = 4;
    OrlBuffer data(orlrig::kHierarchyDataOrlType,
        orlrig::kHierarchyDataStride);
    REQUIRE(data.resize(1));
    REQUIRE(data.write<std::int64_t>(0, 5));
    REQUIRE(execution.set_solver_context(3, 0));
    REQUIRE(execution.set_hierarchy_context(context));
    REQUIRE(execution.bind_hierarchy_data(data));
    const auto result = execution.evaluate();
    REQUIRE(result.has_value());
    REQUIRE(*result == 12);
}

TEST_CASE("orlexec reports invalid named bindings", "[orl][exec][cpu][error]") {
    OrlProgram program = RequireProgram();
    auto execution = OrlExecution::Create(program, Backend::Cpu);
    REQUIRE(execution.valid());

    OrlBuffer wrong_type("float", sizeof(double));
    REQUIRE(wrong_type.resize(1));
    REQUIRE_FALSE(execution.bind_buffer("count", wrong_type));
    REQUIRE_FALSE(execution.bind_int("input", 1));

    OrlBuffer input("int", sizeof(std::int64_t));
    REQUIRE(input.resize(1));
    REQUIRE(execution.bind_buffer("input", input));
    REQUIRE_FALSE(execution.evaluate().has_value());
    REQUIRE_FALSE(execution.errors().empty());
}

TEST_CASE("orlexec retains program diagnostics", "[orl][exec][compile][error]") {
    const auto program = OrlProgram::Compile(
        "int invalid(int values[]) { return 0; }",
        {.entry_function = "missing"});
    REQUIRE_FALSE(program.valid());
    REQUIRE_FALSE(program.errors().empty());

    auto execution = OrlExecution::Create(program, Backend::Cpu);
    REQUIRE_FALSE(execution.valid());
    REQUIRE_FALSE(execution.errors().empty());
}

TEST_CASE("orlexec executes CUDA buffers when CUDA is available", "[orl][exec][cuda]") {
    OrlProgram program = RequireProgram();
    auto execution = OrlExecution::Create(program, Backend::Cuda);
    if (!execution.valid()) {
        const std::string reason = execution.errors().empty()
            ? "CUDA execution runtime unavailable in this environment"
            : execution.errors().back();
        WARN(reason);
        return;
    }

    OrlBuffer input("int", sizeof(std::int64_t));
    OrlBuffer output("int", sizeof(std::int64_t));
    REQUIRE(input.resize(3));
    REQUIRE(output.resize(3));
    REQUIRE(input.write<std::int64_t>(0, 3));
    REQUIRE(input.write<std::int64_t>(1, 7));
    REQUIRE(input.write<std::int64_t>(2, 11));
    REQUIRE(execution.bind_buffer("input", input));
    REQUIRE(execution.bind_buffer("output", output));
    REQUIRE(execution.bind_int("count", 3));
    REQUIRE(execution.evaluate(3).has_value());

    std::int64_t value = 0;
    REQUIRE(output.read(2, &value));
    REQUIRE(value == 12);
}

TEST_CASE("orlexec evaluates a packed buffer subrange on CUDA",
    "[orl][exec][cuda][packed]")
{
    const auto program = OrlProgram::Compile(
        "int read(int values[]) { return values[0]; }\n",
        {.entry_function = "read"});
    REQUIRE(program.valid());

    auto execution = OrlExecution::Create(program, Backend::Cuda);
    if (!execution.valid()) {
        const std::string reason = execution.errors().empty()
            ? "CUDA execution runtime unavailable in this environment"
            : execution.errors().back();
        WARN(reason);
        return;
    }

    std::array<std::int64_t, 2> storage{13, 29};
    REQUIRE(execution.bind_packed_buffer(
        "values",
        PackedBufferView{
            storage.data(),
            sizeof(storage),
            sizeof(std::int64_t),
            sizeof(std::int64_t),
            1}));
    const auto result = execution.evaluate();
    REQUIRE(result.has_value());
    REQUIRE(*result == 29);
}

#endif
