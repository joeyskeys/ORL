#if __has_include(<catch2/catch_all.hpp>)

#include <catch2/catch_all.hpp>

#include "orl_exec.hpp"

#include <array>
#include <cstdint>
#include <string>

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

#endif
