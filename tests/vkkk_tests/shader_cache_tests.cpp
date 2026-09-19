#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <string>

#include <vulkan/vulkan_raii.hpp>

#include "vk_ins/compute_shader.hpp"
#include "vk_ins/shader_module_pack.hpp"

namespace
{

namespace fs = std::filesystem;

constexpr const char* kVertexShader = R"glsl(
#version 450
void main() {
    gl_Position = vec4(0.0);
}
)glsl";

constexpr const char* kComputeShader = R"glsl(
#version 450
layout(local_size_x = 1) in;
void main() {}
)glsl";

fs::path test_cache_directory() {
    return fs::temp_directory_path() / "orl_vkkk_shader_cache_test";
}

} // namespace

TEST_CASE("vkkk SPIR-V cache round-trips shader modules",
    "[vkkk][shader-cache]")
{
    const auto cache_directory = test_cache_directory();
    std::error_code error;
    fs::remove_all(cache_directory, error);

    vkkk::ShaderCacheOptions options;
    options.directory = cache_directory;

    vkkk::ShaderModule compiled;
    REQUIRE(compiled.load(kVertexShader,
        vk::ShaderStageFlagBits::eVertex, "cache_test", options));

    const auto cached_path = vkkk::spirv_cache_path(
        kVertexShader, vk::ShaderStageFlagBits::eVertex, options);
    REQUIRE(fs::exists(cached_path));

    vkkk::ShaderModule loaded;
    REQUIRE(loaded.load_spirv(cached_path,
        vk::ShaderStageFlagBits::eVertex));
    REQUIRE(loaded.spirv_code == compiled.spirv_code);

    const auto explicit_path = cache_directory / "explicit.spv";
    REQUIRE(compiled.save_spirv(explicit_path));
    REQUIRE(fs::exists(explicit_path));

    vkkk::ShaderModule explicit_loaded;
    REQUIRE(explicit_loaded.load_spirv(explicit_path,
        vk::ShaderStageFlagBits::eVertex));
    REQUIRE(explicit_loaded.spirv_code == compiled.spirv_code);

    vkkk::ComputeShader compute;
    REQUIRE(compute.load(kComputeShader, "compute_cache_test", options));
    const auto compute_cached_path = vkkk::spirv_cache_path(
        kComputeShader, vk::ShaderStageFlagBits::eCompute, options);
    REQUIRE(fs::exists(compute_cached_path));

    vkkk::ComputeShader loaded_compute;
    REQUIRE(loaded_compute.load_spirv(compute_cached_path));
    REQUIRE(loaded_compute.spirv_code == compute.spirv_code);

    options.force_recompile = true;
    vkkk::ShaderModule recompiled;
    REQUIRE(recompiled.load(kVertexShader,
        vk::ShaderStageFlagBits::eVertex, "cache_test", options));
    REQUIRE(recompiled.spirv_code == compiled.spirv_code);

    fs::remove_all(cache_directory, error);
}
