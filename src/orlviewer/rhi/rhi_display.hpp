#pragma once

#include <array>
#include <concepts>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "rhi_contracts.hpp"

// Viewer display families. Each family is a pair of calls: prepare_* once from
// a feature's attach and draw_* while a frame is active. ORL decides what is
// drawn (instances, colors, selection); the selected adapter owns the native
// shaders, pipelines, and buffers and records the draws inside its own passes.
namespace ORL::Rhi {

using Vec3 = std::array<float, 3>;
using Vec4 = std::array<float, 4>;
// Column-major, matching CameraState.
using Mat4 = std::array<float, 16>;

constexpr Mat4 identity_mat4() noexcept {
    return {
        1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f,
    };
}

// ORL-authored geometry for small viewer shapes (controller curves, locator
// crosses, guides). Registering an existing name is a no-op.
enum class ShapeKind : std::uint8_t {
    lines,
    triangles,
};

struct ShapeGeometry {
    std::vector<Vec3> positions;
    // Index pairs for lines, index triples for triangles.
    std::vector<std::uint32_t> indices;
};

// Scene meshes are loaded by the model loader and referenced by mesh name.
enum class SceneShading : std::uint8_t {
    phong,
    xray,
};

struct SurfaceMaterial {
    Vec4 ambient{0.08f, 0.08f, 0.08f, 1.0f};
    Vec4 diffuse{0.72f, 0.72f, 0.72f, 1.0f};
    Vec4 specular{0.35f, 0.35f, 0.35f, 1.0f};
    float shininess = 32.0f;
};

struct PointLight {
    Vec4 position{2.0f, 3.0f, 2.0f, 1.0f};
    Vec4 color{1.0f, 1.0f, 1.0f, 1.0f};
};

struct SceneMeshSetup {
    // Selects the front-face winding of the scene world.
    bool right_handed = true;
};

struct SceneMeshInstance {
    std::string mesh;
    Mat4 model = identity_mat4();
};

struct SceneMeshDraw {
    SceneShading shading = SceneShading::phong;
    std::span<const SceneMeshInstance> instances{};
    // Drawn again as a wireframe on top of the shaded pass.
    std::span<const SceneMeshInstance> highlighted{};
    SurfaceMaterial surface{};
    SurfaceMaterial highlight{};
    Vec4 xray_color{0.45f, 0.78f, 0.95f, 0.22f};
    PointLight light{};
};

template <typename Adapter>
concept ShapeRegistryAdapter = requires(
    Adapter& adapter,
    std::string_view name,
    const ShapeGeometry& geometry)
{
    { adapter.register_shape(name, ShapeKind{}, geometry) } -> std::same_as<Status>;
};

template <typename Adapter>
concept SceneMeshDisplayAdapter = requires(
    Adapter& adapter,
    const SceneMeshSetup& setup,
    const SceneMeshDraw& draw)
{
    { adapter.prepare_scene_meshes(setup) } -> std::same_as<Status>;
    { adapter.draw_scene_meshes(draw) } -> std::same_as<Status>;
};

template <typename Adapter>
concept ViewerDisplayAdapter = StaticAdapter<Adapter>
    && ShapeRegistryAdapter<Adapter>
    && SceneMeshDisplayAdapter<Adapter>;

} // namespace ORL::Rhi
