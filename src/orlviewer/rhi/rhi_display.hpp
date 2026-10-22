#pragma once

#include <array>
#include <concepts>
#include <cstddef>
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

// A CUDA device allocation shared with the graphics backend.
struct DeviceBuffer {
    std::uint64_t device_ptr = 0;
    std::size_t bytes = 0;
};

// Joints are packed ORL rig joints; the adapter treats them as opaque records
// of joint_stride bytes that the joint shaders decode.
struct JointDisplaySetup {
    std::size_t joint_stride = 0;
    // Baked into the line pipeline when the backend has no dynamic width.
    float line_width = 2.5f;
};

struct JointDraw {
    // Host joints; ignored when use_device_joints draws the last device upload.
    std::span<const std::byte> joints{};
    std::uint32_t joint_count = 0;
    bool use_device_joints = false;
    bool lines = true;
    bool points = true;
    float point_size = 10.0f;
    Vec4 point_color{0.95f, 0.78f, 0.28f, 1.0f};
    float line_width = 2.5f;
    Vec4 line_color{0.62f, 0.44f, 0.14f, 1.0f};
};

// One registered shape drawn with a model transform and a flat color. Line
// shapes keep the baked controller width; triangle shapes write depth.
struct ShapeInstanceDraw {
    std::string shape;
    Mat4 model = identity_mat4();
    Vec4 color{1.0f, 1.0f, 1.0f, 1.0f};
};

// Locators draw one line shape instanced once per record.
struct LocatorInstance {
    Mat4 model = identity_mat4();
    Vec4 color{0.95f, 0.78f, 0.28f, 1.0f};
};

// Screen-space grid for orthographic views, drawn without depth test before
// the rest of the scene. Fields mirror the backend uniform block.
struct OrthoGridDraw {
    // Projected world origin in pixels (xy) and pixels per world unit (zw).
    Vec4 origin_px_ppu{0.0f, 0.0f, 0.0f, 0.0f};
    // Viewport size in pixels (xy) and world-space cell size (z).
    Vec4 viewport_cell{0.0f, 0.0f, 1.0f, 1.0f};
    Vec4 line_color{0.22f, 0.22f, 0.22f, 1.0f};
    // Axis color (rgb) and axis line width in pixels (a).
    Vec4 axis_color{0.08f, 0.08f, 0.08f, 2.0f};
};

// World-space XZ reference grid centered on the origin.
struct WorldGridSetup {
    std::uint32_t cell_count = 24;
    float cell_size = 1.0f;
};

// Screen-corner orientation gizmo; axes and labels describe the world frame.
struct FrameAxisSetup {
    std::array<Vec3, 3> axes{{
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f},
    }};
    std::array<std::string, 3> labels{"x", "y", "z"};
    // Empty draws the gizmo without labels.
    std::string font_path;
};

// GPU picking writes IDs into per-layer overlap-safe A-buffers so one pixel
// can report every hit beneath it. Layers render in the picking pass before
// the scene and are read back on a later frame.
enum class PickLayer : std::uint8_t {
    mesh,
    joint,
    rig,
};

struct PickLayerSetup {
    PickLayer layer = PickLayer::mesh;
    std::uint32_t nodes_per_pixel = 8;
    // Joint layer only: bytes per packed joint record.
    std::size_t joint_stride = 0;
};

struct PickHit {
    std::uint32_t id = 0;
    float depth = 0.0f;
};

struct PickReadback {
    // False until the layer has rendered a frame at or after the requested serial.
    bool rendered = false;
    std::vector<PickHit> hits;
    bool overflow = false;
};

struct MeshPickInstance {
    std::string mesh;
    Mat4 model = identity_mat4();
    std::uint32_t object_id = 0;
};

struct JointPickDraw {
    // Host joints; ignored when use_device_joints draws the last device upload.
    std::span<const std::byte> joints{};
    std::uint32_t joint_count = 0;
    bool use_device_joints = false;
    float point_size = 10.0f;
};

struct RigPickShape {
    std::string shape;
    Mat4 model = identity_mat4();
    std::uint32_t id = 0;
};

struct RigPickLocator {
    Mat4 model = identity_mat4();
    std::uint32_t id = 0;
};

struct RigPickDraw {
    std::span<const RigPickShape> controllers{};
    // Line shape instanced once per locator.
    std::string locator_shape;
    std::span<const RigPickLocator> locators{};
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

// upload_device_joints copies device joints into the current frame's joint
// buffers immediately, so a failure lets the caller fall back to host joints.
template <typename Adapter>
concept JointDisplayAdapter = requires(
    Adapter& adapter,
    const JointDisplaySetup& setup,
    DeviceBuffer device,
    std::uint32_t count,
    const JointDraw& draw)
{
    { adapter.prepare_joints(setup) } -> std::same_as<Status>;
    { adapter.upload_device_joints(device, count) } -> std::same_as<Status>;
    { adapter.draw_joints(draw) } -> std::same_as<Status>;
};

template <typename Adapter>
concept ControllerDisplayAdapter = requires(
    Adapter& adapter,
    const ShapeInstanceDraw& draw)
{
    { adapter.prepare_controllers() } -> std::same_as<Status>;
    { adapter.draw_controller(draw) } -> std::same_as<Status>;
};

template <typename Adapter>
concept LocatorDisplayAdapter = requires(
    Adapter& adapter,
    std::string_view shape,
    std::span<const LocatorInstance> instances)
{
    { adapter.prepare_locators() } -> std::same_as<Status>;
    { adapter.draw_locators(shape, instances) } -> std::same_as<Status>;
};

template <typename Adapter>
concept OrthoGridDisplayAdapter = requires(Adapter& adapter, const OrthoGridDraw& draw)
{
    { adapter.prepare_ortho_grid() } -> std::same_as<Status>;
    { adapter.draw_ortho_grid(draw) } -> std::same_as<Status>;
};

// Line shapes drawn in the overlay pass without depth test, e.g. transform
// guides. Width follows the controller lines.
template <typename Adapter>
concept OverlayLineDisplayAdapter = requires(
    Adapter& adapter,
    const ShapeInstanceDraw& draw)
{
    { adapter.prepare_overlay_lines() } -> std::same_as<Status>;
    { adapter.draw_overlay_line(draw) } -> std::same_as<Status>;
};

// A one-line runtime status (device, evaluation mode). The adapter shows it
// where its window system puts status: a status bar or an overlay HUD.
template <typename Adapter>
concept StatusTextDisplayAdapter = requires(Adapter& adapter, std::string_view text)
{
    { adapter.prepare_status_text() } -> std::same_as<Status>;
    { adapter.draw_status_text(text) } -> std::same_as<Status>;
};

// Both use the camera from set_camera.
template <typename Adapter>
concept ReferenceDisplayAdapter = requires(
    Adapter& adapter,
    const WorldGridSetup& grid,
    const FrameAxisSetup& axis)
{
    { adapter.prepare_world_grid(grid) } -> std::same_as<Status>;
    { adapter.draw_world_grid() } -> std::same_as<Status>;
    { adapter.prepare_frame_axis(axis) } -> std::same_as<Status>;
    { adapter.draw_frame_axis() } -> std::same_as<Status>;
};

// prepare_pick_layer is idempotent and resizes the layer's A-buffer to the
// current drawable, so callers repeat it after a resize. map_cursor converts
// window cursor coordinates to drawable pixels.
template <typename Adapter>
concept PickingAdapter = requires(
    Adapter& adapter,
    const Adapter& const_adapter,
    const PickLayerSetup& setup,
    double cursor,
    std::uint32_t& pixel,
    std::span<const MeshPickInstance> meshes,
    DeviceBuffer device,
    std::uint32_t count,
    const JointPickDraw& joints,
    const RigPickDraw& rig,
    std::uint64_t serial,
    PickReadback& readback)
{
    { adapter.prepare_pick_layer(setup) } -> std::same_as<Status>;
    { const_adapter.map_cursor(cursor, cursor, pixel, pixel) } -> std::same_as<bool>;
    { adapter.draw_mesh_pick(meshes) } -> std::same_as<Status>;
    { adapter.upload_device_pick_joints(device, count) } -> std::same_as<Status>;
    { adapter.draw_joint_pick(joints) } -> std::same_as<Status>;
    { adapter.draw_rig_pick(rig) } -> std::same_as<Status>;
    { adapter.read_pick(PickLayer{}, serial, pixel, pixel, readback) } -> std::same_as<Status>;
};

template <typename Adapter>
concept ViewerDisplayAdapter = StaticAdapter<Adapter>
    && ShapeRegistryAdapter<Adapter>
    && SceneMeshDisplayAdapter<Adapter>
    && JointDisplayAdapter<Adapter>
    && ControllerDisplayAdapter<Adapter>
    && LocatorDisplayAdapter<Adapter>
    && OrthoGridDisplayAdapter<Adapter>
    && OverlayLineDisplayAdapter<Adapter>
    && StatusTextDisplayAdapter<Adapter>
    && ReferenceDisplayAdapter<Adapter>
    && PickingAdapter<Adapter>;

} // namespace ORL::Rhi
