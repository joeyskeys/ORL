#include "mt_adapter.hpp"

#include <string>

namespace ORL::Rhi {
namespace {

Status unsupported(std::string_view call) {
    return Status::failure(StatusCode::unsupported,
        std::string{call} + " is not implemented by the mt adapter yet");
}

} // namespace

Status MtAdapter::register_shape(std::string_view, ShapeKind, const ShapeGeometry&) {
    return unsupported("register_shape");
}

Status MtAdapter::prepare_scene_meshes(const SceneMeshSetup&) {
    return unsupported("prepare_scene_meshes");
}

Status MtAdapter::draw_scene_meshes(const SceneMeshDraw&) {
    return unsupported("draw_scene_meshes");
}

Status MtAdapter::prepare_joints(const JointDisplaySetup&) {
    return unsupported("prepare_joints");
}

Status MtAdapter::upload_device_joints(DeviceBuffer, std::uint32_t) {
    return unsupported("upload_device_joints");
}

Status MtAdapter::draw_joints(const JointDraw&) {
    return unsupported("draw_joints");
}

Status MtAdapter::prepare_controllers() {
    return unsupported("prepare_controllers");
}

Status MtAdapter::draw_controller(const ShapeInstanceDraw&) {
    return unsupported("draw_controller");
}

Status MtAdapter::prepare_locators() {
    return unsupported("prepare_locators");
}

Status MtAdapter::upload_device_locators(DeviceBuffer, std::uint32_t) {
    return unsupported("upload_device_locators");
}

Status MtAdapter::draw_locators(const LocatorDraw&) {
    return unsupported("draw_locators");
}

Status MtAdapter::prepare_ortho_grid() {
    return unsupported("prepare_ortho_grid");
}

Status MtAdapter::draw_ortho_grid(const OrthoGridDraw&) {
    return unsupported("draw_ortho_grid");
}

Status MtAdapter::prepare_overlay_lines() {
    return unsupported("prepare_overlay_lines");
}

Status MtAdapter::draw_overlay_line(const ShapeInstanceDraw&) {
    return unsupported("draw_overlay_line");
}

Status MtAdapter::prepare_status_text() {
    return unsupported("prepare_status_text");
}

Status MtAdapter::draw_status_text(std::string_view) {
    return unsupported("draw_status_text");
}

Status MtAdapter::prepare_world_grid(const WorldGridSetup&) {
    return unsupported("prepare_world_grid");
}

Status MtAdapter::draw_world_grid() {
    return unsupported("draw_world_grid");
}

Status MtAdapter::prepare_frame_axis(const FrameAxisSetup&) {
    return unsupported("prepare_frame_axis");
}

Status MtAdapter::draw_frame_axis() {
    return unsupported("draw_frame_axis");
}

Status MtAdapter::prepare_pick_layer(const PickLayerSetup&) {
    return unsupported("prepare_pick_layer");
}

bool MtAdapter::map_cursor(double, double, std::uint32_t&, std::uint32_t&) const {
    return false;
}

Status MtAdapter::draw_mesh_pick(std::span<const MeshPickInstance>) {
    return unsupported("draw_mesh_pick");
}

Status MtAdapter::upload_device_pick_joints(DeviceBuffer, std::uint32_t) {
    return unsupported("upload_device_pick_joints");
}

Status MtAdapter::draw_joint_pick(const JointPickDraw&) {
    return unsupported("draw_joint_pick");
}

Status MtAdapter::upload_device_pick_locators(DeviceBuffer, std::uint32_t) {
    return unsupported("upload_device_pick_locators");
}

Status MtAdapter::draw_rig_pick(const RigPickDraw&) {
    return unsupported("draw_rig_pick");
}

Status MtAdapter::read_pick(PickLayer, std::uint64_t, std::uint32_t, std::uint32_t,
    PickReadback&)
{
    return unsupported("read_pick");
}

Status MtAdapter::prepare_mesh_adjacency() {
    return unsupported("prepare_mesh_adjacency");
}

Status MtAdapter::build_mesh_adjacency(const MeshAdjacencyInput&, MeshAdjacency&) {
    return unsupported("build_mesh_adjacency");
}

} // namespace ORL::Rhi
