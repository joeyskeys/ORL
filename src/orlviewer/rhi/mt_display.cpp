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

Status MtAdapter::draw_locators(std::string_view, std::span<const LocatorInstance>) {
    return unsupported("draw_locators");
}

Status MtAdapter::prepare_ortho_grid() {
    return unsupported("prepare_ortho_grid");
}

Status MtAdapter::draw_ortho_grid(const OrthoGridDraw&) {
    return unsupported("draw_ortho_grid");
}

} // namespace ORL::Rhi
