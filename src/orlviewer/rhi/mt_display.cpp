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

} // namespace ORL::Rhi
