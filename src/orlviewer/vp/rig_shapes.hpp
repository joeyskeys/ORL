#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <glm/vec3.hpp>

#include "comps/controller.hpp"
#include "comps/controller_curves.hpp"
#include "vp/rhi_feature.hpp"

// Rig display shapes shared by the rig display and picking features. They are
// registered by name with the adapter; registering twice is a no-op.
namespace ORL::rig_shapes
{

inline constexpr const char* kControllerQuad = "orl_controller_quad";

inline Rhi::ShapeGeometry polyline(const std::vector<glm::vec3>& points) {
    Rhi::ShapeGeometry geometry;
    geometry.positions.reserve(points.size());
    for (const auto& point : points) {
        geometry.positions.push_back({point.x, point.y, point.z});
    }
    for (std::uint32_t index = 0; index + 1 < points.size(); ++index) {
        geometry.indices.insert(geometry.indices.end(), {index, index + 1});
    }
    return geometry;
}

inline Rhi::ShapeGeometry controller_quad() {
    Rhi::ShapeGeometry geometry;
    for (const auto& point : orlviewer::controller_polygon_points()) {
        geometry.positions.push_back({point.x, point.y, point.z});
    }
    geometry.indices = {0, 1, 2, 0, 2, 3};
    return geometry;
}

inline std::string controller_shape(orlviewer::ControllerShape shape) {
    return shape == orlviewer::ControllerShape::Polygon
        ? std::string{kControllerQuad}
        : std::string{orlviewer::controller_curves::name(shape)};
}

inline Rhi::Status register_controller_shapes(ViewerAdapter& adapter) {
    for (const auto& definition : orlviewer::controller_curves::definitions) {
        const auto points = orlviewer::controller_curves::sample(
            definition.curve(), definition.segments);
        if (auto status = adapter.register_shape(
                definition.name, Rhi::ShapeKind::lines, polyline(points));
            !status)
        {
            return status;
        }
    }
    return adapter.register_shape(
        kControllerQuad, Rhi::ShapeKind::triangles, controller_quad());
}

} // namespace ORL::rig_shapes
