#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <string_view>
#include <vector>

#include <glm/vec3.hpp>

#include "concepts/curve.hpp"

namespace orlviewer
{

enum class ControllerShape {
    Circle,
    Square,
    Triangle,
    Diamond,
    Star,
    Cross,
    Arrow,
    // Compatibility value for the original filled-polygon binding.
    Polygon,
    // Compatibility name for the original default curve binding.
    Curve = Circle,
};

namespace controller_curves
{

namespace detail
{

inline std::vector<glm::vec3> closed(std::initializer_list<glm::vec3> points) {
    std::vector<glm::vec3> result(points.begin(), points.end());
    if (!result.empty()) {
        result.push_back(result.front());
    }
    return result;
}

inline std::vector<glm::vec3> circle_points(float radius = 0.2f,
    int segments = 64)
{
    std::vector<glm::vec3> points;
    points.reserve(static_cast<std::size_t>(segments) + 1);
    constexpr float kPi = 3.14159265358979323846f;
    for (int index = 0; index <= segments; ++index) {
        const float angle = 2.0f * kPi
            * static_cast<float>(index) / static_cast<float>(segments);
        points.emplace_back(
            radius * std::cos(angle), radius * std::sin(angle), 0.0f);
    }
    return points;
}

inline std::vector<glm::vec3> star_points(float outer = 0.24f,
    float inner = 0.105f)
{
    std::vector<glm::vec3> points;
    points.reserve(11);
    constexpr float kPi = 3.14159265358979323846f;
    for (int index = 0; index <= 10; ++index) {
        const float angle = 0.5f * kPi
            + static_cast<float>(index) * kPi / 5.0f;
        const float radius = index % 2 == 0 ? outer : inner;
        points.emplace_back(
            radius * std::cos(angle), radius * std::sin(angle), 0.0f);
    }
    return points;
}

} // namespace detail

// Degree-one clamped B-splines are piecewise linear. They preserve the
// intended corners of Maya-style controller curves instead of smoothing them.
// These function-local statics provide stable objects for non-owning links.
inline const vkkk::BSplineCurve& circle() {
    static const vkkk::BSplineCurve curve{detail::circle_points(), 1};
    return curve;
}

inline const vkkk::BSplineCurve& square() {
    static const vkkk::BSplineCurve curve{detail::closed({
        {-0.2f, -0.2f, 0.0f},
        { 0.2f, -0.2f, 0.0f},
        { 0.2f,  0.2f, 0.0f},
        {-0.2f,  0.2f, 0.0f},
    }), 1};
    return curve;
}

inline const vkkk::BSplineCurve& triangle() {
    static const vkkk::BSplineCurve curve{detail::closed({
        { 0.0f,  0.24f, 0.0f},
        {-0.21f, -0.15f, 0.0f},
        { 0.21f, -0.15f, 0.0f},
    }), 1};
    return curve;
}

inline const vkkk::BSplineCurve& diamond() {
    static const vkkk::BSplineCurve curve{detail::closed({
        { 0.0f,  0.24f, 0.0f},
        { 0.2f,  0.0f, 0.0f},
        { 0.0f, -0.24f, 0.0f},
        {-0.2f,  0.0f, 0.0f},
    }), 1};
    return curve;
}

inline const vkkk::BSplineCurve& star() {
    static const vkkk::BSplineCurve curve{detail::star_points(), 1};
    return curve;
}

inline const vkkk::BSplineCurve& cross() {
    static const vkkk::BSplineCurve curve{detail::closed({
        {-0.07f,  0.24f, 0.0f},
        { 0.07f,  0.24f, 0.0f},
        { 0.07f,  0.07f, 0.0f},
        { 0.24f,  0.07f, 0.0f},
        { 0.24f, -0.07f, 0.0f},
        { 0.07f, -0.07f, 0.0f},
        { 0.07f, -0.24f, 0.0f},
        {-0.07f, -0.24f, 0.0f},
        {-0.07f, -0.07f, 0.0f},
        {-0.24f, -0.07f, 0.0f},
        {-0.24f,  0.07f, 0.0f},
        {-0.07f,  0.07f, 0.0f},
    }), 1};
    return curve;
}

inline const vkkk::BSplineCurve& arrow() {
    static const vkkk::BSplineCurve curve{detail::closed({
        {-0.24f,  0.08f, 0.0f},
        { 0.02f,  0.08f, 0.0f},
        { 0.02f,  0.17f, 0.0f},
        { 0.24f,  0.0f, 0.0f},
        { 0.02f, -0.17f, 0.0f},
        { 0.02f, -0.08f, 0.0f},
        {-0.24f, -0.08f, 0.0f},
    }), 1};
    return curve;
}

struct Definition {
    ControllerShape kind;
    std::string_view name;
    const vkkk::BSplineCurve& (*curve)();
    std::uint32_t segments;
};

inline constexpr std::array<Definition, 7> definitions{{
    {ControllerShape::Circle, "circle", &circle, 64},
    {ControllerShape::Square, "square", &square, 64},
    {ControllerShape::Triangle, "triangle", &triangle, 60},
    {ControllerShape::Diamond, "diamond", &diamond, 64},
    {ControllerShape::Star, "star", &star, 60},
    {ControllerShape::Cross, "cross", &cross, 60},
    {ControllerShape::Arrow, "arrow", &arrow, 56},
}};

inline const vkkk::BSplineCurve& get(ControllerShape kind) {
    switch (kind) {
    case ControllerShape::Circle:
        return circle();
    case ControllerShape::Square:
    case ControllerShape::Polygon:
        return square();
    case ControllerShape::Triangle:
        return triangle();
    case ControllerShape::Diamond:
        return diamond();
    case ControllerShape::Star:
        return star();
    case ControllerShape::Cross:
        return cross();
    case ControllerShape::Arrow:
        return arrow();
    }
    return circle();
}

inline std::string_view name(ControllerShape kind) {
    if (kind == ControllerShape::Polygon) {
        return "square";
    }
    for (const auto& definition : definitions) {
        if (definition.kind == kind) {
            return definition.name;
        }
    }
    return "circle";
}

inline ControllerShape next(ControllerShape current) {
    for (std::size_t index = 0; index < definitions.size(); ++index) {
        if (definitions[index].kind == current) {
            return definitions[(index + 1) % definitions.size()].kind;
        }
    }
    return definitions.front().kind;
}

inline std::size_t curve_segments(ControllerShape kind) {
    if (kind == ControllerShape::Polygon) {
        kind = ControllerShape::Square;
    }
    for (const auto& definition : definitions) {
        if (definition.kind == kind) {
            return definition.segments;
        }
    }
    return definitions.front().segments;
}

inline std::vector<glm::vec3> sample(
    const vkkk::BSplineCurve& curve, std::size_t segments = 64)
{
    if (segments == 0) {
        return {curve.evaluate(0.0f)};
    }
    std::vector<glm::vec3> points;
    points.reserve(segments + 1);
    for (std::size_t index = 0; index <= segments; ++index) {
        points.push_back(curve.evaluate(
            static_cast<float>(index) / static_cast<float>(segments)));
    }
    return points;
}

inline std::vector<glm::vec3> points(
    ControllerShape shape, std::size_t segments = 0)
{
    if (segments == 0) {
        segments = curve_segments(shape);
    }
    return sample(get(shape), segments);
}

} // namespace orlviewer::controller_curves

} // namespace orlviewer
