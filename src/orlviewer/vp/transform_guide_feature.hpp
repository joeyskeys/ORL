#pragma once

#include <algorithm>
#include <iostream>

#include <glm/geometric.hpp>
#include <glm/mat4x4.hpp>
#include <glm/vec4.hpp>

#include "concepts/camera.h"
#include "ops/move_op.hpp"
#include "ops/rotate_op.hpp"
#include "ops/scale_op.hpp"
#include "vp/rhi_feature.hpp"

namespace ORL
{

class TransformGuideFeature final : public RhiFeature {
public:
    TransformGuideFeature(const MoveOp& move, const RotateOp& rotate,
        const ScaleOp& scale, const vkkk::Camera& camera)
        : move(move)
        , rotate(rotate)
        , scale(scale)
        , camera(camera)
    {
    }

    void on_attach(ViewerAdapter& adapter, Rhi::Extent2D) {
        report(adapter.register_shape(kLine, Rhi::ShapeKind::lines, {
            .positions = {{-1.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}},
            .indices = {0, 1},
        }));
        report(adapter.prepare_overlay_lines());
    }

    void on_submit(ViewerAdapter& adapter, const Rhi::FrameState&) {
        const auto active = active_guide();
        if (!active.visible) {
            return;
        }

        glm::vec3 axis = active.axis;
        const float axis_length = glm::length(axis);
        if (axis_length < 1.0e-6f) {
            return;
        }
        axis /= axis_length;

        glm::mat4 model{1.0f};
        model[0] = glm::vec4{axis * guide_half_length(), 0.0f};
        model[3] = glm::vec4{active.origin, 1.0f};
        report(adapter.draw_overlay_line({
            .shape = kLine,
            .model = to_rhi(model),
            .color = to_rhi(axis_color(active.axis_kind)),
        }));
    }

private:
    struct GuideState {
        bool visible = false;
        TransformAxis axis_kind = TransformAxis::None;
        glm::vec3 axis{0.0f};
        glm::vec3 origin{0.0f};
    };

    static void report(const Rhi::Status& status) {
        if (!status) {
            std::cerr << "TransformGuideFeature: " << status.message << '\n';
        }
    }

    GuideState active_guide() const {
        if (move.guide_visible()) {
            return {
                true, move.guide_axis_kind(),
                move.guide_axis(), move.guide_origin(),
            };
        }
        if (rotate.guide_visible()) {
            return {
                true, rotate.guide_axis_kind(),
                rotate.guide_axis(), rotate.guide_origin(),
            };
        }
        if (scale.guide_visible()) {
            return {
                true, scale.guide_axis_kind(),
                scale.guide_axis(), scale.guide_origin(),
            };
        }
        return {};
    }

    static constexpr const char* kLine = "orl_transform_guide_line";

    static glm::vec4 axis_color(TransformAxis axis) {
        switch (axis) {
        case TransformAxis::X:
            return {0.95f, 0.2f, 0.25f, 1.0f};
        case TransformAxis::Y:
            return {0.25f, 0.9f, 0.35f, 1.0f};
        case TransformAxis::Z:
            return {0.25f, 0.45f, 1.0f, 1.0f};
        default:
            return {1.0f, 1.0f, 1.0f, 1.0f};
        }
    }

    float guide_half_length() const {
        return std::max(10.0f, camera.far);
    }

    const MoveOp& move;
    const RotateOp& rotate;
    const ScaleOp& scale;
    const vkkk::Camera& camera;
};

} // namespace ORL
