#pragma once

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <glm/geometric.hpp>
#include <glm/mat4x4.hpp>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

#include "comps/joint.hpp"
#include "gui/input.hpp"
#include "gui/window_backend.hpp"
#include "ops/create_controller_op.hpp"
#include "ops/create_joint_op.hpp"
#include "selection.hpp"
#include "vp/joint_picking_feature.hpp"
#include "vp/mesh_picking_feature.hpp"
#include "vp_operation.hpp"

namespace ORL
{

class SelectOp : public VpOperation<SelectOp> {
public:
    SelectOp(Selection& selection, ComponentManager& components, vkkk::Scene& scene,
        const vkkk::Camera& camera, vkkk::WindowBackend* window, const CreateJointOp& create_joint,
        const CreateControllerOp& create_controller)
        : selection(selection)
        , components(components)
        , scene(scene)
        , camera(camera)
        , window(window)
        , create_joint(create_joint)
        , create_controller(create_controller)
    {
    }

    void set_gpu_picking(JointPickingFeature& gpu_picking) {
        this->gpu_picking = &gpu_picking;
        gpu_picking.set_hit_callback([this](const std::vector<uint32_t>& ids, bool) {
            apply_gpu_joint_hits(ids);
        });
    }

    void set_mesh_picking(MeshPickingFeature& mesh_picking) {
        this->mesh_picking = &mesh_picking;
        mesh_picking.set_hit_callback([this](uint32_t object_id) {
            apply_gpu_mesh_hit(object_id);
        });
    }

    void on_eval(const InputEvent& event) {
        if (create_joint.active() || create_controller.active()) {
            return;
        }
        if (event.kind != InputEvent::Kind::MouseButton
            || event.button != vkkk::MouseButton::Left
            || event.action != vkkk::InputAction::Press)
        {
            return;
        }

        fallback_x = event.x;
        fallback_y = event.y;
        if (pick_cpu_controllers(event.x, event.y)) {
            return;
        }
        awaiting_joint = gpu_picking != nullptr && gpu_picking->request(
            vkkk::mouse_button_event(vkkk::MouseButton::Left, vkkk::InputAction::Press,
                event.x, event.y));
        awaiting_mesh = mesh_picking != nullptr && mesh_picking->request(
            vkkk::mouse_button_event(vkkk::MouseButton::Left, vkkk::InputAction::Press,
                event.x, event.y));
        joint_hit = false;
        mesh_id = 0;
        if (awaiting_joint || awaiting_mesh) {
            return;
        }
        pick_cpu(event.x, event.y);
    }

private:
    static constexpr float kPickPixels = 16.0f;

    void apply_gpu_joint_hits(const std::vector<uint32_t>& ids) {
        awaiting_joint = false;
        if (choose_gpu_joint(ids)) {
            joint_hit = true;
            awaiting_mesh = false;
            return;
        }
        finish_click();
    }

    void apply_gpu_mesh_hit(uint32_t object_id) {
        awaiting_mesh = false;
        mesh_id = object_id;
        if (joint_hit) {
            return;
        }
        finish_click();
    }

    void finish_click() {
        if (joint_hit || awaiting_joint || awaiting_mesh) {
            return;
        }
        if (mesh_id != 0 && mesh_picking != nullptr) {
            const auto& name = mesh_picking->object_name(mesh_id);
            if (!name.empty()) {
                selection.set(SelectionRef::scene_object(name));
                return;
            }
        }
        pick_cpu(fallback_x, fallback_y);
    }

    bool choose_gpu_joint(const std::vector<uint32_t>& ids) {
        if (ids.empty()) {
            return false;
        }
        const auto packed = components.packed_joints();
        const auto packed_ids = components.packed_joint_ids();
        float best_depth = std::numeric_limits<float>::infinity();
        SelectionRef chosen;
        for (const uint32_t index : ids) {
            if (index >= packed.size() || index >= packed_ids.size()) {
                continue;
            }
            const glm::vec3 world{orlviewer::joint_world_matrix(packed,
                static_cast<std::int64_t>(index))[3]};
            const float depth = glm::dot(world - camera.pos, camera.front);
            if (depth <= 0.0f || depth >= best_depth) {
                continue;
            }
            best_depth = depth;
            chosen = SelectionRef::joint(packed_ids[index]);
        }
        if (!chosen) {
            return false;
        }
        selection.set(chosen);
        return true;
    }

    bool pick_cpu_controllers(double cursor_x, double cursor_y) {
        SelectionRef chosen;
        if (!closest_controller(cursor_x, cursor_y, chosen)) {
            return false;
        }
        selection.set(chosen);
        return true;
    }

    void pick_cpu(double cursor_x, double cursor_y) {
        if (window == nullptr) {
            selection.set({});
            return;
        }
        SelectionRef chosen;
        float best = kPickPixels * kPickPixels;
        if (closest_controller(cursor_x, cursor_y, chosen, &best)) {
            selection.set(chosen);
            return;
        }
        pick_cpu_joints(cursor_x, cursor_y);
    }

    bool closest_controller(double cursor_x, double cursor_y, SelectionRef& chosen,
        float* best_dist = nullptr)
    {
        if (window == nullptr) {
            return false;
        }
        const auto size = window->window_size();
        const int width = static_cast<int>(size.width);
        const int height = static_cast<int>(size.height);
        if (width <= 0 || height <= 0) {
            return false;
        }
        float best = best_dist != nullptr ? *best_dist : kPickPixels * kPickPixels;
        bool found = false;
        components.for_each([&](const Component& meta) {
            if (meta.kind != ComponentKind::Controller) {
                return;
            }
            const auto* controller = components.controller(meta.id);
            if (controller == nullptr) {
                return;
            }
            const auto points = orlviewer::controller_shape_points(*controller);
            for (const auto& local : points) {
                glm::vec2 pixel{};
                if (!project(orlviewer::controller_world(*controller, local), width, height, pixel)) {
                    continue;
                }
                const float dx = pixel.x - static_cast<float>(cursor_x);
                const float dy = pixel.y - static_cast<float>(cursor_y);
                const float dist = dx * dx + dy * dy;
                if (dist < best) {
                    best = dist;
                    chosen = SelectionRef::controller(meta.id);
                    found = true;
                }
            }
        });
        if (found && best_dist != nullptr) {
            *best_dist = best;
        }
        return found;
    }

    void pick_cpu_joints(double cursor_x, double cursor_y) {
        if (window == nullptr) {
            selection.set({});
            return;
        }
        const auto size = window->window_size();
        const int width = static_cast<int>(size.width);
        const int height = static_cast<int>(size.height);
        if (width <= 0 || height <= 0) {
            selection.set({});
            return;
        }

        float best = kPickPixels * kPickPixels;
        SelectionRef chosen;
        const auto packed = components.packed_joints();
        components.for_each([&](const Component& meta) {
            if (meta.kind != ComponentKind::Joint) {
                return;
            }
            const auto index = components.joint_index(meta.id);
            const glm::vec3 world{orlviewer::joint_world_matrix(packed, index)[3]};
            glm::vec2 pixel{};
            if (!project(world, width, height, pixel)) {
                return;
            }
            const float dx = pixel.x - static_cast<float>(cursor_x);
            const float dy = pixel.y - static_cast<float>(cursor_y);
            const float dist = dx * dx + dy * dy;
            if (dist < best) {
                best = dist;
                chosen = SelectionRef::joint(meta.id);
            }
        });
        selection.set(chosen);
    }

    bool project(const glm::vec3& world, int width, int height, glm::vec2& pixel) const {
        const glm::vec4 clip = camera.ubo_data.proj * camera.ubo_data.view
            * glm::vec4{world, 1.0f};
        if (std::abs(clip.w) < 1.0e-8f) {
            return false;
        }
        const glm::vec3 ndc = glm::vec3{clip} / clip.w;
        pixel.x = (ndc.x * 0.5f + 0.5f) * static_cast<float>(width);
        pixel.y = (ndc.y * 0.5f + 0.5f) * static_cast<float>(height);
        return true;
    }

    Selection& selection;
    ComponentManager& components;
    vkkk::Scene& scene;
    const vkkk::Camera& camera;
    vkkk::WindowBackend* window = nullptr;
    const CreateJointOp& create_joint;
    const CreateControllerOp& create_controller;
    JointPickingFeature* gpu_picking = nullptr;
    MeshPickingFeature* mesh_picking = nullptr;
    double fallback_x = 0.0;
    double fallback_y = 0.0;
    uint32_t mesh_id = 0;
    bool awaiting_joint = false;
    bool awaiting_mesh = false;
    bool joint_hit = false;
};

} // namespace ORL
