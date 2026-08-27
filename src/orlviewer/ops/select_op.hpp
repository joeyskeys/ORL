#pragma once

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <GLFW/glfw3.h>
#include <glm/geometric.hpp>
#include <glm/mat4x4.hpp>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

#include "comps/joint.hpp"
#include "ops/create_joint_op.hpp"
#include "selection.hpp"
#include "vp/joint_picking_feature.hpp"
#include "vp_operation.hpp"

namespace ORL
{

class SelectOp : public VpOperation<SelectOp> {
public:
    SelectOp(Selection& selection, ComponentManager& components, vkkk::Scene& scene,
        const vkkk::Camera& camera, GLFWwindow* window, const CreateJointOp& create_joint)
        : selection(selection)
        , components(components)
        , scene(scene)
        , camera(camera)
        , window(window)
        , create_joint(create_joint)
    {
    }

    void set_gpu_picking(JointPickingFeature& gpu_picking) {
        this->gpu_picking = &gpu_picking;
        gpu_picking.set_hit_callback([this](const std::vector<uint32_t>& ids, bool) {
            apply_gpu_hits(ids);
        });
    }

    void on_eval(const InputEvent& event) {
        if (create_joint.active()) {
            return;
        }
        if (event.kind != InputEvent::Kind::MouseButton
            || event.button != GLFW_MOUSE_BUTTON_LEFT
            || event.action != GLFW_PRESS)
        {
            return;
        }

        fallback_x = event.x;
        fallback_y = event.y;
        if (gpu_picking != nullptr && gpu_picking->request(event.x, event.y)) {
            return;
        }
        pick(event.x, event.y, PickMask::All);
    }

private:
    static constexpr float kPickPixels = 16.0f;

    enum class PickMask {
        Joints = 1,
        Objects = 2,
        All = 3,
    };

    static bool has(PickMask mask, PickMask bit) {
        return (static_cast<int>(mask) & static_cast<int>(bit)) != 0;
    }

    void apply_gpu_hits(const std::vector<uint32_t>& ids) {
        if (choose_gpu_joint(ids)) {
            return;
        }
        pick(fallback_x, fallback_y, PickMask::Objects);
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
        selection.replace(chosen);
        return true;
    }

    void pick(double cursor_x, double cursor_y, PickMask mask) {
        if (window == nullptr) {
            return;
        }
        int width = 0;
        int height = 0;
        glfwGetWindowSize(window, &width, &height);
        if (width <= 0 || height <= 0) {
            return;
        }

        float best = kPickPixels * kPickPixels;
        SelectionRef chosen;

        const auto consider = [&](const glm::vec3& world, SelectionRef ref) {
            glm::vec2 pixel{};
            if (!project(world, width, height, pixel)) {
                return;
            }
            const float dx = pixel.x - static_cast<float>(cursor_x);
            const float dy = pixel.y - static_cast<float>(cursor_y);
            const float dist = dx * dx + dy * dy;
            if (dist < best) {
                best = dist;
                chosen = std::move(ref);
            }
        };

        if (has(mask, PickMask::Joints)) {
            const auto packed = components.packed_joints();
            components.for_each([&](const Component& meta) {
                if (meta.kind != ComponentKind::Joint) {
                    return;
                }
                const auto index = components.joint_index(meta.id);
                consider(glm::vec3{orlviewer::joint_world_matrix(packed, index)[3]},
                    SelectionRef::joint(meta.id));
            });
        }

        if (has(mask, PickMask::Objects)) {
            scene.for_each_object([&](const std::string& name, const vkkk::SceneObject& object) {
                consider(glm::vec3{object.model[3]}, SelectionRef::scene_object(name));
            });
        }

        selection.replace(chosen);
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
    GLFWwindow* window = nullptr;
    const CreateJointOp& create_joint;
    JointPickingFeature* gpu_picking = nullptr;
    double fallback_x = 0.0;
    double fallback_y = 0.0;
};

} // namespace ORL
