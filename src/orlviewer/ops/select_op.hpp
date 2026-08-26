#pragma once

#include <cmath>
#include <string>
#include <utility>

#include <GLFW/glfw3.h>
#include <glm/mat4x4.hpp>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

#include "comps/joint.hpp"
#include "ops/create_joint_op.hpp"
#include "selection.hpp"
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
        pick(event.x, event.y);
    }

private:
    static constexpr float kPickPixels = 16.0f;

    void pick(double cursor_x, double cursor_y) {
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

        const auto packed = components.packed_joints();
        components.for_each([&](const Component& meta) {
            if (meta.kind != ComponentKind::Joint) {
                return;
            }
            const auto index = components.joint_index(meta.id);
            consider(glm::vec3{orlviewer::joint_world_matrix(packed, index)[3]},
                SelectionRef::joint(meta.id));
        });

        scene.for_each_object([&](const std::string& name, const vkkk::SceneObject& object) {
            consider(glm::vec3{object.model[3]}, SelectionRef::scene_object(name));
        });

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
};

} // namespace ORL
