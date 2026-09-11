#pragma once

#include <cmath>
#include <iostream>
#include <string>

#include <glm/geometric.hpp>
#include <glm/gtc/matrix_inverse.hpp>
#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

#include "component_manager.hpp"
#include "concepts/camera.h"
#include "gui/window_backend.hpp"
#include "ops/create_joint_op.hpp"
#include "selection.hpp"
#include "vp_operation.hpp"

namespace ORL
{

// Modal controller placement: C enters, left-click places a curve locator on
// the pivot/view plane, Enter finishes. Shift-C places a polygon instead.
class CreateControllerOp : public VpOperation<CreateControllerOp> {
public:
    CreateControllerOp(ComponentManager& components, vkkk::Camera& camera, const glm::vec3& pivot,
        vkkk::WindowBackend* window, Selection& selection, CreateJointOp* create_joint = nullptr)
        : components(components)
        , camera(camera)
        , pivot(pivot)
        , window(window)
        , selection(selection)
        , create_joint(create_joint)
    {
    }

    void on_eval(const InputEvent& event) {
        if (event.kind == InputEvent::Kind::Key && event.action == vkkk::InputAction::Press) {
            if (event.key == vkkk::Key::C && !active_) {
                shape = (event.mods & vkkk::input_mod::shift)
                    ? orlviewer::ControllerShape::Polygon
                    : orlviewer::ControllerShape::Curve;
                enter();
                return;
            }
            if (active_ && (event.key == vkkk::Key::Enter || event.key == vkkk::Key::NumpadEnter)) {
                exit();
            }
            return;
        }

        if (active_ && event.kind == InputEvent::Kind::MouseButton
            && event.button == vkkk::MouseButton::Left && event.action == vkkk::InputAction::Press)
        {
            place(event.x, event.y);
        }
    }

    bool is_active() const { return active_; }
    void on_cancel() {
        if (active_) {
            exit();
        }
    }

private:
    void enter() {
        if (create_joint != nullptr) {
            create_joint->cancel();
        }
        active_ = true;
        const char* kind = shape == orlviewer::ControllerShape::Polygon ? "polygon" : "curve";
        std::cout << "Controller create (" << kind << "): click to place, Enter to finish\n";
    }

    void exit() {
        active_ = false;
        std::cout << "Controller create: done\n";
    }

    void place(double cursor_x, double cursor_y) {
        glm::vec3 world{};
        if (!hit_pivot_plane(cursor_x, cursor_y, world)) {
            return;
        }
        const auto id = components.create_controller(
            unique_name(), orlrig::make_controller(world), shape);
        if (!id) {
            std::cerr << "CreateControllerOp: failed to create controller\n";
            return;
        }
        selection.set(SelectionRef::controller(id));
        if (const auto* created = components.find(id)) {
            std::cout << "Created '" << created->name << "'\n";
        }
    }

    bool hit_pivot_plane(double cursor_x, double cursor_y, glm::vec3& world) const {
        if (window == nullptr) {
            return false;
        }
        const auto size = window->window_size();
        const int width = static_cast<int>(size.width);
        const int height = static_cast<int>(size.height);
        if (width <= 0 || height <= 0) {
            return false;
        }

        const float ndc_x = static_cast<float>(2.0 * cursor_x / static_cast<double>(width) - 1.0);
        const float ndc_y = static_cast<float>(2.0 * cursor_y / static_cast<double>(height) - 1.0);
        const glm::mat4 inv = glm::inverse(camera.ubo_data.proj * camera.ubo_data.view);

        const auto unproject = [&](float ndc_z, glm::vec3& out) {
            glm::vec4 clip = inv * glm::vec4{ndc_x, ndc_y, ndc_z, 1.0f};
            if (std::abs(clip.w) < 1e-8f) {
                return false;
            }
            out = glm::vec3{clip / clip.w};
            return true;
        };

        glm::vec3 ray_origin{};
        glm::vec3 ray_far{};
        if (!unproject(-1.0f, ray_origin) || !unproject(1.0f, ray_far)) {
            return false;
        }
        glm::vec3 ray_dir = ray_far - ray_origin;
        const float ray_len = glm::length(ray_dir);
        if (ray_len < 1e-8f) {
            return false;
        }
        ray_dir /= ray_len;

        const glm::vec3 normal = glm::normalize(camera.front);
        const float denom = glm::dot(ray_dir, normal);
        if (std::abs(denom) < 1e-6f) {
            return false;
        }
        const float t = glm::dot(pivot - ray_origin, normal) / denom;
        world = ray_origin + ray_dir * t;
        return glm::dot(world - camera.pos, camera.front) > 0.0f;
    }

    std::string unique_name() const {
        for (std::size_t i = 1;; ++i) {
            std::string name = "ctrl" + std::to_string(i);
            if (!components.contains(name)) {
                return name;
            }
        }
    }

    ComponentManager& components;
    vkkk::Camera& camera;
    const glm::vec3& pivot;
    vkkk::WindowBackend* window = nullptr;
    Selection& selection;
    CreateJointOp* create_joint = nullptr;
    orlviewer::ControllerShape shape = orlviewer::ControllerShape::Curve;
    bool active_ = false;
};

} // namespace ORL
