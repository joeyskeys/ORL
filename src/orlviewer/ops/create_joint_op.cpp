#include "create_joint_op.hpp"

#include <cmath>
#include <iostream>
#include <string>

#include <glm/geometric.hpp>
#include <glm/gtc/matrix_inverse.hpp>
#include <glm/mat4x4.hpp>
#include <glm/vec4.hpp>

#include "selection.hpp"

namespace ORL
{

CreateJointOp::CreateJointOp(ComponentManager& components, vkkk::Camera& camera,
    const glm::vec3& pivot, GLFWwindow* window, Selection& selection)
    : components(components)
    , camera(camera)
    , pivot(pivot)
    , window(window)
    , selection(selection)
{
}

void CreateJointOp::on_eval(const InputEvent& event) {
    if (event.kind == InputEvent::Kind::Key && event.action == GLFW_PRESS) {
        if (event.key == GLFW_KEY_J && !active_) {
            enter();
            return;
        }
        if (active_ && (event.key == GLFW_KEY_ENTER || event.key == GLFW_KEY_KP_ENTER)) {
            exit();
        }
        return;
    }

    if (active_ && event.kind == InputEvent::Kind::MouseButton
        && event.button == GLFW_MOUSE_BUTTON_LEFT && event.action == GLFW_PRESS)
    {
        place(event.x, event.y);
    }
}

void CreateJointOp::enter() {
    active_ = true;
    last_in_chain = {};
    std::cout << "Joint create: click to place, Enter to finish\n";
}

void CreateJointOp::exit() {
    active_ = false;
    last_in_chain = {};
    std::cout << "Joint create: done\n";
}

void CreateJointOp::on_cancel() {
    if (active_) {
        exit();
    }
}

void CreateJointOp::place(double cursor_x, double cursor_y) {
    glm::vec3 world{};
    if (!hit_pivot_plane(cursor_x, cursor_y, world)) {
        return;
    }

    const std::int64_t parent_index = last_in_chain ? components.joint_index(last_in_chain) : -1;
    const glm::vec3 local = orlviewer::world_to_local(components.packed_joints(), parent_index, world);

    orlviewer::Joint joint = orlviewer::make_identity_joint();
    joint.parent = parent_index;
    joint.translation[0] = local.x;
    joint.translation[1] = local.y;
    joint.translation[2] = local.z;

    const auto id = components.create_joint(unique_joint_name(), joint);
    if (!id) {
        std::cerr << "CreateJointOp: failed to create joint\n";
        return;
    }
    last_in_chain = id;
    selection.set(SelectionRef::joint(id));
    if (const auto* created = components.find(id)) {
        std::cout << "Created '" << created->name << "'\n";
    }
}

bool CreateJointOp::hit_pivot_plane(double cursor_x, double cursor_y, glm::vec3& world) const {
    if (window == nullptr) {
        return false;
    }

    int width = 0;
    int height = 0;
    glfwGetWindowSize(window, &width, &height);
    if (width <= 0 || height <= 0) {
        return false;
    }

    const float ndc_x = static_cast<float>(2.0 * cursor_x / static_cast<double>(width) - 1.0);
    const float ndc_y = static_cast<float>(2.0 * cursor_y / static_cast<double>(height) - 1.0);
    const glm::mat4 inv = glm::inverse(camera.ubo_data.proj * camera.ubo_data.view);

    // Unproject the same pixel at two clip depths. That ray goes through the
    // camera for perspective, and is parallel to camera.front for ortho —
    // camera.pos is only the view center, so it is not a valid ortho origin.
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

std::string CreateJointOp::unique_joint_name() const {
    for (std::size_t i = 1;; ++i) {
        std::string name = "joint" + std::to_string(i);
        if (!components.contains(name)) {
            return name;
        }
    }
}

} // namespace ORL
