#include "create_joint_op.hpp"

#include <cmath>
#include <iostream>
#include <string>

#include <glm/geometric.hpp>
#include <glm/gtc/matrix_inverse.hpp>
#include <glm/mat4x4.hpp>
#include <glm/vec4.hpp>

namespace ORL
{
namespace
{

glm::mat4 joint_local_matrix(const orlviewer::Joint& joint) {
    glm::vec4 q{
        static_cast<float>(joint.rotation[0]),
        static_cast<float>(joint.rotation[1]),
        static_cast<float>(joint.rotation[2]),
        static_cast<float>(joint.rotation[3]),
    };
    const float qlen = glm::length(q);
    q = qlen > 0.0f ? q / qlen : glm::vec4{0.0f, 0.0f, 0.0f, 1.0f};

    const float xx = q.x * q.x;
    const float yy = q.y * q.y;
    const float zz = q.z * q.z;
    const float xy = q.x * q.y;
    const float xz = q.x * q.z;
    const float yz = q.y * q.z;
    const float wx = q.w * q.x;
    const float wy = q.w * q.y;
    const float wz = q.w * q.z;
    const float sx = static_cast<float>(joint.scale[0]);
    const float sy = static_cast<float>(joint.scale[1]);
    const float sz = static_cast<float>(joint.scale[2]);

    return glm::mat4{
        glm::vec4{sx * (1.0f - 2.0f * (yy + zz)), sx * (2.0f * (xy + wz)), sx * (2.0f * (xz - wy)), 0.0f},
        glm::vec4{sy * (2.0f * (xy - wz)), sy * (1.0f - 2.0f * (xx + zz)), sy * (2.0f * (yz + wx)), 0.0f},
        glm::vec4{sz * (2.0f * (xz + wy)), sz * (2.0f * (yz - wx)), sz * (1.0f - 2.0f * (xx + yy)), 0.0f},
        glm::vec4{
            static_cast<float>(joint.translation[0]),
            static_cast<float>(joint.translation[1]),
            static_cast<float>(joint.translation[2]),
            1.0f},
    };
}

glm::mat4 joint_world_matrix(const std::vector<orlviewer::Joint>& joints, std::int64_t index) {
    if (index < 0 || static_cast<std::size_t>(index) >= joints.size()) {
        return glm::mat4{1.0f};
    }

    glm::mat4 world = joint_local_matrix(joints[static_cast<std::size_t>(index)]);
    std::int64_t parent = joints[static_cast<std::size_t>(index)].parent;
    for (int i = 0; i < 64 && parent >= 0 && static_cast<std::size_t>(parent) < joints.size(); ++i) {
        world = joint_local_matrix(joints[static_cast<std::size_t>(parent)]) * world;
        parent = joints[static_cast<std::size_t>(parent)].parent;
    }
    return world;
}

glm::vec3 world_to_local(const std::vector<orlviewer::Joint>& joints, std::int64_t parent_index,
    const glm::vec3& world)
{
    if (parent_index < 0) {
        return world;
    }
    const glm::vec4 local = glm::inverse(joint_world_matrix(joints, parent_index))
        * glm::vec4{world, 1.0f};
    return glm::vec3{local};
}

} // namespace

CreateJointOp::CreateJointOp(ComponentManager& components, vkkk::Camera& camera,
    const glm::vec3& pivot, GLFWwindow* window)
    : components(components)
    , camera(camera)
    , pivot(pivot)
    , window(window)
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

void CreateJointOp::place(double cursor_x, double cursor_y) {
    glm::vec3 world{};
    if (!hit_pivot_plane(cursor_x, cursor_y, world)) {
        return;
    }

    const std::int64_t parent_index = last_in_chain ? components.joint_index(last_in_chain) : -1;
    const glm::vec3 local = world_to_local(components.packed_joints(), parent_index, world);

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
    glm::vec4 far_h = inv * glm::vec4{ndc_x, ndc_y, 1.0f, 1.0f};
    if (std::abs(far_h.w) < 1e-8f) {
        return false;
    }
    far_h /= far_h.w;

    glm::vec3 ray_dir = glm::vec3{far_h} - camera.pos;
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

    const float t = glm::dot(pivot - camera.pos, normal) / denom;
    if (t < 0.0f) {
        return false;
    }
    world = camera.pos + ray_dir * t;
    return true;
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
