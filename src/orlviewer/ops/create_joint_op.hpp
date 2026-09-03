#pragma once

#include <GLFW/glfw3.h>
#include <glm/vec3.hpp>

#include "component_manager.hpp"
#include "concepts/camera.h"
#include "vp_operation.hpp"

namespace ORL
{

class Selection;

// Modal joint placement: J enters, left-click places on the pivot/view plane,
// Enter finishes. Each session starts a new chain; later joints parent to the
// previous joint created in that session.
class CreateJointOp : public VpOperation<CreateJointOp> {
public:
    CreateJointOp(ComponentManager& components, vkkk::Camera& camera, const glm::vec3& pivot,
        GLFWwindow* window, Selection& selection);

    void on_eval(const InputEvent& event);
    bool active() const { return active_; }
    bool is_active() const { return active_; }
    void on_cancel();

private:
    void enter();
    void exit();
    void place(double cursor_x, double cursor_y);
    bool hit_pivot_plane(double cursor_x, double cursor_y, glm::vec3& world) const;
    std::string unique_joint_name() const;

    ComponentManager& components;
    vkkk::Camera& camera;
    const glm::vec3& pivot;
    GLFWwindow* window = nullptr;
    Selection& selection;
    bool active_ = false;
    ComponentId last_in_chain;
};

} // namespace ORL
