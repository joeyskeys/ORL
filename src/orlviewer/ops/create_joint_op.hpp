#pragma once

#include <string>

#include <glm/vec3.hpp>

#include "component_manager.hpp"
#include "concepts/camera.h"
#include "gui/window_backend.hpp"
#include "vp_operation.hpp"

namespace ORL
{

class Selection;

// Modal joint placement: J enters, left-click places on the pivot/view plane,
// Enter finishes. Each session starts a new chain; later joints parent to the
// previous joint created in that session.
class CreateJointOp : public VpOperation<CreateJointOp> {
public:
    static constexpr OpMode kMode = OpMode::Modal;

    struct Preview {
        bool visible = false;
        glm::vec3 world{};
        ComponentId parent;
    };

    CreateJointOp(ComponentManager& components, vkkk::Camera& camera, const glm::vec3& pivot,
        vkkk::WindowBackend* window, Selection& selection);

    void on_enter() { begin_session(); }
    bool begin_extension();
    void on_confirm();
    void on_eval(const InputEvent& event);
    bool active() const { return active_; }
    bool is_active() const { return active_; }
    void on_cancel();
    const Preview& preview() const { return preview_; }
    void refresh_preview();

private:
    // Keep this distinct from VpOperation::enter(), which ControlMap
    // discovers when registering the modal operation.
    void begin_session();
    void exit();
    void place(double cursor_x, double cursor_y);
    void update_preview(double cursor_x, double cursor_y);
    bool hit_pivot_plane(double cursor_x, double cursor_y, glm::vec3& world) const;
    std::string unique_joint_name() const;

    ComponentManager& components;
    vkkk::Camera& camera;
    const glm::vec3& pivot;
    vkkk::WindowBackend* window = nullptr;
    Selection& selection;
    bool active_ = false;
    ComponentId last_in_chain;
    Preview preview_;
};

// Modal joint placement that continues an existing chain. The wrapped
// CreateJointOp owns the placement implementation so both J and E share the
// same interaction and naming behavior.
class ExtendJointChainOp : public VpOperation<ExtendJointChainOp> {
public:
    static constexpr OpMode kMode = OpMode::Modal;

    explicit ExtendJointChainOp(CreateJointOp& create_joint)
        : create_joint(create_joint)
    {
    }

    void on_enter() { create_joint.begin_extension(); }
    void on_confirm() { create_joint.on_confirm(); }
    void on_eval(const InputEvent& event) { create_joint.on_eval(event); }
    bool active() const { return create_joint.active(); }
    bool is_active() const { return create_joint.is_active(); }
    const CreateJointOp::Preview& preview() const {
        return create_joint.preview();
    }
    void on_cancel() { create_joint.on_cancel(); }

private:
    CreateJointOp& create_joint;
};

} // namespace ORL
