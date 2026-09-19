#pragma once

#include <string>
#include <string_view>

#include <glm/vec3.hpp>

#include "component_manager.hpp"
#include "selection.hpp"
#include "vp_operation.hpp"

namespace ORL
{

// Modal joint mirroring. Press M, then X/Y/Z to duplicate the selected joint
// subtrees across the corresponding world-axis plane.
class MirrorOp : public VpOperation<MirrorOp> {
public:
    static constexpr OpMode kMode = OpMode::Modal;

    MirrorOp(ComponentManager& components, Selection& selection)
        : components(components)
        , selection(selection)
    {
    }

    void on_enter();
    void on_eval(const InputEvent& event);
    void on_confirm();
    void on_cancel();

    bool active() const { return active_; }
    bool is_active() const { return active_; }

private:
    enum class Axis {
        None,
        X,
        Y,
        Z,
    };

    static Axis axis_for_key(vkkk::Key key);
    static glm::vec3 mirrored_position(
        const glm::vec3& position, Axis axis);
    std::string unique_joint_name(std::string_view source) const;
    bool mirror(Axis axis);

    ComponentManager& components;
    Selection& selection;
    bool active_ = false;
};

} // namespace ORL
