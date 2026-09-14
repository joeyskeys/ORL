#pragma once

#include <cstddef>
#include <iostream>
#include <string>

#include <glm/mat4x4.hpp>

#include "component_manager.hpp"
#include "selection.hpp"
#include "vp_operation.hpp"

namespace ORL
{

// Toggles a selected joint or locator's controller attachment. If the target
// has no controller and no controller is selected, create a circle controller
// at the target transform and attach it. The attachment stores a target-local
// transform and is otherwise transparent to ORL graph evaluation.
class ToggleControllerAttachmentOp
    : public VpOperation<ToggleControllerAttachmentOp> {
public:
    ToggleControllerAttachmentOp(ComponentManager& components,
        Selection& selection)
        : components(components)
        , selection(selection)
    {
    }

    void on_eval(const InputEvent&) {
        ComponentId controller;
        ComponentId target;
        for (const auto& ref : selection.refs()) {
            if (ref.kind == SelectionRef::Kind::Controller) {
                if (controller) {
                    std::cerr << "Attachment: select exactly one controller\n";
                    return;
                }
                controller = ref.component;
            }
            else if (ref.kind == SelectionRef::Kind::Joint
                || ref.kind == SelectionRef::Kind::Locator)
            {
                if (target) {
                    std::cerr << "Attachment: select exactly one joint or locator\n";
                    return;
                }
                target = ref.component;
            }
            else if (ref.kind == SelectionRef::Kind::SceneObject
                || ref.kind == SelectionRef::Kind::Vector)
            {
                std::cerr << "Attachment: remove mesh or point selection first\n";
                return;
            }
        }
        if (!target) {
            std::cerr << "Attachment: select a joint or locator\n";
            return;
        }

        const ComponentId attached = components.attached_controller(target);
        if (attached && (!controller || attached == controller)) {
            components.detach_controller(attached);
            std::cout << "Controller attachment removed\n";
            return;
        }

        bool created = false;
        if (!controller) {
            controller = create_controller(target);
            if (!controller) {
                return;
            }
            created = true;
        }

        if (const auto* attachment =
                components.controller_attachment(controller);
            attachment != nullptr && attachment->target == target)
        {
            components.detach_controller(controller);
            std::cout << "Controller attachment removed\n";
            return;
        }

        std::string error;
        if (!components.attach_controller(controller, target, &error)) {
            if (created) {
                components.destroy(controller);
            }
            std::cerr << "Attachment: " << error << '\n';
            return;
        }
        if (created) {
            // Keep the target selected while also selecting the new control,
            // so the next C press toggles the same attachment off.
            selection.set(SelectionRef::controller(controller));
        }
        std::cout << "Controller attached to '"
                  << components.find(target)->name << "'\n";
    }

private:
    ComponentId create_controller(ComponentId target) {
        const auto* meta = components.find(target);
        if (meta == nullptr) {
            std::cerr << "Attachment: target no longer exists\n";
            return {};
        }

        glm::mat4 world{1.0f};
        if (meta->kind == ComponentKind::Joint) {
            const auto index = components.joint_index(target);
            const auto packed = components.packed_joints();
            if (index < 0
                || static_cast<std::size_t>(index) >= packed.size())
            {
                std::cerr << "Attachment: joint transform is unavailable\n";
                return {};
            }
            world = orlviewer::joint_world_matrix(packed, index);
        }
        else if (meta->kind == ComponentKind::Locator) {
            const auto* locator = components.locator(target);
            if (locator == nullptr) {
                std::cerr << "Attachment: locator transform is unavailable\n";
                return {};
            }
            world = locator->xform;
        }
        else {
            std::cerr << "Attachment: target must be a joint or locator\n";
            return {};
        }

        const auto id = components.create_controller(
            unique_name(), orlrig::Controller{world},
            orlviewer::ControllerShape::Circle);
        if (!id) {
            std::cerr << "Attachment: failed to create controller\n";
        }
        return id;
    }

    std::string unique_name() const {
        for (std::size_t index = 1;; ++index) {
            const std::string name = "ctrl" + std::to_string(index);
            if (!components.contains(name)) {
                return name;
            }
        }
    }

    ComponentManager& components;
    Selection& selection;
};

} // namespace ORL
