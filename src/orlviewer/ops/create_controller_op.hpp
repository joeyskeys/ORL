#pragma once

#include <iostream>
#include <string>

#include "asset_mgr/scene.h"
#include "component_manager.hpp"
#include "selection.hpp"
#include "vp_operation.hpp"

namespace ORL
{

// Explicit controller creation utility. The default viewer shortcut now
// toggles target attachments; this operation remains available to callers
// that still need to create an unattached controller.
class CreateControllerOp : public VpOperation<CreateControllerOp> {
public:
    CreateControllerOp(ComponentManager& components, vkkk::Scene& scene,
        Selection& selection)
        : components(components)
        , scene(scene)
        , selection(selection)
    {
    }

    void on_eval(const InputEvent&) {
        create();
    }

private:
    void create() {
        orlrig::Controller controller;
        const auto* focus = selection.focus();
        if (focus != nullptr && focus->kind == SelectionRef::Kind::SceneObject) {
            if (const auto* object = scene.find_object(focus->object_name);
                object != nullptr && !object->mesh_name.empty())
            {
                controller.xform = object->model;
            }
        }
        else if (focus != nullptr && focus->kind == SelectionRef::Kind::Joint) {
            const auto index = components.joint_index(focus->component);
            if (index >= 0) {
                controller.xform = orlviewer::joint_world_matrix(
                    components.packed_joints(), index);
            }
        }
        else if (focus != nullptr
            && focus->kind == SelectionRef::Kind::Locator)
        {
            if (const auto* locator = components.locator(focus->component);
                locator != nullptr)
            {
                controller.xform = locator->xform;
            }
        }
        else if (focus != nullptr
            && focus->kind == SelectionRef::Kind::Controller)
        {
            controller.xform = components.controller_world_xform(
                focus->component);
        }

        const auto id = components.create_controller(
            unique_name(), controller, orlviewer::ControllerShape::Circle);
        if (!id) {
            std::cerr << "CreateControllerOp: failed to create controller\n";
            return;
        }
        selection.replace(SelectionRef::controller(id));
        if (const auto* created = components.find(id)) {
            std::cout << "Created '" << created->name << "'\n";
        }
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
    vkkk::Scene& scene;
    Selection& selection;
};

} // namespace ORL
