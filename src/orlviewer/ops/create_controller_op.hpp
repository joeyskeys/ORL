#pragma once

#include <iostream>
#include <string>

#include "asset_mgr/scene.h"
#include "component_manager.hpp"
#include "selection.hpp"
#include "vp_operation.hpp"

namespace ORL
{

// Immediate controller creation. C creates a controller from the focused mesh
// or joint transform, or at the origin when no supported object is selected.
class CreateControllerOp : public VpOperation<CreateControllerOp> {
public:
    CreateControllerOp(ComponentManager& components, vkkk::Scene& scene,
        Selection& selection)
        : components(components)
        , scene(scene)
        , selection(selection)
    {
    }

    void on_eval(const InputEvent& event) {
        if (event.kind == InputEvent::Kind::Key
            && event.action == vkkk::InputAction::Press
            && event.key == vkkk::Key::C)
        {
            create();
        }
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

        const auto id = components.create_controller(
            unique_name(), controller, orlviewer::ControllerShape::Circle);
        if (!id) {
            std::cerr << "CreateControllerOp: failed to create controller\n";
            return;
        }
        selection.set(SelectionRef::controller(id));
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
