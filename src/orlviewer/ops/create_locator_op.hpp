#pragma once

#include <iostream>
#include <string>

#include "asset_mgr/scene.h"
#include "component_manager.hpp"
#include "selection.hpp"
#include "vp_operation.hpp"

namespace ORL
{

// Immediate locator creation. L creates a locator at the focused transform,
// or at the origin when no transform is selected.
class CreateLocatorOp : public VpOperation<CreateLocatorOp> {
public:
    CreateLocatorOp(ComponentManager& components, vkkk::Scene& scene,
        Selection& selection)
        : components(components)
        , scene(scene)
        , selection(selection)
    {
    }

    void on_eval(const InputEvent& event) {
        if (event.kind == InputEvent::Kind::Key
            && event.action == vkkk::InputAction::Press
            && event.key == vkkk::Key::L)
        {
            create();
        }
    }

private:
    void create() {
        orlrig::Locator locator;
        const auto* focus = selection.focus();
        if (focus != nullptr && focus->kind == SelectionRef::Kind::SceneObject) {
            if (const auto* object = scene.find_object(focus->object_name);
                object != nullptr && !object->mesh_name.empty())
            {
                locator.xform = object->model;
            }
        }
        else if (focus != nullptr && focus->kind == SelectionRef::Kind::Joint) {
            const auto index = components.joint_index(focus->component);
            if (index >= 0) {
                locator.xform = orlviewer::joint_world_matrix(
                    components.packed_joints(), index);
            }
        }
        else if (focus != nullptr && focus->kind == SelectionRef::Kind::Controller) {
            locator.xform = components.controller_world_xform(
                focus->component);
        }
        else if (focus != nullptr && focus->kind == SelectionRef::Kind::Locator) {
            if (const auto* selected = components.locator(focus->component);
                selected != nullptr)
            {
                locator.xform = selected->xform;
            }
        }

        const auto id = components.create_locator(unique_name(), locator);
        if (!id) {
            std::cerr << "CreateLocatorOp: failed to create locator\n";
            return;
        }
        selection.set(SelectionRef::locator(id));
        if (const auto* created = components.find(id)) {
            std::cout << "Created '" << created->name << "'\n";
        }
    }

    std::string unique_name() const {
        for (std::size_t i = 1;; ++i) {
            const std::string name = "loc" + std::to_string(i);
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
