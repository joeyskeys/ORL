#pragma once

#include <algorithm>
#include <iostream>
#include <string>

#include <glm/geometric.hpp>
#include <glm/vec3.hpp>

#include "../../orlexec/orlrig/ik.hpp"
#include "component_manager.hpp"
#include "selection.hpp"
#include "vp_operation.hpp"

namespace ORL
{

// Immediate two-bone IK: select the end joint, press I. Creates a curve
// target and polygon pole the user can move with G. The solver owns
// root/mid rotation.
class CreateIkOp : public VpOperation<CreateIkOp> {
public:
    CreateIkOp(ComponentManager& components, Selection& selection)
        : components(components)
        , selection(selection)
    {
    }

    void on_eval(const InputEvent&) {
        const ComponentId end_id = selected_joint();
        if (!end_id) {
            std::cerr << "IK: select the end joint of a two-bone chain\n";
            return;
        }
        if (has_ik(end_id)) {
            std::cerr << "IK: that end already has a two-bone solver\n";
            return;
        }

        const auto end_index = components.joint_index(end_id);
        const auto packed = components.packed_joints();
        if (end_index < 0 || static_cast<std::size_t>(end_index) >= packed.size()) {
            std::cerr << "IK: select the end joint of a two-bone chain\n";
            return;
        }

        const auto chain = orlrig::make_two_bone_chain(packed, end_index);
        if (!chain.has_value()) {
            std::cerr << "IK: end joint needs a parent and grandparent\n";
            return;
        }

        const auto ids = components.packed_joint_ids();
        const ComponentId root_id = ids[static_cast<std::size_t>(chain->root)];
        const ComponentId mid_id = ids[static_cast<std::size_t>(chain->mid)];

        const glm::vec3 root_w = chain->root_world;
        const glm::vec3 mid_w = chain->mid_world;
        const glm::vec3 end_w = chain->end_world;

        const auto target_id = create_handle("ik_target", end_w, orlviewer::ControllerShape::Curve);
        const auto pole_id = create_handle("ik_pole",
            orlrig::pole_position(root_w, mid_w, end_w),
            orlviewer::ControllerShape::Polygon);
        if (!target_id || !pole_id) {
            std::cerr << "IK: failed to create controllers\n";
            return;
        }

        ConstraintData data;
        data.type = "ik_two_bone";
        data.root = root_id;
        data.mid = mid_id;
        data.end = end_id;
        data.target = target_id;
        data.pole = pole_id;
        data.bound = true;

        const auto constraint_id = components.create_constraint(unique_name("ik_two_bone"), data);
        if (!constraint_id) {
            std::cerr << "IK: failed to create solver\n";
            return;
        }

        selection.set(SelectionRef::controller(target_id));
        const auto* end_meta = components.find(end_id);
        const auto* target_meta = components.find(target_id);
        const auto* pole_meta = components.find(pole_id);
        std::cout << "IK two-bone: "
            << (end_meta != nullptr ? end_meta->name : "?")
            << " -> " << (target_meta != nullptr ? target_meta->name : "?")
            << " (pole " << (pole_meta != nullptr ? pole_meta->name : "?") << ")\n";
    }

private:
    ComponentId selected_joint() const {
        const auto* focus = selection.focus();
        if (focus != nullptr && focus->kind == SelectionRef::Kind::Joint) {
            return focus->component;
        }
        ComponentId id{};
        for (const auto& ref : selection.refs()) {
            if (ref.kind == SelectionRef::Kind::Joint) {
                id = ref.component;
            }
        }
        return id;
    }

    bool has_ik(ComponentId end) const {
        bool found = false;
        components.for_each([&](const Component& meta) {
            if (meta.kind != ComponentKind::Constraint) {
                return;
            }
            const auto* data = components.constraint(meta.id);
            if (data != nullptr && data->bound && data->end == end) {
                found = true;
            }
        });
        return found;
    }

    std::string unique_name(const char* prefix) const {
        for (std::size_t i = 1;; ++i) {
            std::string name = std::string{prefix} + std::to_string(i);
            if (!components.contains(name)) {
                return name;
            }
        }
    }

    ComponentId create_handle(const char* prefix, const glm::vec3& world,
        orlviewer::ControllerShape shape)
    {
        return components.create_controller(unique_name(prefix),
            orlviewer::make_controller(world, shape));
    }

    ComponentManager& components;
    Selection& selection;
};

} // namespace ORL
