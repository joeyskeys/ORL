#include "mirror_op.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include <glm/gtc/matrix_inverse.hpp>
#include <glm/vec4.hpp>

namespace ORL
{

MirrorOp::Axis MirrorOp::axis_for_key(vkkk::Key key) {
    switch (key) {
    case vkkk::Key::X:
        return Axis::X;
    case vkkk::Key::Y:
        return Axis::Y;
    case vkkk::Key::Z:
        return Axis::Z;
    default:
        return Axis::None;
    }
}

glm::vec3 MirrorOp::mirrored_position(
    const glm::vec3& position, Axis axis)
{
    glm::vec3 mirrored = position;
    switch (axis) {
    case Axis::X:
        mirrored.x = -mirrored.x;
        break;
    case Axis::Y:
        mirrored.y = -mirrored.y;
        break;
    case Axis::Z:
        mirrored.z = -mirrored.z;
        break;
    case Axis::None:
        break;
    }
    return mirrored;
}

void MirrorOp::on_enter() {
    active_ = true;
    std::cout << "Mirror: press X, Y, or Z to choose the mirror plane\n";
}

void MirrorOp::on_eval(const InputEvent& event) {
    if (!active_
        || event.kind != InputEvent::Kind::Key
        || event.action != vkkk::InputAction::Press)
    {
        return;
    }

    const auto axis = axis_for_key(event.key);
    if (axis == Axis::None) {
        return;
    }
    mirror(axis);
    active_ = false;
}

void MirrorOp::on_confirm() {
    // Mirroring is confirmed by the axis key, not by a mouse click.
}

void MirrorOp::on_cancel() {
    if (active_) {
        std::cout << "Mirror: cancelled\n";
    }
    active_ = false;
}

std::string MirrorOp::unique_joint_name(std::string_view source) const {
    const std::string base = std::string{source} + "_mirror";
    for (std::size_t suffix = 1;; ++suffix) {
        const std::string candidate = suffix == 1
            ? base
            : base + std::to_string(suffix);
        if (!components.contains(candidate)) {
            return candidate;
        }
    }
}

bool MirrorOp::mirror(Axis axis) {
    const auto& refs = selection.refs();
    if (refs.empty()) {
        std::cout << "Mirror: select one or more joints first\n";
        return false;
    }

    std::vector<ComponentId> selected_ids;
    selected_ids.reserve(refs.size());
    for (const auto& ref : refs) {
        if (ref.kind != SelectionRef::Kind::Joint) {
            std::cout << "Mirror: only joint selections are supported\n";
            return false;
        }
        if (std::find(selected_ids.begin(), selected_ids.end(),
                ref.component) == selected_ids.end())
        {
            selected_ids.push_back(ref.component);
        }
    }

    const auto packed = components.packed_joints();
    const auto ids = components.packed_joint_ids();
    if (packed.empty() || packed.size() != ids.size()) {
        std::cout << "Mirror: no valid joints are selected\n";
        return false;
    }

    std::vector<bool> copied(packed.size(), false);
    for (const auto id : selected_ids) {
        const auto found = std::find(ids.begin(), ids.end(), id);
        if (found == ids.end()) {
            std::cout << "Mirror: selection contains an unavailable joint\n";
            return false;
        }
        copied[static_cast<std::size_t>(
            std::distance(ids.begin(), found))] = true;
    }

    // Selecting a joint also mirrors its complete descendant subtree.
    bool changed = true;
    while (changed) {
        changed = false;
        for (std::size_t index = 0; index < packed.size(); ++index) {
            if (copied[index]) {
                continue;
            }
            const auto parent = packed[index].parent;
            if (parent >= 0
                && static_cast<std::size_t>(parent) < copied.size()
                && copied[static_cast<std::size_t>(parent)])
            {
                copied[index] = true;
                changed = true;
            }
        }
    }

    std::vector<std::size_t> copy_indices;
    copy_indices.reserve(packed.size());
    for (std::size_t index = 0; index < packed.size(); ++index) {
        if (copied[index]) {
            copy_indices.push_back(index);
        }
    }

    const auto* focus = selection.focus();
    const auto focus_id = focus != nullptr
        ? focus->component : ComponentId{};
    const auto focus_found = std::find(ids.begin(), ids.end(), focus_id);
    const std::size_t focus_index = focus_found == ids.end()
        ? packed.size()
        : static_cast<std::size_t>(
            std::distance(ids.begin(), focus_found));

    std::vector<ComponentId> copied_ids(packed.size());
    std::vector<ComponentId> created;
    created.reserve(copy_indices.size());

    const auto rollback = [&] {
        for (auto it = created.rbegin(); it != created.rend(); ++it) {
            components.destroy_joint_recursive(*it);
        }
    };

    std::vector<std::size_t> pending = copy_indices;
    while (!pending.empty()) {
        bool progress = false;
        for (auto it = pending.begin(); it != pending.end();) {
            const std::size_t index = *it;
            const auto original_parent = packed[index].parent;
            if (original_parent >= 0
                && static_cast<std::size_t>(original_parent) < copied.size()
                && copied[static_cast<std::size_t>(original_parent)]
                && !copied_ids[static_cast<std::size_t>(original_parent)])
            {
                ++it;
                continue;
            }

            std::int64_t parent_index = -1;
            if (original_parent >= 0
                && static_cast<std::size_t>(original_parent) < packed.size())
            {
                if (copied[static_cast<std::size_t>(original_parent)]) {
                    const auto parent_id =
                        copied_ids[static_cast<std::size_t>(original_parent)];
                    parent_index = components.joint_index(parent_id);
                    if (parent_index < 0) {
                        rollback();
                        return false;
                    }
                }
                else {
                    parent_index = original_parent;
                }
            }

            auto current = components.packed_joints();
            if (parent_index >= 0
                && static_cast<std::size_t>(parent_index) >= current.size())
            {
                rollback();
                return false;
            }

            const glm::vec3 original_world = glm::vec3{
                orlviewer::joint_world_matrix(packed, index)[3]};
            const glm::vec3 mirrored_world =
                mirrored_position(original_world, axis);
            const glm::vec3 local = orlviewer::world_to_local(
                current, parent_index, mirrored_world);

            auto joint = packed[index];
            joint.parent = parent_index;
            joint.selected = 0;
            joint.translation[0] = local.x;
            joint.translation[1] = local.y;
            joint.translation[2] = local.z;
            joint.translation[3] = 0.0;

            const auto* meta = components.find(ids[index]);
            const std::string source_name = meta != nullptr
                ? meta->name : std::string{"joint"};
            const auto id = components.create_joint(
                unique_joint_name(source_name), std::move(joint));
            if (!id) {
                rollback();
                return false;
            }

            copied_ids[index] = id;
            created.push_back(id);
            it = pending.erase(it);
            progress = true;
        }

        if (!progress) {
            rollback();
            std::cout << "Mirror: joint hierarchy contains an invalid cycle\n";
            return false;
        }
    }

    selection.clear();
    for (const auto index : copy_indices) {
        if (index == focus_index) {
            continue;
        }
        selection.add(SelectionRef::joint(copied_ids[index]));
    }
    if (focus_index < copied_ids.size()
        && copied_ids[focus_index])
    {
        selection.add(SelectionRef::joint(copied_ids[focus_index]));
    }

    std::cout << "Mirrored " << created.size() << " joint"
              << (created.size() == 1 ? "" : "s") << '\n';
    return true;
}

} // namespace ORL
