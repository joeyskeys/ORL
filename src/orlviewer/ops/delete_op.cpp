#include "delete_op.hpp"

#include <cstddef>
#include <iostream>

namespace ORL
{

void DeleteOp::on_eval(const InputEvent& event) {
    if (event.kind != InputEvent::Kind::Key
        || event.action != vkkk::InputAction::Press)
    {
        return;
    }

    const auto selected = selection.refs();
    if (selected.empty()) {
        return;
    }

    selection.clear();
    std::size_t deleted = 0;
    for (const auto& ref : selected) {
        bool removed = false;
        switch (ref.kind) {
        case SelectionRef::Kind::SceneObject:
            removed = scene.remove_object(ref.object_name);
            break;
        case SelectionRef::Kind::Joint:
            removed = components.destroy_joint_recursive(ref.component);
            break;
        case SelectionRef::Kind::Controller:
        case SelectionRef::Kind::Locator:
            removed = components.destroy(ref.component);
            break;
        case SelectionRef::Kind::None:
        case SelectionRef::Kind::Vector:
            break;
        }
        if (removed) {
            ++deleted;
        }
    }

    if (deleted != 0) {
        std::cout << "Deleted " << deleted << " selected element"
                  << (deleted == 1 ? "" : "s") << '\n';
    }
}

} // namespace ORL
