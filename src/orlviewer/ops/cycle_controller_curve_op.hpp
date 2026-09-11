#pragma once

#include <iostream>

#include "component_manager.hpp"
#include "selection.hpp"
#include "vp_operation.hpp"

namespace ORL
{

// Immediate Shift-C operation for cycling the selected controller's
// viewer-only display curve. The core controller xform is unchanged.
class CycleControllerCurveOp : public VpOperation<CycleControllerCurveOp> {
public:
    CycleControllerCurveOp(ComponentManager& components, Selection& selection)
        : components(components)
        , selection(selection)
    {
    }

    void on_eval(const InputEvent& event) {
        if (event.kind != InputEvent::Kind::Key
            || event.action != vkkk::InputAction::Press
            || event.key != vkkk::Key::C)
        {
            return;
        }

        const auto* focus = selection.focus();
        if (focus == nullptr || focus->kind != SelectionRef::Kind::Controller) {
            return;
        }
        const auto id = focus->component;
        const auto next = orlviewer::controller_curves::next(
            components.controller_shape(id));
        if (!components.set_controller_shape(id, next)) {
            return;
        }
        std::cout << "Controller curve: "
                  << orlviewer::controller_curves::name(next) << '\n';
    }

private:
    ComponentManager& components;
    Selection& selection;
};

} // namespace ORL
