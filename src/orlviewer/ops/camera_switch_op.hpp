#pragma once

#include "camera_navigator.hpp"
#include "gui/input.hpp"
#include "vp_operation.hpp"

namespace ORL
{

// Numpad 1/3/7 toggles Frame-relative orthogonal views (front / right / top).
// Pressing the active view key again returns to perspective.
class CameraSwitchOp : public VpOperation<CameraSwitchOp> {
public:
    explicit CameraSwitchOp(CameraNavigator& navigator)
        : navigator(navigator)
    {
    }

    void on_eval(const InputEvent& event) {
        if (event.kind != InputEvent::Kind::Key || event.action != vkkk::InputAction::Press) {
            return;
        }
        switch (event.key) {
        case vkkk::Key::Numpad1:
            navigator.look_front();
            break;
        case vkkk::Key::Numpad3:
            navigator.look_right();
            break;
        case vkkk::Key::Numpad7:
            navigator.look_top();
            break;
        default:
            break;
        }
    }

private:
    CameraNavigator& navigator;
};

} // namespace ORL
