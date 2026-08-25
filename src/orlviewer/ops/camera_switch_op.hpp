#pragma once

#include <GLFW/glfw3.h>

#include "camera_navigator.hpp"
#include "vp_operation.hpp"

namespace ORL
{

// Numpad 1/3/7 snaps to Frame-relative orthogonal views (front / right / top).
class CameraSwitchOp : public VpOperation<CameraSwitchOp> {
public:
    explicit CameraSwitchOp(CameraNavigator& navigator)
        : navigator(navigator)
    {
    }

    void on_eval(const InputEvent& event) {
        if (event.kind != InputEvent::Kind::Key || event.action != GLFW_PRESS) {
            return;
        }
        switch (event.key) {
        case GLFW_KEY_KP_1:
            navigator.look_front();
            break;
        case GLFW_KEY_KP_3:
            navigator.look_right();
            break;
        case GLFW_KEY_KP_7:
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
