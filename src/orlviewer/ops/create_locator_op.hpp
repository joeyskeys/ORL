#pragma once

#include <iostream>
#include <string>
#include <vector>

#include "camera_navigator.hpp"
#include "component_manager.hpp"
#include "gui/window_backend.hpp"
#include "selection.hpp"
#include "vp_operation.hpp"

namespace ORL
{

// Modal locator placement: L creates a locator under the cursor, mouse motion
// moves it on the current view plane, left-click confirms it, and right-click
// or Escape cancels and removes it.
class CreateLocatorOp : public VpOperation<CreateLocatorOp> {
public:
    static constexpr OpMode kMode = OpMode::Modal;

    CreateLocatorOp(ComponentManager& components, CameraNavigator& navigator,
        vkkk::WindowBackend* window, Selection& selection)
        : components(components)
        , navigator(navigator)
        , window(window)
        , selection(selection)
    {
    }

    bool is_active() const { return active_; }

    void on_enter() {
        if (active_ || window == nullptr) {
            return;
        }

        previous_selection = selection.refs();
        const auto pointer = window->pointer();
        glm::vec3 world{};
        if (!hit(pointer.x, pointer.y, world)) {
            previous_selection.clear();
            return;
        }

        const auto id = components.create_locator(
            unique_name(), orlrig::make_locator(world));
        if (!id) {
            previous_selection.clear();
            std::cerr << "CreateLocatorOp: failed to create locator\n";
            return;
        }
        created_ = id;
        selection.replace(SelectionRef::locator(created_));
        active_ = true;
        std::cout << "Locator create: click to place, right-click or Escape to cancel\n";
    }

    void on_confirm() {
        if (!active_) {
            return;
        }

        // The Qt polling backend can deliver a button edge before its
        // per-frame cursor-move event. Sample the backend again so the
        // confirmed position is exactly under the click.
        const auto pointer = window != nullptr ? window->pointer()
                                               : vkkk::InputPointer{};
        update(pointer.x, pointer.y);
        active_ = false;
        created_ = {};
        previous_selection.clear();
        std::cout << "Locator create: placed\n";
    }

    void on_cancel() {
        if (!active_) {
            return;
        }

        const ComponentId created = created_;
        active_ = false;
        created_ = {};
        if (created) {
            components.destroy(created);
        }
        selection.clear();
        for (const auto& ref : previous_selection) {
            selection.add(ref);
        }
        previous_selection.clear();
        std::cout << "Locator create: cancelled\n";
    }

    void on_eval(const InputEvent& event) {
        if (!active_ || event.kind != InputEvent::Kind::MouseMove) {
            return;
        }
        update(event.x, event.y);
    }

private:
    bool hit(double cursor_x, double cursor_y, glm::vec3& world) const {
        if (window == nullptr) {
            return false;
        }
        const auto size = window->window_size();
        return navigator.view_plane_hit(
            cursor_x, cursor_y,
            static_cast<int>(size.width), static_cast<int>(size.height),
            navigator.target, world);
    }

    void update(double cursor_x, double cursor_y) {
        if (!active_ || !created_) {
            return;
        }
        glm::vec3 world{};
        if (hit(cursor_x, cursor_y, world)) {
            components.set_locator_world_xform(
                created_, orlrig::make_locator(world).xform);
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
    CameraNavigator& navigator;
    vkkk::WindowBackend* window = nullptr;
    Selection& selection;
    bool active_ = false;
    ComponentId created_;
    std::vector<SelectionRef> previous_selection;
};

} // namespace ORL
