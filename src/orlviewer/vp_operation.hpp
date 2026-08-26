#pragma once

#include "control_map.hpp"

namespace ORL
{

// CRTP base for viewport operations that ControlMap can dispatch.
// Immediate ops implement on_eval(). Modal ops also set kMode = OpMode::Modal
// and implement on_enter / on_confirm / on_cancel / is_active; ControlMap
// invokes those instead of extra per-gesture JSON bindings.
template <typename Derived>
class VpOperation {
public:
    static constexpr OpMode kMode = OpMode::Immediate;

    OpMode mode() const { return Derived::kMode; }

    bool active() const {
        return static_cast<const Derived*>(this)->is_active();
    }
    bool is_active() const { return false; }

    void enter() { static_cast<Derived*>(this)->on_enter(); }
    void confirm() { static_cast<Derived*>(this)->on_confirm(); }
    void cancel() { static_cast<Derived*>(this)->on_cancel(); }
    void eval(const InputEvent& event) {
        static_cast<Derived*>(this)->on_eval(event);
    }

    void on_enter() {}
    void on_confirm() {}
    void on_cancel() {}
};

} // namespace ORL
