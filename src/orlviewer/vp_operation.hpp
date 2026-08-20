#pragma once

#include "control_map.hpp"

namespace ORL
{

// CRTP base for viewport operations that ControlMap can dispatch.
// Derived types implement on_eval(const InputEvent&); callers use eval().
template <typename Derived>
class VpOperation {
public:
    void eval(const InputEvent& event) {
        static_cast<Derived*>(this)->on_eval(event);
    }
};

} // namespace ORL
