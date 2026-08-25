#pragma once

#include <functional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "vp_operation.hpp"

namespace ORL
{

class DisplayModeSwitch : public VpOperation<DisplayModeSwitch> {
public:
    using ApplyFn = std::function<void()>;

    void register_mode(std::string name, ApplyFn apply) {
        for (auto& mode : modes) {
            if (mode.name == name) {
                mode.apply = std::move(apply);
                return;
            }
        }
        modes.push_back(Mode{std::move(name), std::move(apply)});
        if (current.empty() && !modes.empty()) {
            set_mode(modes.front().name);
        }
    }

    bool set_mode(std::string_view name) {
        for (const auto& mode : modes) {
            if (mode.name != name || !mode.apply) {
                continue;
            }
            current = mode.name;
            mode.apply();
            return true;
        }
        return false;
    }

    std::string_view current_mode() const { return current; }

    void on_eval(const InputEvent& event) {
        const int index = mode_index_from_key(event.key);
        if (index < 0 || static_cast<std::size_t>(index) >= modes.size()) {
            return;
        }
        set_mode(modes[static_cast<std::size_t>(index)].name);
    }

private:
    struct Mode {
        std::string name;
        ApplyFn apply;
    };

    static int mode_index_from_key(int key) {
        if (key >= GLFW_KEY_1 && key <= GLFW_KEY_9) {
            return key - GLFW_KEY_1;
        }
        return -1;
    }

    std::vector<Mode> modes;
    std::string current;
};

} // namespace ORL
