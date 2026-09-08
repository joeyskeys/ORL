#pragma once

#include <array>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <string>
#include <string_view>
#include <vector>

#include "gui/input.hpp"
#include "gui/window_backend.hpp"

#ifndef ORL_USE_QT6
#define ORL_USE_QT6 0
#endif

#if !ORL_USE_QT6
#include "gui/glfw_backend.hpp"
#endif

namespace ORL
{

using InputEvent = vkkk::InputEvent;

struct InputSpec {
    enum class Type {
        Key,
        MouseButton,
        MouseDrag,
        Scroll,
    };

    static constexpr int kHold = -1;

    Type type = Type::Key;
    int code = 0;
    int action = static_cast<int>(vkkk::InputAction::Press);
    std::uint32_t mods = 0;
};

struct ControlBinding {
    InputSpec input;
    std::vector<std::string> ops;
};

enum class ControlMapLoadMode {
    Replace,
    Merge,
};

enum class OpMode {
    Immediate,
    Modal,
};

// Maps named viewport operations to keyboard and mouse inputs.
// Bindings can be loaded from JSON and changed at runtime. A binding's
// `op` may be a string or an array; one trigger invokes every named op
// in order. Operation handlers are registered in code and invoked from
// GLFW callbacks or Qt per-frame polling, depending on the viewer backend.
class ControlMap {
public:
    using OpHandler = std::function<void(const InputEvent&)>;

    ControlMap() = default;
    ControlMap(const ControlMap&) = delete;
    ControlMap& operator=(const ControlMap&) = delete;
    ControlMap(ControlMap&&) = delete;
    ControlMap& operator=(ControlMap&&) = delete;
    ~ControlMap();

    void attach(vkkk::WindowBackend& backend);
    void detach();
    vkkk::WindowBackend* window() const { return backend_; }

    void bind_op(std::string op, OpHandler handler);

    // Bind any viewport operation that exposes eval(const InputEvent&),
    // including VpOperation<Derived> CRTP types. Modal ops also expose
    // mode()/enter()/confirm()/cancel()/active(); ControlMap calls those
    // instead of stuffing modal input into extra JSON bindings.
    template <typename Op>
    void bind_op(std::string op, Op& operation) {
        unbind_op(op);
        BoundOp bound;
        bound.name = std::move(op);
        bound.eval = [&operation](const InputEvent& event) {
            operation.eval(event);
        };
        if constexpr (requires { operation.mode(); }) {
            bound.mode = operation.mode();
        }
        if constexpr (requires { operation.enter(); }) {
            bound.enter = [&operation] { operation.enter(); };
        }
        if constexpr (requires { operation.confirm(); }) {
            bound.confirm = [&operation] { operation.confirm(); };
        }
        if constexpr (requires { operation.cancel(); }) {
            bound.cancel = [&operation] { operation.cancel(); };
        }
        if constexpr (requires { operation.active(); }) {
            bound.active = [&operation] { return operation.active(); };
        }
        ops.push_back(std::move(bound));
    }

    void unbind_op(std::string_view op);
    bool has_op(std::string_view op) const;

    void map(InputSpec input, std::string op);
    void map(InputSpec input, std::vector<std::string> ops);
    void unmap(const InputSpec& input);
    void unmap_op(std::string_view op);
    void clear_bindings();

    void load_config(const std::filesystem::path& path,
        ControlMapLoadMode mode = ControlMapLoadMode::Replace);

    const std::vector<ControlBinding>& bindings() const { return bindings_; }

    // Dispatch hold-style key bindings, and Qt edge events when that backend
    // is compiled in. Call once per frame after WindowBackend::poll_events.
    void poll();

    static int parse_key(std::string_view name);
    static int parse_mouse_button(std::string_view name);
    static std::uint32_t parse_mods(const std::vector<std::string>& names);
    static int parse_action(std::string_view name);

private:
    struct BoundOp {
        std::string name;
        OpMode mode = OpMode::Immediate;
        std::function<void(const InputEvent&)> eval;
        std::function<void()> enter;
        std::function<void()> confirm;
        std::function<void()> cancel;
        std::function<bool()> active;
    };

    void dispatch(const InputEvent& event);
    void invoke_modal(BoundOp& op, const InputEvent& event);
    BoundOp* find_op(std::string_view name);
    bool matches(const InputSpec& spec, const InputEvent& event) const;
    std::uint32_t current_mods() const;
    void sync_cursor();
    void poll_holds();

#if ORL_USE_QT6
    static constexpr std::size_t kKeyCount =
        static_cast<std::size_t>(vkkk::Key::NumpadSubtract) + 1;

    void poll_qt_events();
    void snapshot_qt_state();

    std::array<bool, kKeyCount> prev_keys_{};
    bool prev_mouse_[3] = {};
    bool qt_synced_ = false;
#else
    static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);
    static void mouse_button_callback(GLFWwindow* window, int button, int action, int mods);
    static void cursor_pos_callback(GLFWwindow* window, double x, double y);
    static void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
    static ControlMap* map_for(GLFWwindow* window);

    GLFWwindow* glfw_window_ = nullptr;
#endif

    vkkk::WindowBackend* backend_ = nullptr;
    std::vector<ControlBinding> bindings_;
    std::vector<BoundOp> ops;
    std::string modal;
    unsigned buttons_down_ = 0;
    bool cursor_valid_ = false;
    double cursor_x_ = 0.0;
    double cursor_y_ = 0.0;
};

} // namespace ORL
