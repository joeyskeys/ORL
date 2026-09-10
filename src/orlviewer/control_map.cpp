#include "control_map.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <utility>

#include <rapidjson/document.h>
#include <rapidjson/error/en.h>

#if ORL_USE_QT6
#include "gui/qt_backend.hpp"
#endif

namespace ORL
{
namespace
{

#if !ORL_USE_QT6
std::unordered_map<GLFWwindow*, ControlMap*> g_window_maps;
#endif

std::string to_lower(std::string_view text) {
    std::string out(text);
    for (char& c : out) {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    return out;
}

const std::unordered_map<std::string, vkkk::Key> kNamedKeys = {
    {"space", vkkk::Key::Space},
    {"escape", vkkk::Key::Escape},
    {"esc", vkkk::Key::Escape},
    {"enter", vkkk::Key::Enter},
    {"return", vkkk::Key::Enter},
    {"tab", vkkk::Key::Tab},
    {"backspace", vkkk::Key::Backspace},
    {"delete", vkkk::Key::Delete},
    {"insert", vkkk::Key::Insert},
    {"home", vkkk::Key::Home},
    {"end", vkkk::Key::End},
    {"pageup", vkkk::Key::PageUp},
    {"pagedown", vkkk::Key::PageDown},
    {"left", vkkk::Key::Left},
    {"right", vkkk::Key::Right},
    {"up", vkkk::Key::Up},
    {"down", vkkk::Key::Down},
    {"leftshift", vkkk::Key::LeftShift},
    {"rightshift", vkkk::Key::RightShift},
    {"leftctrl", vkkk::Key::LeftCtrl},
    {"leftcontrol", vkkk::Key::LeftCtrl},
    {"rightctrl", vkkk::Key::RightCtrl},
    {"leftalt", vkkk::Key::LeftAlt},
    {"rightalt", vkkk::Key::RightAlt},
    {"leftsuper", vkkk::Key::LeftSuper},
    {"rightsuper", vkkk::Key::RightSuper},
    {"minus", vkkk::Key::Minus},
    {"equal", vkkk::Key::Equal},
    {"comma", vkkk::Key::Comma},
    {"period", vkkk::Key::Period},
    {"slash", vkkk::Key::Slash},
    {"semicolon", vkkk::Key::Semicolon},
    {"apostrophe", vkkk::Key::Apostrophe},
    {"grave", vkkk::Key::Grave},
    {"leftbracket", vkkk::Key::LeftBracket},
    {"rightbracket", vkkk::Key::RightBracket},
    {"backslash", vkkk::Key::Backslash},
    {"kpenter", vkkk::Key::NumpadEnter},
    {"numpadenter", vkkk::Key::NumpadEnter},
    {"kpadd", vkkk::Key::NumpadAdd},
    {"kpsubtract", vkkk::Key::NumpadSubtract},
};

std::uint32_t parse_mod_name(std::string_view name) {
    const std::string key = to_lower(name);
    if (key == "shift") {
        return vkkk::input_mod::shift;
    }
    if (key == "ctrl" || key == "control") {
        return vkkk::input_mod::ctrl;
    }
    if (key == "alt") {
        return vkkk::input_mod::alt;
    }
    if (key == "super" || key == "cmd" || key == "meta") {
        return vkkk::input_mod::super;
    }
    throw std::runtime_error("unknown modifier '" + std::string(name) + "'");
}

InputSpec::Type parse_input_type(std::string_view name) {
    const std::string key = to_lower(name);
    if (key == "key") {
        return InputSpec::Type::Key;
    }
    if (key == "mouse_button" || key == "mouse-button") {
        return InputSpec::Type::MouseButton;
    }
    if (key == "mouse_drag" || key == "mouse-drag" || key == "drag") {
        return InputSpec::Type::MouseDrag;
    }
    if (key == "scroll" || key == "wheel") {
        return InputSpec::Type::Scroll;
    }
    throw std::runtime_error("unknown input type '" + std::string(name) + "'");
}

std::vector<std::string> read_mod_names(const rapidjson::Value& value) {
    std::vector<std::string> names;
    if (!value.HasMember("mods")) {
        return names;
    }
    const auto& mods = value["mods"];
    if (mods.IsString()) {
        names.emplace_back(mods.GetString());
        return names;
    }
    if (!mods.IsArray()) {
        throw std::runtime_error("'mods' must be a string or array of strings");
    }
    for (const auto& mod : mods.GetArray()) {
        if (!mod.IsString()) {
            throw std::runtime_error("modifier entries must be strings");
        }
        names.emplace_back(mod.GetString());
    }
    return names;
}

InputSpec parse_input_spec(const rapidjson::Value& value) {
    if (!value.IsObject() || !value.HasMember("type") || !value["type"].IsString()) {
        throw std::runtime_error("binding 'input' must be an object with a 'type' string");
    }

    InputSpec spec;
    spec.type = parse_input_type(value["type"].GetString());
    spec.mods = ControlMap::parse_mods(read_mod_names(value));

    switch (spec.type) {
    case InputSpec::Type::Key: {
        if (!value.HasMember("key") || !value["key"].IsString()) {
            throw std::runtime_error("key bindings require a 'key' string");
        }
        spec.code = ControlMap::parse_key(value["key"].GetString());
        if (value.HasMember("action")) {
            if (!value["action"].IsString()) {
                throw std::runtime_error("key 'action' must be a string");
            }
            spec.action = ControlMap::parse_action(value["action"].GetString());
        }
        break;
    }
    case InputSpec::Type::MouseButton:
    case InputSpec::Type::MouseDrag: {
        if (!value.HasMember("button") || !value["button"].IsString()) {
            throw std::runtime_error("mouse bindings require a 'button' string");
        }
        spec.code = ControlMap::parse_mouse_button(value["button"].GetString());
        if (spec.type == InputSpec::Type::MouseButton && value.HasMember("action")) {
            if (!value["action"].IsString()) {
                throw std::runtime_error("mouse_button 'action' must be a string");
            }
            spec.action = ControlMap::parse_action(value["action"].GetString());
        }
        break;
    }
    case InputSpec::Type::Scroll:
        break;
    }
    return spec;
}

} // namespace

int ControlMap::parse_key(std::string_view name) {
    if (name.empty()) {
        throw std::runtime_error("empty key name");
    }
    if (name.size() == 1) {
        const char c = static_cast<char>(std::toupper(static_cast<unsigned char>(name[0])));
        if (c >= 'A' && c <= 'Z') {
            return static_cast<int>(vkkk::Key::A) + (c - 'A');
        }
        if (c >= '0' && c <= '9') {
            return static_cast<int>(vkkk::Key::Digit0) + (c - '0');
        }
    }

    std::string key = to_lower(name);
    key.erase(std::remove(key.begin(), key.end(), '_'), key.end());
    key.erase(std::remove(key.begin(), key.end(), '-'), key.end());
    if (key.size() >= 2 && key.front() == 'f') {
        bool digits = true;
        for (size_t i = 1; i < key.size(); ++i) {
            digits = digits && std::isdigit(static_cast<unsigned char>(key[i]));
        }
        if (digits) {
            const int index = std::stoi(key.substr(1));
            if (index >= 1 && index <= 12) {
                return static_cast<int>(vkkk::Key::F1) + (index - 1);
            }
        }
    }

    std::string keypad;
    if (key.rfind("numpad", 0) == 0) {
        keypad = key.substr(6);
    }
    else if (key.rfind("kp", 0) == 0) {
        keypad = key.substr(2);
    }
    else if (key.rfind("np", 0) == 0) {
        keypad = key.substr(2);
    }
    if (keypad.size() == 1 && keypad[0] >= '0' && keypad[0] <= '9') {
        return static_cast<int>(vkkk::Key::Numpad0) + (keypad[0] - '0');
    }

    const auto it = kNamedKeys.find(key);
    if (it == kNamedKeys.end()) {
        throw std::runtime_error("unknown key '" + std::string(name) + "'");
    }
    return static_cast<int>(it->second);
}

int ControlMap::parse_mouse_button(std::string_view name) {
    const std::string key = to_lower(name);
    if (key == "left" || key == "0") {
        return static_cast<int>(vkkk::MouseButton::Left);
    }
    if (key == "right" || key == "1") {
        return static_cast<int>(vkkk::MouseButton::Right);
    }
    if (key == "middle" || key == "2") {
        return static_cast<int>(vkkk::MouseButton::Middle);
    }
    throw std::runtime_error("unknown mouse button '" + std::string(name) + "'");
}

std::uint32_t ControlMap::parse_mods(const std::vector<std::string>& names) {
    std::uint32_t mods = 0;
    for (const auto& name : names) {
        mods |= parse_mod_name(name);
    }
    return mods;
}

int ControlMap::parse_action(std::string_view name) {
    const std::string key = to_lower(name);
    if (key == "press" || key == "down") {
        return static_cast<int>(vkkk::InputAction::Press);
    }
    if (key == "release" || key == "up") {
        return static_cast<int>(vkkk::InputAction::Release);
    }
    if (key == "repeat") {
        return static_cast<int>(vkkk::InputAction::Repeat);
    }
    if (key == "hold") {
        return InputSpec::kHold;
    }
    throw std::runtime_error("unknown action '" + std::string(name) + "'");
}

ControlMap::~ControlMap() {
    detach();
}

void ControlMap::sync_cursor() {
    if (backend_ == nullptr) {
        return;
    }
    const auto pointer = backend_->pointer();
    cursor_x_ = pointer.x;
    cursor_y_ = pointer.y;
    cursor_valid_ = true;
}

void ControlMap::attach(vkkk::WindowBackend& backend) {
    detach();
    backend_ = &backend;
    sync_cursor();

#if ORL_USE_QT6
    qt_synced_ = false;
    snapshot_qt_state();
#else
    auto* glfw = dynamic_cast<vkkk::GlfwBackend*>(backend_);
    if (glfw == nullptr) {
        return;
    }
    glfw_window_ = glfw->glfw_window();
    if (glfw_window_ == nullptr) {
        return;
    }
    g_window_maps[glfw_window_] = this;
    glfwSetKeyCallback(glfw_window_, key_callback);
    glfwSetMouseButtonCallback(glfw_window_, mouse_button_callback);
    glfwSetCursorPosCallback(glfw_window_, cursor_pos_callback);
    glfwSetScrollCallback(glfw_window_, scroll_callback);
#endif
}

void ControlMap::detach() {
#if ORL_USE_QT6
    qt_synced_ = false;
    prev_keys_.fill(false);
    prev_mouse_[0] = prev_mouse_[1] = prev_mouse_[2] = false;
#else
    if (glfw_window_ != nullptr) {
        const auto it = g_window_maps.find(glfw_window_);
        if (it != g_window_maps.end() && it->second == this) {
            glfwSetKeyCallback(glfw_window_, nullptr);
            glfwSetMouseButtonCallback(glfw_window_, nullptr);
            glfwSetCursorPosCallback(glfw_window_, nullptr);
            glfwSetScrollCallback(glfw_window_, nullptr);
            g_window_maps.erase(it);
        }
        glfw_window_ = nullptr;
    }
#endif
    backend_ = nullptr;
    modal.clear();
    buttons_down_ = 0;
    cursor_valid_ = false;
}

void ControlMap::bind_op(std::string op, OpHandler handler) {
    unbind_op(op);
    BoundOp bound;
    bound.name = std::move(op);
    bound.eval = std::move(handler);
    ops.push_back(std::move(bound));
}

void ControlMap::unbind_op(std::string_view op) {
    if (modal == op) {
        modal.clear();
    }
    ops.erase(std::remove_if(ops.begin(), ops.end(),
                  [op](const BoundOp& bound) { return bound.name == op; }),
        ops.end());
}

bool ControlMap::has_op(std::string_view op) const {
    return std::any_of(ops.begin(), ops.end(),
        [op](const BoundOp& bound) { return bound.name == op; });
}

void ControlMap::map(InputSpec input, std::string op) {
    map(input, std::vector<std::string>{std::move(op)});
}

void ControlMap::map(InputSpec input, std::vector<std::string> ops) {
    bindings_.push_back(ControlBinding{input, std::move(ops)});
}

void ControlMap::unmap(const InputSpec& input) {
    bindings_.erase(std::remove_if(bindings_.begin(), bindings_.end(),
                        [&input](const ControlBinding& binding) {
                            return binding.input.type == input.type
                                && binding.input.code == input.code
                                && binding.input.action == input.action
                                && binding.input.mods == input.mods;
                        }),
        bindings_.end());
}

void ControlMap::unmap_op(std::string_view op) {
    for (auto& binding : bindings_) {
        binding.ops.erase(std::remove(binding.ops.begin(), binding.ops.end(), op),
            binding.ops.end());
    }
    bindings_.erase(std::remove_if(bindings_.begin(), bindings_.end(),
                        [](const ControlBinding& binding) { return binding.ops.empty(); }),
        bindings_.end());
}

void ControlMap::clear_bindings() {
    bindings_.clear();
}

void ControlMap::load_config(const std::filesystem::path& path, ControlMapLoadMode mode) {
    std::ifstream file(path, std::ios::binary);
    if (!file) {
        throw std::runtime_error("failed to open control map config '" + path.string() + "'");
    }
    std::ostringstream buffer;
    buffer << file.rdbuf();
    const std::string text = buffer.str();

    rapidjson::Document document;
    document.Parse(text.c_str());
    if (document.HasParseError()) {
        throw std::runtime_error(std::string("invalid control map JSON: ")
            + rapidjson::GetParseError_En(document.GetParseError()));
    }
    if (!document.IsObject() || !document.HasMember("bindings") || !document["bindings"].IsArray()) {
        throw std::runtime_error("control map JSON must contain a 'bindings' array");
    }

    if (mode == ControlMapLoadMode::Replace) {
        clear_bindings();
    }

    for (const auto& binding : document["bindings"].GetArray()) {
        if (!binding.IsObject() || !binding.HasMember("op")) {
            throw std::runtime_error("each binding needs an 'op' string or array");
        }
        if (!binding.HasMember("input")) {
            throw std::runtime_error("each binding needs an 'input' object");
        }

        std::vector<std::string> ops;
        const auto& op = binding["op"];
        if (op.IsString()) {
            ops.emplace_back(op.GetString());
        }
        else if (op.IsArray()) {
            for (const auto& name : op.GetArray()) {
                if (!name.IsString()) {
                    throw std::runtime_error("each 'op' entry must be a string");
                }
                ops.emplace_back(name.GetString());
            }
        }
        else {
            throw std::runtime_error("binding 'op' must be a string or array of strings");
        }
        if (ops.empty()) {
            throw std::runtime_error("each binding needs at least one op");
        }
        map(parse_input_spec(binding["input"]), std::move(ops));
    }
}

void ControlMap::poll_holds() {
    if (backend_ == nullptr) {
        return;
    }

    const std::uint32_t mods = current_mods();
    for (const auto& binding : bindings_) {
        if (binding.input.type != InputSpec::Type::Key
            || binding.input.action != InputSpec::kHold)
        {
            continue;
        }
        if (binding.input.mods != mods) {
            continue;
        }
        const auto key = static_cast<vkkk::Key>(binding.input.code);
        if (!backend_->key_down(key)) {
            continue;
        }

        InputEvent event;
        event.kind = InputEvent::Kind::Hold;
        event.key = key;
        event.mods = mods;
        event.x = cursor_x_;
        event.y = cursor_y_;
        dispatch(event);
    }
}

#if ORL_USE_QT6

bool qt_key_active(vkkk::WindowBackend& backend, vkkk::Key key) {
    switch (key) {
    case vkkk::Key::RightShift:
    case vkkk::Key::RightCtrl:
    case vkkk::Key::RightAlt:
    case vkkk::Key::RightSuper:
        return false;
    default:
        break;
    }
    if (!backend.key_down(key)) {
        return false;
    }
    return true;
}

void ControlMap::snapshot_qt_state() {
    if (backend_ == nullptr) {
        return;
    }
    for (std::size_t i = 1; i < kKeyCount; ++i) {
        prev_keys_[i] = qt_key_active(*backend_, static_cast<vkkk::Key>(i));
    }
    buttons_down_ = 0;
    for (int i = 0; i < 3; ++i) {
        const auto button = static_cast<vkkk::MouseButton>(i);
        prev_mouse_[i] = backend_->mouse_down(button);
        if (prev_mouse_[i]) {
            buttons_down_ |= (1u << i);
        }
    }
    sync_cursor();
    qt_synced_ = true;
}

void ControlMap::poll_qt_events() {
    if (backend_ == nullptr) {
        return;
    }
    if (!qt_synced_) {
        snapshot_qt_state();
        return;
    }

    const std::uint32_t mods = current_mods();

    for (int i = 0; i < 3; ++i) {
        const auto button = static_cast<vkkk::MouseButton>(i);
        const bool down = backend_->mouse_down(button);
        if (down == prev_mouse_[i]) {
            continue;
        }
        prev_mouse_[i] = down;
        if (down) {
            buttons_down_ |= (1u << i);
        }
        else {
            buttons_down_ &= ~(1u << i);
        }
        InputEvent event;
        event.kind = InputEvent::Kind::MouseButton;
        event.button = button;
        event.action = down ? vkkk::InputAction::Press : vkkk::InputAction::Release;
        event.mods = mods;
        event.x = cursor_x_;
        event.y = cursor_y_;
        dispatch(event);
    }

    const auto pointer = backend_->pointer();
    const double dx = cursor_valid_ ? pointer.x - cursor_x_ : 0.0;
    const double dy = cursor_valid_ ? pointer.y - cursor_y_ : 0.0;
    const bool moved = !cursor_valid_ || dx != 0.0 || dy != 0.0;
    cursor_x_ = pointer.x;
    cursor_y_ = pointer.y;
    cursor_valid_ = true;
    if (moved) {
        InputEvent event;
        event.mods = mods;
        event.x = pointer.x;
        event.y = pointer.y;
        event.dx = dx;
        event.dy = dy;
        event.kind = InputEvent::Kind::MouseMove;
        dispatch(event);
        if (buttons_down_ != 0) {
            event.kind = InputEvent::Kind::MouseDrag;
            for (int i = 0; i < 3; ++i) {
                if ((buttons_down_ & (1u << i)) == 0) {
                    continue;
                }
                event.button = static_cast<vkkk::MouseButton>(i);
                dispatch(event);
            }
        }
    }

    for (std::size_t i = 1; i < kKeyCount; ++i) {
        const auto key = static_cast<vkkk::Key>(i);
        const bool down = qt_key_active(*backend_, key);
        if (down == prev_keys_[i]) {
            continue;
        }
        prev_keys_[i] = down;
        InputEvent event;
        event.kind = InputEvent::Kind::Key;
        event.key = key;
        event.action = down ? vkkk::InputAction::Press : vkkk::InputAction::Release;
        event.mods = mods;
        event.x = cursor_x_;
        event.y = cursor_y_;
        dispatch(event);
    }

    if (auto* qt = dynamic_cast<vkkk::QtBackend*>(backend_)) {
        const float scroll = qt->take_scroll_delta();
        if (scroll != 0.0f) {
            InputEvent event;
            event.kind = InputEvent::Kind::Scroll;
            event.mods = mods;
            event.x = cursor_x_;
            event.y = cursor_y_;
            event.scroll_y = static_cast<double>(scroll);
            dispatch(event);
        }
    }
}

#endif

void ControlMap::poll() {
    if (backend_ == nullptr) {
        return;
    }
#if ORL_USE_QT6
    poll_qt_events();
#else
    sync_cursor();
#endif
    poll_holds();
}

void ControlMap::dispatch(const InputEvent& event) {
    if (auto* current = find_op(modal)) {
        if (current->active && !current->active()) {
            modal.clear();
        }
        else {
            invoke_modal(*current, event);
            if (current->active && !current->active()) {
                modal.clear();
            }
            return;
        }
    }

    for (const auto& binding : bindings_) {
        if (!matches(binding.input, event)) {
            continue;
        }
        for (const auto& name : binding.ops) {
            auto* op = find_op(name);
            if (op == nullptr) {
                continue;
            }
            if (op->mode == OpMode::Modal) {
                if (op->enter) {
                    op->enter();
                }
                if (op->active && op->active()) {
                    modal = op->name;
                }
                continue;
            }
            if (op->eval) {
                op->eval(event);
            }
        }
    }
}

void ControlMap::invoke_modal(BoundOp& op, const InputEvent& event) {
    if (event.kind == InputEvent::Kind::MouseButton
        && event.action == vkkk::InputAction::Press)
    {
        if (event.button == vkkk::MouseButton::Left) {
            if (op.confirm) {
                op.confirm();
            }
            return;
        }
        if (event.button == vkkk::MouseButton::Right) {
            if (op.cancel) {
                op.cancel();
            }
            return;
        }
    }
    if (event.kind == InputEvent::Kind::Key && event.action == vkkk::InputAction::Press
        && event.key == vkkk::Key::Escape)
    {
        if (op.cancel) {
            op.cancel();
        }
        return;
    }
    if (op.eval) {
        op.eval(event);
    }
}

ControlMap::BoundOp* ControlMap::find_op(std::string_view name) {
    if (name.empty()) {
        return nullptr;
    }
    for (auto& op : ops) {
        if (op.name == name) {
            return &op;
        }
    }
    return nullptr;
}

bool ControlMap::matches(const InputSpec& spec, const InputEvent& event) const {
    if (spec.mods != event.mods) {
        return false;
    }

    switch (spec.type) {
    case InputSpec::Type::Key:
        if (event.kind == InputEvent::Kind::Hold) {
            return spec.action == InputSpec::kHold
                && spec.code == static_cast<int>(event.key);
        }
        return event.kind == InputEvent::Kind::Key
            && spec.code == static_cast<int>(event.key)
            && spec.action == static_cast<int>(event.action);
    case InputSpec::Type::MouseButton:
        return event.kind == InputEvent::Kind::MouseButton
            && spec.code == static_cast<int>(event.button)
            && spec.action == static_cast<int>(event.action);
    case InputSpec::Type::MouseDrag:
        return event.kind == InputEvent::Kind::MouseDrag
            && spec.code == static_cast<int>(event.button);
    case InputSpec::Type::Scroll:
        return event.kind == InputEvent::Kind::Scroll;
    }
    return false;
}

std::uint32_t ControlMap::current_mods() const {
    return backend_ != nullptr ? backend_->modifiers() : 0;
}

#if !ORL_USE_QT6

ControlMap* ControlMap::map_for(GLFWwindow* window) {
    const auto it = g_window_maps.find(window);
    return it == g_window_maps.end() ? nullptr : it->second;
}

void ControlMap::key_callback(GLFWwindow* window, int key, int, int action, int mods) {
    auto* map = map_for(window);
    if (map == nullptr) {
        return;
    }
    InputEvent event;
    event.kind = InputEvent::Kind::Key;
    event.key = vkkk::key_from_glfw(key);
    event.action = vkkk::action_from_glfw(action);
    event.mods = vkkk::mods_from_glfw(mods);
    event.x = map->cursor_x_;
    event.y = map->cursor_y_;
    map->dispatch(event);
}

void ControlMap::mouse_button_callback(GLFWwindow* window, int button, int action, int mods) {
    auto* map = map_for(window);
    if (map == nullptr) {
        return;
    }
    const auto mapped = vkkk::mouse_from_glfw(button);
    const auto index = static_cast<unsigned>(mapped);
    if (action == GLFW_PRESS) {
        map->buttons_down_ |= (1u << index);
    }
    else if (action == GLFW_RELEASE) {
        map->buttons_down_ &= ~(1u << index);
    }

    InputEvent event;
    event.kind = InputEvent::Kind::MouseButton;
    event.button = mapped;
    event.action = vkkk::action_from_glfw(action);
    event.mods = vkkk::mods_from_glfw(mods);
    event.x = map->cursor_x_;
    event.y = map->cursor_y_;
    map->dispatch(event);
}

void ControlMap::cursor_pos_callback(GLFWwindow* window, double x, double y) {
    auto* map = map_for(window);
    if (map == nullptr) {
        return;
    }

    const double dx = map->cursor_valid_ ? x - map->cursor_x_ : 0.0;
    const double dy = map->cursor_valid_ ? y - map->cursor_y_ : 0.0;
    map->cursor_x_ = x;
    map->cursor_y_ = y;
    map->cursor_valid_ = true;

    InputEvent event;
    event.mods = map->current_mods();
    event.x = x;
    event.y = y;
    event.dx = dx;
    event.dy = dy;
    event.kind = InputEvent::Kind::MouseMove;
    map->dispatch(event);

    if (map->buttons_down_ == 0) {
        return;
    }

    event.kind = InputEvent::Kind::MouseDrag;
    for (int button = 0; button < 3; ++button) {
        if ((map->buttons_down_ & (1u << button)) == 0) {
            continue;
        }
        event.button = static_cast<vkkk::MouseButton>(button);
        map->dispatch(event);
    }
}

void ControlMap::scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
    auto* map = map_for(window);
    if (map == nullptr) {
        return;
    }
    InputEvent event;
    event.kind = InputEvent::Kind::Scroll;
    event.mods = map->current_mods();
    event.x = map->cursor_x_;
    event.y = map->cursor_y_;
    event.scroll_x = xoffset;
    event.scroll_y = yoffset;
    map->dispatch(event);
}

#endif

} // namespace ORL
