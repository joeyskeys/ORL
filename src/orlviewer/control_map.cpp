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

namespace ORL
{
namespace
{

std::unordered_map<GLFWwindow*, ControlMap*> g_window_maps;

int mask_lock_mods(int mods) {
    return mods & ~(GLFW_MOD_CAPS_LOCK | GLFW_MOD_NUM_LOCK);
}

std::string to_lower(std::string_view text) {
    std::string out(text);
    for (char& c : out) {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    return out;
}

const std::unordered_map<std::string, int> kNamedKeys = {
    {"space", GLFW_KEY_SPACE},
    {"escape", GLFW_KEY_ESCAPE},
    {"esc", GLFW_KEY_ESCAPE},
    {"enter", GLFW_KEY_ENTER},
    {"return", GLFW_KEY_ENTER},
    {"tab", GLFW_KEY_TAB},
    {"backspace", GLFW_KEY_BACKSPACE},
    {"delete", GLFW_KEY_DELETE},
    {"insert", GLFW_KEY_INSERT},
    {"home", GLFW_KEY_HOME},
    {"end", GLFW_KEY_END},
    {"pageup", GLFW_KEY_PAGE_UP},
    {"pagedown", GLFW_KEY_PAGE_DOWN},
    {"left", GLFW_KEY_LEFT},
    {"right", GLFW_KEY_RIGHT},
    {"up", GLFW_KEY_UP},
    {"down", GLFW_KEY_DOWN},
    {"leftshift", GLFW_KEY_LEFT_SHIFT},
    {"rightshift", GLFW_KEY_RIGHT_SHIFT},
    {"leftctrl", GLFW_KEY_LEFT_CONTROL},
    {"leftcontrol", GLFW_KEY_LEFT_CONTROL},
    {"rightctrl", GLFW_KEY_RIGHT_CONTROL},
    {"leftalt", GLFW_KEY_LEFT_ALT},
    {"rightalt", GLFW_KEY_RIGHT_ALT},
    {"leftsuper", GLFW_KEY_LEFT_SUPER},
    {"rightsuper", GLFW_KEY_RIGHT_SUPER},
    {"minus", GLFW_KEY_MINUS},
    {"equal", GLFW_KEY_EQUAL},
    {"comma", GLFW_KEY_COMMA},
    {"period", GLFW_KEY_PERIOD},
    {"slash", GLFW_KEY_SLASH},
    {"semicolon", GLFW_KEY_SEMICOLON},
    {"apostrophe", GLFW_KEY_APOSTROPHE},
    {"grave", GLFW_KEY_GRAVE_ACCENT},
    {"leftbracket", GLFW_KEY_LEFT_BRACKET},
    {"rightbracket", GLFW_KEY_RIGHT_BRACKET},
    {"backslash", GLFW_KEY_BACKSLASH},
};

int parse_mod_name(std::string_view name) {
    const std::string key = to_lower(name);
    if (key == "shift") {
        return GLFW_MOD_SHIFT;
    }
    if (key == "ctrl" || key == "control") {
        return GLFW_MOD_CONTROL;
    }
    if (key == "alt") {
        return GLFW_MOD_ALT;
    }
    if (key == "super" || key == "cmd" || key == "meta") {
        return GLFW_MOD_SUPER;
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
            return GLFW_KEY_A + (c - 'A');
        }
        if (c >= '0' && c <= '9') {
            return GLFW_KEY_0 + (c - '0');
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
            if (index >= 1 && index <= 25) {
                return GLFW_KEY_F1 + (index - 1);
            }
        }
    }

    const auto it = kNamedKeys.find(key);
    if (it == kNamedKeys.end()) {
        throw std::runtime_error("unknown key '" + std::string(name) + "'");
    }
    return it->second;
}

int ControlMap::parse_mouse_button(std::string_view name) {
    const std::string key = to_lower(name);
    if (key == "left" || key == "0") {
        return GLFW_MOUSE_BUTTON_LEFT;
    }
    if (key == "right" || key == "1") {
        return GLFW_MOUSE_BUTTON_RIGHT;
    }
    if (key == "middle" || key == "2") {
        return GLFW_MOUSE_BUTTON_MIDDLE;
    }
    if (key == "button4" || key == "4") {
        return GLFW_MOUSE_BUTTON_4;
    }
    if (key == "button5" || key == "5") {
        return GLFW_MOUSE_BUTTON_5;
    }
    throw std::runtime_error("unknown mouse button '" + std::string(name) + "'");
}

int ControlMap::parse_mods(const std::vector<std::string>& names) {
    int mods = 0;
    for (const auto& name : names) {
        mods |= parse_mod_name(name);
    }
    return mods;
}

int ControlMap::parse_action(std::string_view name) {
    const std::string key = to_lower(name);
    if (key == "press" || key == "down") {
        return GLFW_PRESS;
    }
    if (key == "release" || key == "up") {
        return GLFW_RELEASE;
    }
    if (key == "repeat") {
        return GLFW_REPEAT;
    }
    if (key == "hold") {
        return InputSpec::kHold;
    }
    throw std::runtime_error("unknown action '" + std::string(name) + "'");
}

ControlMap::~ControlMap() {
    detach();
}

void ControlMap::attach(GLFWwindow* window) {
    detach();
    if (window == nullptr) {
        return;
    }
    window_ = window;
    g_window_maps[window_] = this;
    glfwSetKeyCallback(window_, key_callback);
    glfwSetMouseButtonCallback(window_, mouse_button_callback);
    glfwSetCursorPosCallback(window_, cursor_pos_callback);
    glfwSetScrollCallback(window_, scroll_callback);
    glfwGetCursorPos(window_, &cursor_x_, &cursor_y_);
    cursor_valid_ = true;
}

void ControlMap::detach() {
    if (window_ == nullptr) {
        return;
    }
    const auto it = g_window_maps.find(window_);
    if (it != g_window_maps.end() && it->second == this) {
        glfwSetKeyCallback(window_, nullptr);
        glfwSetMouseButtonCallback(window_, nullptr);
        glfwSetCursorPosCallback(window_, nullptr);
        glfwSetScrollCallback(window_, nullptr);
        g_window_maps.erase(it);
    }
    window_ = nullptr;
    buttons_down_ = 0;
    cursor_valid_ = false;
}

void ControlMap::bind_op(std::string op, OpHandler handler) {
    unbind_op(op);
    handlers_.emplace_back(std::move(op), std::move(handler));
}

void ControlMap::unbind_op(std::string_view op) {
    handlers_.erase(std::remove_if(handlers_.begin(), handlers_.end(),
                        [op](const auto& entry) { return entry.first == op; }),
        handlers_.end());
}

bool ControlMap::has_op(std::string_view op) const {
    return std::any_of(handlers_.begin(), handlers_.end(),
        [op](const auto& entry) { return entry.first == op; });
}

void ControlMap::map(InputSpec input, std::string op) {
    bindings_.push_back(ControlBinding{input, std::move(op)});
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
    bindings_.erase(std::remove_if(bindings_.begin(), bindings_.end(),
                        [op](const ControlBinding& binding) { return binding.op == op; }),
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
        if (!binding.IsObject() || !binding.HasMember("op") || !binding["op"].IsString()) {
            throw std::runtime_error("each binding needs an 'op' string");
        }
        if (!binding.HasMember("input")) {
            throw std::runtime_error("each binding needs an 'input' object");
        }
        map(parse_input_spec(binding["input"]), binding["op"].GetString());
    }
}

void ControlMap::poll() {
    if (window_ == nullptr) {
        return;
    }

    const int mods = current_mods();
    for (const auto& binding : bindings_) {
        if (binding.input.type != InputSpec::Type::Key
            || binding.input.action != InputSpec::kHold)
        {
            continue;
        }
        if (binding.input.mods != mods) {
            continue;
        }
        if (glfwGetKey(window_, binding.input.code) != GLFW_PRESS) {
            continue;
        }

        InputEvent event;
        event.kind = InputEvent::Kind::Hold;
        event.key = binding.input.code;
        event.mods = mods;
        event.x = cursor_x_;
        event.y = cursor_y_;
        dispatch(event);
    }
}

void ControlMap::dispatch(const InputEvent& event) {
    for (const auto& binding : bindings_) {
        if (!matches(binding.input, event)) {
            continue;
        }
        for (const auto& handler : handlers_) {
            if (handler.first == binding.op && handler.second) {
                handler.second(event);
            }
        }
    }
}

bool ControlMap::matches(const InputSpec& spec, const InputEvent& event) const {
    const int mods = mask_lock_mods(event.mods);
    if (spec.mods != mods) {
        return false;
    }

    switch (spec.type) {
    case InputSpec::Type::Key:
        if (event.kind == InputEvent::Kind::Hold) {
            return spec.action == InputSpec::kHold && spec.code == event.key;
        }
        return event.kind == InputEvent::Kind::Key
            && spec.code == event.key
            && spec.action == event.action;
    case InputSpec::Type::MouseButton:
        return event.kind == InputEvent::Kind::MouseButton
            && spec.code == event.button
            && spec.action == event.action;
    case InputSpec::Type::MouseDrag:
        return event.kind == InputEvent::Kind::MouseDrag
            && spec.code == event.button;
    case InputSpec::Type::Scroll:
        return event.kind == InputEvent::Kind::Scroll;
    }
    return false;
}

int ControlMap::current_mods() const {
    if (window_ == nullptr) {
        return 0;
    }
    int mods = 0;
    if (glfwGetKey(window_, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS
        || glfwGetKey(window_, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS)
    {
        mods |= GLFW_MOD_SHIFT;
    }
    if (glfwGetKey(window_, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS
        || glfwGetKey(window_, GLFW_KEY_RIGHT_CONTROL) == GLFW_PRESS)
    {
        mods |= GLFW_MOD_CONTROL;
    }
    if (glfwGetKey(window_, GLFW_KEY_LEFT_ALT) == GLFW_PRESS
        || glfwGetKey(window_, GLFW_KEY_RIGHT_ALT) == GLFW_PRESS)
    {
        mods |= GLFW_MOD_ALT;
    }
    if (glfwGetKey(window_, GLFW_KEY_LEFT_SUPER) == GLFW_PRESS
        || glfwGetKey(window_, GLFW_KEY_RIGHT_SUPER) == GLFW_PRESS)
    {
        mods |= GLFW_MOD_SUPER;
    }
    return mods;
}

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
    event.key = key;
    event.action = action;
    event.mods = mask_lock_mods(mods);
    event.x = map->cursor_x_;
    event.y = map->cursor_y_;
    map->dispatch(event);
}

void ControlMap::mouse_button_callback(GLFWwindow* window, int button, int action, int mods) {
    auto* map = map_for(window);
    if (map == nullptr) {
        return;
    }
    if (action == GLFW_PRESS) {
        map->buttons_down_ |= (1 << button);
    }
    else if (action == GLFW_RELEASE) {
        map->buttons_down_ &= ~(1 << button);
    }

    InputEvent event;
    event.kind = InputEvent::Kind::MouseButton;
    event.button = button;
    event.action = action;
    event.mods = mask_lock_mods(mods);
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
    if (map->buttons_down_ == 0) {
        return;
    }

    InputEvent event;
    event.kind = InputEvent::Kind::MouseDrag;
    event.mods = map->current_mods();
    event.x = x;
    event.y = y;
    event.dx = dx;
    event.dy = dy;
    for (int button = 0; button <= GLFW_MOUSE_BUTTON_LAST; ++button) {
        if ((map->buttons_down_ & (1 << button)) == 0) {
            continue;
        }
        event.button = button;
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

} // namespace ORL
