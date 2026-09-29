#pragma once

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <functional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include <glm/gtc/matrix_inverse.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

#include "asset_mgr/scene.h"
#include "component_manager.hpp"

namespace ORL
{

enum class XformAttrKind {
    None,
    Matrix,
    Vector,
};

// Destination transform ops write. Matrix is an object xform; Vector is a
// position (joint translation, mesh point, etc.). Local vectors use to_world.
struct XformAttr {
    XformAttrKind kind = XformAttrKind::None;
    glm::mat4* matrix = nullptr;
    glm::vec3* vector = nullptr;
    double* translation = nullptr;
    double* rotation = nullptr;
    double* scale = nullptr;
    glm::mat4 to_world{1.0f};
    std::function<glm::mat4()> read_world;
    std::function<bool(const glm::mat4&)> write_world;

    explicit operator bool() const { return kind != XformAttrKind::None; }

    bool has_world_matrix() const {
        return matrix != nullptr
            || static_cast<bool>(read_world)
            || static_cast<bool>(write_world);
    }

    glm::mat4 world_matrix() const {
        if (read_world) {
            return read_world();
        }
        if (kind == XformAttrKind::Matrix && matrix != nullptr) {
            return *matrix;
        }
        return glm::mat4{1.0f};
    }

    bool set_world_matrix(const glm::mat4& world) {
        if (write_world) {
            return write_world(world);
        }
        if (kind == XformAttrKind::Matrix && matrix != nullptr) {
            *matrix = world;
            return true;
        }
        return false;
    }

    glm::vec3 world_position() const {
        if (read_world) {
            return glm::vec3{read_world()[3]};
        }
        if (kind == XformAttrKind::Matrix && matrix != nullptr) {
            return glm::vec3{(*matrix)[3]};
        }
        if (vector != nullptr) {
            return glm::vec3{to_world * glm::vec4{*vector, 1.0f}};
        }
        if (translation != nullptr) {
            return glm::vec3{to_world * glm::vec4{
                static_cast<float>(translation[0]),
                static_cast<float>(translation[1]),
                static_cast<float>(translation[2]),
                1.0f}};
        }
        return {};
    }

    void set_world_position(const glm::vec3& world) {
        if (write_world) {
            auto current = world_matrix();
            current[3] = glm::vec4{world, 1.0f};
            write_world(current);
            return;
        }
        glm::vec3 local = world;
        if (translation != nullptr || vector != nullptr) {
            local = glm::vec3{glm::inverse(to_world) * glm::vec4{world, 1.0f}};
        }
        if (kind == XformAttrKind::Matrix && matrix != nullptr) {
            (*matrix)[3] = glm::vec4{world, 1.0f};
            return;
        }
        if (vector != nullptr) {
            *vector = local;
            return;
        }
        if (translation != nullptr) {
            translation[0] = local.x;
            translation[1] = local.y;
            translation[2] = local.z;
        }
    }

    glm::vec3 local_axis(int index) const {
        if (index < 0 || index > 2) {
            return {};
        }
        const glm::mat4 basis = has_world_matrix()
            ? world_matrix()
            : to_world * glm::mat4_cast(local_rotation());
        const glm::vec3 axis{basis[index]};
        const float length = glm::length(axis);
        return length < 1.0e-6f ? glm::vec3{} : axis / length;
    }

    glm::quat local_rotation() const {
        if (rotation == nullptr) {
            return glm::quat{1.0f, 0.0f, 0.0f, 0.0f};
        }
        return glm::normalize(glm::quat{
            static_cast<float>(rotation[3]),
            static_cast<float>(rotation[0]),
            static_cast<float>(rotation[1]),
            static_cast<float>(rotation[2]),
        });
    }

    void set_local_rotation(const glm::quat& local) {
        if (rotation == nullptr) {
            return;
        }
        const glm::quat q = glm::normalize(local);
        rotation[0] = q.x;
        rotation[1] = q.y;
        rotation[2] = q.z;
        rotation[3] = q.w;
    }

    glm::vec3 local_scale() const {
        if (scale == nullptr) {
            return glm::vec3{1.0f};
        }
        return {
            static_cast<float>(scale[0]),
            static_cast<float>(scale[1]),
            static_cast<float>(scale[2]),
        };
    }

    void set_local_scale(const glm::vec3& local) {
        if (scale == nullptr) {
            return;
        }
        scale[0] = local.x;
        scale[1] = local.y;
        scale[2] = local.z;
    }
};

struct SelectionRef {
    enum class Kind {
        None,
        Joint,
        SceneObject,
        Vector,
        Controller,
        Locator,
    };

    Kind kind = Kind::None;
    ComponentId component;
    std::string object_name;
    glm::vec3* vector = nullptr;
    // A point owner can provide a stable token while retaining the pointer as
    // the current transform resolver. The pointer is only the compatibility
    // fallback for older callers that do not have an owner token.
    std::uint64_t vector_id = 0;

    explicit operator bool() const { return kind != Kind::None; }

    static SelectionRef joint(ComponentId id) {
        SelectionRef ref;
        ref.kind = Kind::Joint;
        ref.component = id;
        return ref;
    }

    static SelectionRef scene_object(std::string name) {
        SelectionRef ref;
        ref.kind = Kind::SceneObject;
        ref.object_name = std::move(name);
        return ref;
    }

    static SelectionRef point(glm::vec3* position, std::uint64_t id = 0) {
        SelectionRef ref;
        ref.kind = Kind::Vector;
        ref.vector = position;
        ref.vector_id = id;
        return ref;
    }

    static SelectionRef controller(ComponentId id) {
        SelectionRef ref;
        ref.kind = Kind::Controller;
        ref.component = id;
        return ref;
    }

    static SelectionRef locator(ComponentId id) {
        SelectionRef ref;
        ref.kind = Kind::Locator;
        ref.component = id;
        return ref;
    }
};

using SelectableKind = SelectionRef::Kind;
using SelectionMask = std::uint32_t;

inline constexpr SelectionMask selection_bit(SelectableKind kind) {
    switch (kind) {
    case SelectableKind::Joint:
        return 1u << 0;
    case SelectableKind::SceneObject:
        return 1u << 1;
    case SelectableKind::Vector:
        return 1u << 2;
    case SelectableKind::Controller:
        return 1u << 3;
    case SelectableKind::Locator:
        return 1u << 4;
    case SelectableKind::None:
        return 0;
    }
    return 0;
}

inline constexpr SelectionMask kAllSelectionKinds =
    selection_bit(SelectableKind::Joint)
    | selection_bit(SelectableKind::SceneObject)
    | selection_bit(SelectableKind::Vector)
    | selection_bit(SelectableKind::Controller)
    | selection_bit(SelectableKind::Locator);

inline constexpr bool selection_mask_allows(
    SelectionMask mask, SelectableKind kind)
{
    const auto bit = selection_bit(kind);
    return bit != 0 && (mask & bit) != 0;
}

enum class SelectionAction {
    Replace,
    Add,
};

enum class SelectionChangeReason {
    Programmatic,
    Pick,
    ModeChanged,
    Invalidation,
    Restore,
};

enum class SelectionResult {
    Replaced,
    Added,
    AlreadySelected,
    RejectedByMask,
    InvalidTarget,
    Cleared,
    NoOp,
};

enum class SelectionModeTransition {
    Preserve,
    PruneDisallowed,
};

struct SelectionMode {
    std::string id;
    std::string label;
    SelectionMask mask = kAllSelectionKinds;
    SelectionModeTransition transition =
        SelectionModeTransition::PruneDisallowed;
};

class SelectionModeRegistry {
public:
    bool register_mode(SelectionMode mode) {
        if (mode.id.empty() || modes.contains(mode.id)) {
            return false;
        }
        const auto id = mode.id;
        modes.emplace(id, std::move(mode));
        if (active_mode_id.empty()) {
            active_mode_id = id;
        }
        return true;
    }

    bool set_active(std::string_view id) {
        if (!modes.contains(std::string{id})) {
            return false;
        }
        active_mode_id = id;
        return true;
    }

    const SelectionMode* active() const {
        const auto found = modes.find(active_mode_id);
        return found == modes.end() ? nullptr : &found->second;
    }

    const SelectionMode* find(std::string_view id) const {
        const auto found = modes.find(std::string{id});
        return found == modes.end() ? nullptr : &found->second;
    }

    std::string_view active_id() const { return active_mode_id; }

private:
    std::unordered_map<std::string, SelectionMode> modes;
    std::string active_mode_id;
};

inline void register_default_selection_modes(SelectionModeRegistry& registry) {
    registry.register_mode(SelectionMode{
        .id = "all",
        .label = "All",
        .mask = kAllSelectionKinds,
    });
    registry.register_mode(SelectionMode{
        .id = "objects",
        .label = "Objects",
        .mask = selection_bit(SelectableKind::SceneObject),
    });
    registry.register_mode(SelectionMode{
        .id = "joints",
        .label = "Joints",
        .mask = selection_bit(SelectableKind::Joint),
    });
    registry.register_mode(SelectionMode{
        .id = "rig",
        .label = "Rig",
        .mask = selection_bit(SelectableKind::Joint)
            | selection_bit(SelectableKind::Controller)
            | selection_bit(SelectableKind::Locator),
    });
    registry.register_mode(SelectionMode{
        .id = "bind",
        .label = "Bind",
        .mask = selection_bit(SelectableKind::SceneObject)
            | selection_bit(SelectableKind::Joint),
    });
}

inline bool selection_refs_equal(
    const SelectionRef& first, const SelectionRef& second)
{
    if (first.kind != second.kind) {
        return false;
    }
    switch (first.kind) {
    case SelectionRef::Kind::Joint:
    case SelectionRef::Kind::Controller:
    case SelectionRef::Kind::Locator:
        return first.component == second.component;
    case SelectionRef::Kind::SceneObject:
        return first.object_name == second.object_name;
    case SelectionRef::Kind::Vector:
        if (first.vector_id != 0 || second.vector_id != 0) {
            return first.vector_id != 0
                && first.vector_id == second.vector_id;
        }
        return first.vector == second.vector;
    case SelectionRef::Kind::None:
        return true;
    }
    return false;
}

// Viewport selection. Transform ops read dest() / dests() for the attribute
// they should write: a matrix for objects that have one, otherwise a vector.
class Selection {
public:
    Selection(ComponentManager& components, vkkk::Scene& scene)
        : components(components)
        , scene(scene)
    {
    }

    void clear(SelectionChangeReason reason = SelectionChangeReason::Programmatic) {
        (void)reason;
        if (items.empty()) {
            return;
        }
        items.clear();
        ++revision_;
        sync_joint_states();
    }
    bool empty() const { return items.empty(); }
    std::size_t size() const { return items.size(); }

    SelectionResult replace(SelectionRef ref,
        SelectionChangeReason reason = SelectionChangeReason::Programmatic)
    {
        (void)reason;
        if (ref && !valid_target(ref)) {
            return SelectionResult::InvalidTarget;
        }
        const bool changed = items.size() != (ref ? 1u : 0u)
            || (ref && !selection_refs_equal(items.front(), ref));
        items.clear();
        if (ref) {
            items.push_back(std::move(ref));
        }
        if (changed) {
            ++revision_;
        }
        sync_joint_states();
        return items.empty() ? SelectionResult::Cleared
                             : SelectionResult::Replaced;
    }

    // Compatibility alias. New code should use replace() or apply().
    SelectionResult set(SelectionRef ref,
        SelectionChangeReason reason = SelectionChangeReason::Programmatic) {
        return replace(std::move(ref), reason);
    }

    SelectionResult apply(SelectionAction action, SelectionRef ref,
        SelectionChangeReason reason = SelectionChangeReason::Programmatic,
        bool enforce_mask = true)
    {
        if (!ref) {
            return SelectionResult::InvalidTarget;
        }
        if (!valid_target(ref)) {
            return SelectionResult::InvalidTarget;
        }
        if (enforce_mask && !selection_mask_allows(mask_, ref.kind)) {
            return SelectionResult::RejectedByMask;
        }
        if (action == SelectionAction::Replace) {
            return replace(std::move(ref), reason);
        }
        if (contains(ref)) {
            return SelectionResult::AlreadySelected;
        }
        if (ref) {
            items.push_back(std::move(ref));
        }
        ++revision_;
        sync_joint_states();
        return SelectionResult::Added;
    }

    SelectionResult add(SelectionRef ref,
        SelectionChangeReason reason = SelectionChangeReason::Programmatic)
    {
        return apply(SelectionAction::Add, std::move(ref), reason, false);
    }

    SelectionResult remove(const SelectionRef& ref,
        SelectionChangeReason reason = SelectionChangeReason::Programmatic)
    {
        (void)reason;
        const auto found = std::find_if(items.begin(), items.end(),
            [&ref](const SelectionRef& item) {
                return selection_refs_equal(item, ref);
            });
        if (found == items.end()) {
            return SelectionResult::NoOp;
        }
        items.erase(found);
        ++revision_;
        sync_joint_states();
        return SelectionResult::Replaced;
    }

    const std::vector<SelectionRef>& refs() const { return items; }
    const std::vector<SelectionRef>& ordered() const { return items; }
    bool contains(const SelectionRef& ref) const {
        return std::any_of(items.begin(), items.end(),
            [&ref](const SelectionRef& item) {
                return selection_refs_equal(item, ref);
            });
    }
    bool contains(SelectionRef::Kind kind, ComponentId id) const {
        return std::any_of(items.begin(), items.end(),
            [kind, id](const SelectionRef& item) {
                return item.kind == kind && item.component == id;
            });
    }
    bool is_selected(SelectionRef::Kind kind, ComponentId id) const {
        return contains(kind, id);
    }

    const SelectionRef* focus() const {
        return items.empty() ? nullptr : &items.back();
    }
    const SelectionRef* first() const {
        return items.empty() ? nullptr : &items.front();
    }
    const SelectionRef* at(std::size_t index) const {
        return index < items.size() ? &items[index] : nullptr;
    }

    SelectionMask mask() const { return mask_; }
    std::uint64_t revision() const { return revision_; }
    std::uint64_t mask_revision() const { return mask_revision_; }
    std::string_view mode() const { return mode_id_; }
    bool allows(SelectionRef::Kind kind) const {
        return selection_mask_allows(mask_, kind);
    }
    void set_mask(SelectionMask mask, bool prune_disallowed = true,
        SelectionChangeReason reason = SelectionChangeReason::ModeChanged) {
        (void)reason;
        const bool mask_changed = mask_ != mask;
        mask_ = mask;
        mode_id_.clear();
        if (mask_changed) {
            ++mask_revision_;
            ++revision_;
        }
        if (prune_disallowed) {
            prune_disallowed_items();
        }
        if (mask_changed || prune_disallowed) {
            sync_joint_states();
        }
    }

    bool set_mode(const SelectionMode& mode) {
        const bool mask_changed = mask_ != mode.mask;
        const bool mode_changed = mode_id_ != mode.id;
        mask_ = mode.mask;
        mode_id_ = mode.id;
        if (mask_changed || mode_changed) {
            ++mask_revision_;
            ++revision_;
        }
        if (mode.transition == SelectionModeTransition::PruneDisallowed) {
            prune_disallowed_items();
        }
        sync_joint_states();
        return true;
    }

    bool set_mode(std::string_view id, const SelectionModeRegistry& registry) {
        const auto* mode = registry.find(id);
        return mode != nullptr && set_mode(*mode);
    }

    bool activate_mode(std::string_view id, SelectionModeRegistry& registry) {
        const auto* mode = registry.find(id);
        if (mode == nullptr || !registry.set_active(id)) {
            return false;
        }
        return set_mode(*mode);
    }

    // Remove references whose owner no longer exists. This is intended to be
    // called at component/scene mutation boundaries, before consumers query
    // the selection. Surviving entries retain their original order.
    std::size_t prune_invalid() {
        const auto old_size = items.size();
        items.erase(std::remove_if(items.begin(), items.end(),
                        [this](const SelectionRef& item) {
                            switch (item.kind) {
                            case SelectionRef::Kind::Joint:
                            case SelectionRef::Kind::Controller:
                            case SelectionRef::Kind::Locator:
                                return components.find(item.component) == nullptr;
                            case SelectionRef::Kind::SceneObject:
                                return scene.find_object(item.object_name) == nullptr;
                            case SelectionRef::Kind::Vector:
                                return item.vector == nullptr;
                            case SelectionRef::Kind::None:
                                return true;
                            }
                            return true;
                        }),
            items.end());
        const auto removed = old_size - items.size();
        if (removed != 0) {
            ++revision_;
            sync_joint_states();
        }
        return removed;
    }

    void set_controller_input_mode(bool enabled) {
        controller_input_mode_ = enabled;
    }
    bool controller_input_mode() const { return controller_input_mode_; }

    std::string selected_mesh_name() const {
        for (const auto& item : items) {
            if (item.kind != SelectionRef::Kind::SceneObject) {
                continue;
            }
            const auto* object = scene.find_object(item.object_name);
            if (object != nullptr && !object->mesh_name.empty()) {
                return object->mesh_name;
            }
        }
        return {};
    }

    glm::mat4 selected_mesh_model() const {
        for (const auto& item : items) {
            if (item.kind != SelectionRef::Kind::SceneObject) {
                continue;
            }
            const auto* object = scene.find_object(item.object_name);
            if (object != nullptr && !object->mesh_name.empty()) {
                glm::mat4 model{1.0f};
                std::memcpy(&model, &object->model, sizeof(model));
                return model;
            }
        }
        return glm::mat4{1.0f};
    }

    bool has_selected_joint() const {
        for (const auto& item : items) {
            if (item.kind == SelectionRef::Kind::Joint) {
                return true;
            }
        }
        return false;
    }

    bool has_selected_controller() const {
        for (const auto& item : items) {
            if (item.kind == SelectionRef::Kind::Controller) {
                return true;
            }
        }
        return false;
    }

    bool has_selected_locator() const {
        for (const auto& item : items) {
            if (item.kind == SelectionRef::Kind::Locator) {
                return true;
            }
        }
        return false;
    }

    bool valid_for_bind() const {
        return !selected_mesh_name().empty() && has_selected_joint();
    }

    XformAttr dest(const SelectionRef& ref) const { return make_dest(ref); }

    XformAttr dest() const {
        return items.empty() ? XformAttr{} : make_dest(items.back());
    }

    std::vector<XformAttr> dests() const {
        std::vector<XformAttr> attrs;
        attrs.reserve(items.size());
        for (const auto& item : items) {
            if (auto attr = make_dest(item)) {
                attrs.push_back(attr);
            }
        }
        return attrs;
    }

private:
    bool valid_target(const SelectionRef& ref) const {
        switch (ref.kind) {
        case SelectionRef::Kind::Joint:
        case SelectionRef::Kind::Controller:
        case SelectionRef::Kind::Locator:
            return static_cast<bool>(ref.component)
                && components.find(ref.component) != nullptr;
        case SelectionRef::Kind::SceneObject:
            return !ref.object_name.empty()
                && scene.find_object(ref.object_name) != nullptr;
        case SelectionRef::Kind::Vector:
            return ref.vector != nullptr;
        case SelectionRef::Kind::None:
            return false;
        }
        return false;
    }

    void prune_disallowed_items() {
        const auto old_size = items.size();
        items.erase(std::remove_if(items.begin(), items.end(),
                        [this](const SelectionRef& item) {
                            return !allows(item.kind);
                        }),
            items.end());
        if (old_size != items.size()) {
            ++revision_;
        }
    }

    void sync_joint_states() {
        const auto packed = components.packed_joints();
        const auto ids = components.packed_joint_ids();
        std::vector<bool> direct(packed.size(), false);
        for (const auto& item : items) {
            if (item.kind != SelectionRef::Kind::Joint) {
                continue;
            }
            const auto index = components.joint_index(item.component);
            if (index >= 0 && static_cast<std::size_t>(index) < direct.size()) {
                direct[static_cast<std::size_t>(index)] = true;
            }
        }

        for (std::size_t index = 0; index < packed.size(); ++index) {
            bool selected = direct[index];
            auto parent = packed[index].parent;
            std::size_t traversed = 0;
            while (!selected && parent >= 0
                && static_cast<std::size_t>(parent) < packed.size()
                && traversed++ < packed.size())
            {
                selected = direct[static_cast<std::size_t>(parent)];
                parent = packed[static_cast<std::size_t>(parent)].parent;
            }
            if (index < ids.size()) {
                if (auto* joint = components.joint(ids[index])) {
                    joint->selected = selected ? 1u : 0u;
                }
            }
        }
    }

    XformAttr make_dest(const SelectionRef& ref) const {
        XformAttr attr;
        if (ref.kind == SelectionRef::Kind::Joint) {
            auto* joint = components.joint(ref.component);
            if (joint == nullptr) {
                return {};
            }
            attr.kind = XformAttrKind::Vector;
            attr.translation = joint->translation;
            attr.rotation = joint->rotation;
            attr.scale = joint->scale;
            const auto packed = components.packed_joints();
            const auto index = components.joint_index(ref.component);
            if (index >= 0 && static_cast<std::size_t>(index) < packed.size()) {
                attr.to_world = orlviewer::joint_world_matrix(
                    packed, packed[static_cast<std::size_t>(index)].parent);
            }
            attr.read_world = [this, id = ref.component] {
                const auto packed = components.packed_joints();
                const auto index = components.joint_index(id);
                if (index < 0 || static_cast<std::size_t>(index) >= packed.size()) {
                    return glm::mat4{1.0f};
                }
                return orlviewer::joint_world_matrix(packed, index);
            };
            attr.write_world = [this, id = ref.component](
                                   const glm::mat4& world) {
                auto* value = components.joint(id);
                if (value == nullptr) {
                    return false;
                }
                const auto packed = components.packed_joints();
                const auto index = components.joint_index(id);
                if (index < 0 || static_cast<std::size_t>(index) >= packed.size()) {
                    return false;
                }
                return orlrig::write_joint_world_matrix(
                    packed, index, world, *value);
            };
            return attr;
        }
        if (ref.kind == SelectionRef::Kind::SceneObject) {
            auto* object = scene.find_object(ref.object_name);
            if (object == nullptr) {
                return {};
            }
            attr.kind = XformAttrKind::Matrix;
            attr.matrix = &object->model;
            return attr;
        }
        if (ref.kind == SelectionRef::Kind::Controller) {
            auto* controller = components.controller(ref.component);
            if (controller == nullptr) {
                return {};
            }
            attr.kind = XformAttrKind::Matrix;
            attr.read_world = [this, id = ref.component] {
                return components.controller_world_xform(id);
            };
            attr.write_world = [this, id = ref.component](
                                   const glm::mat4& world) {
                return controller_input_mode_
                    ? components.set_controller_world_xform(id, world)
                    : components.set_controller_setup_world_xform(id, world);
            };
            return attr;
        }
        if (ref.kind == SelectionRef::Kind::Locator) {
            auto* locator = components.locator(ref.component);
            if (locator == nullptr) {
                return {};
            }
            attr.kind = XformAttrKind::Matrix;
            attr.matrix = &locator->xform;
            return attr;
        }
        if (ref.kind == SelectionRef::Kind::Vector && ref.vector != nullptr) {
            attr.kind = XformAttrKind::Vector;
            attr.vector = ref.vector;
            return attr;
        }
        return {};
    }

    ComponentManager& components;
    vkkk::Scene& scene;
    std::vector<SelectionRef> items;
    SelectionMask mask_ = kAllSelectionKinds;
    std::string mode_id_;
    std::uint64_t revision_ = 0;
    std::uint64_t mask_revision_ = 0;
    bool controller_input_mode_ = true;
};

} // namespace ORL
