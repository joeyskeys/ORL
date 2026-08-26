#pragma once

#include <string>
#include <vector>

#include <glm/gtc/matrix_inverse.hpp>
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
    glm::mat4 to_world{1.0f};

    explicit operator bool() const { return kind != XformAttrKind::None; }

    glm::vec3 world_position() const {
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
};

struct SelectionRef {
    enum class Kind {
        None,
        Joint,
        SceneObject,
        Vector,
    };

    Kind kind = Kind::None;
    ComponentId component;
    std::string object_name;
    glm::vec3* vector = nullptr;

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

    static SelectionRef point(glm::vec3* position) {
        SelectionRef ref;
        ref.kind = Kind::Vector;
        ref.vector = position;
        return ref;
    }
};

// Viewport selection. Transform ops read dest() / dests() for the attribute
// they should write: a matrix for objects that have one, otherwise a vector.
class Selection {
public:
    Selection(ComponentManager& components, vkkk::Scene& scene)
        : components(components)
        , scene(scene)
    {
    }

    void clear() {
        items.clear();
        sync_joint_states();
    }
    bool empty() const { return items.empty(); }
    std::size_t size() const { return items.size(); }

    void replace(SelectionRef ref) {
        items.clear();
        if (ref) {
            items.push_back(std::move(ref));
        }
        sync_joint_states();
    }

    void add(SelectionRef ref) {
        if (ref) {
            items.push_back(std::move(ref));
        }
        sync_joint_states();
    }

    const std::vector<SelectionRef>& refs() const { return items; }
    const SelectionRef* focus() const {
        return items.empty() ? nullptr : &items.back();
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
    void sync_joint_states() {
        components.for_each([&](const Component& meta) {
            if (meta.kind != ComponentKind::Joint) {
                return;
            }
            auto* joint = components.joint(meta.id);
            if (joint == nullptr) {
                return;
            }
            joint->selected = 0;
            joint->displayed = 0;
        });

        for (const auto& item : items) {
            if (item.kind != SelectionRef::Kind::Joint) {
                continue;
            }
            if (auto* joint = components.joint(item.component)) {
                joint->selected = 1;
            }
        }

        const auto packed = components.packed_joints();
        components.for_each([&](const Component& meta) {
            if (meta.kind != ComponentKind::Joint) {
                return;
            }
            auto* joint = components.joint(meta.id);
            if (joint == nullptr) {
                return;
            }
            bool display = joint->selected != 0;
            auto parent = joint->parent;
            while (!display && parent >= 0
                && static_cast<std::size_t>(parent) < packed.size())
            {
                if (packed[static_cast<std::size_t>(parent)].selected != 0) {
                    display = true;
                    break;
                }
                parent = packed[static_cast<std::size_t>(parent)].parent;
            }
            joint->displayed = display ? 1 : 0;
        });
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
            const auto packed = components.packed_joints();
            const auto index = components.joint_index(ref.component);
            if (index >= 0 && static_cast<std::size_t>(index) < packed.size()) {
                attr.to_world = orlviewer::joint_world_matrix(
                    packed, packed[static_cast<std::size_t>(index)].parent);
            }
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
};

} // namespace ORL
