#include "component_manager.hpp"

#include <utility>

namespace ORL
{

namespace
{

Component make_meta(ComponentId id, std::string name, ComponentKind kind) {
    return Component{id, std::move(name), kind, {}};
}

} // namespace

ComponentId ComponentManager::create_joint(std::string name, orlviewer::Joint joint) {
    const auto id = store.create_joint(name, std::move(joint));
    if (id) {
        metadata.emplace(id.value, make_meta(id, std::move(name), ComponentKind::Joint));
    }
    return id;
}

ComponentId ComponentManager::create_controller(std::string name,
    orlviewer::Controller controller)
{
    orlrig::Controller core_controller;
    core_controller.xform = controller.xform;
    const auto id = store.create_controller(name, core_controller);
    if (id) {
        metadata.emplace(id.value, make_meta(id, name, ComponentKind::Controller));
        controllers.emplace(id.value, std::move(controller));
    }
    return id;
}

ComponentId ComponentManager::create_curve(std::string name, CurveLink curve) {
    const auto id = store.create_curve(name);
    if (id) {
        metadata.emplace(id.value, make_meta(id, name, ComponentKind::Curve));
        curves.emplace(id.value, std::move(curve));
    }
    return id;
}

ComponentId ComponentManager::create_weight(std::string name, WeightData weight) {
    const auto id = store.create_weight(name, std::move(weight));
    if (id) {
        metadata.emplace(id.value, make_meta(id, std::move(name), ComponentKind::Weight));
    }
    return id;
}

ComponentId ComponentManager::create_constraint(std::string name, ConstraintData constraint) {
    const auto id = store.create_constraint(name, std::move(constraint));
    if (id) {
        metadata.emplace(id.value, make_meta(id, std::move(name), ComponentKind::Constraint));
    }
    return id;
}

ComponentId ComponentManager::create_deformer(std::string name, DeformerData deformer) {
    const auto id = store.create_deformer(name, std::move(deformer));
    if (id) {
        metadata.emplace(id.value, make_meta(id, std::move(name), ComponentKind::Deformer));
    }
    return id;
}

bool ComponentManager::destroy(ComponentId id) {
    if (!store.destroy(id)) {
        return false;
    }
    metadata.erase(id.value);
    controllers.erase(id.value);
    curves.erase(id.value);
    return true;
}

bool ComponentManager::destroy(std::string_view name) {
    const auto* comp = find(name);
    return comp != nullptr && destroy(comp->id);
}

void ComponentManager::destroy_kind(ComponentKind kind) {
    std::vector<ComponentId> ids;
    for (const auto& [_, meta] : metadata) {
        if (meta.kind == kind) {
            ids.push_back(meta.id);
        }
    }
    for (const ComponentId id : ids) {
        destroy(id);
    }
}

bool ComponentManager::rename(ComponentId id, std::string new_name) {
    auto* meta = find(id);
    if (meta == nullptr || !store.rename(id, new_name)) {
        return false;
    }
    meta->name = std::move(new_name);
    return true;
}

bool ComponentManager::bind_display(ComponentId id, DisplayLink display) {
    auto* meta = find(id);
    if (meta == nullptr) {
        return false;
    }
    meta->display = std::move(display);
    return true;
}

bool ComponentManager::unbind_display(ComponentId id) {
    return bind_display(id, {});
}

const Component* ComponentManager::find(ComponentId id) const {
    const auto found = metadata.find(id.value);
    return found == metadata.end() ? nullptr : &found->second;
}

Component* ComponentManager::find(ComponentId id) {
    const auto found = metadata.find(id.value);
    return found == metadata.end() ? nullptr : &found->second;
}

const Component* ComponentManager::find(std::string_view name) const {
    for (const auto& [_, meta] : metadata) {
        if (meta.name == name) {
            return &meta;
        }
    }
    return nullptr;
}

Component* ComponentManager::find(std::string_view name) {
    for (auto& [_, meta] : metadata) {
        if (meta.name == name) {
            return &meta;
        }
    }
    return nullptr;
}

orlviewer::Joint* ComponentManager::joint(ComponentId id) {
    return store.joint(id);
}

const orlviewer::Joint* ComponentManager::joint(ComponentId id) const {
    return store.joint(id);
}

orlviewer::Controller* ComponentManager::controller(ComponentId id) {
    const auto found = controllers.find(id.value);
    return found == controllers.end() ? nullptr : &found->second;
}

const orlviewer::Controller* ComponentManager::controller(ComponentId id) const {
    const auto found = controllers.find(id.value);
    return found == controllers.end() ? nullptr : &found->second;
}

CurveLink* ComponentManager::curve(ComponentId id) {
    const auto found = curves.find(id.value);
    return found == curves.end() ? nullptr : &found->second;
}

const CurveLink* ComponentManager::curve(ComponentId id) const {
    const auto found = curves.find(id.value);
    return found == curves.end() ? nullptr : &found->second;
}

WeightData* ComponentManager::weight(ComponentId id) {
    return store.weight(id);
}

const WeightData* ComponentManager::weight(ComponentId id) const {
    return store.weight(id);
}

ConstraintData* ComponentManager::constraint(ComponentId id) {
    return store.constraint(id);
}

const ConstraintData* ComponentManager::constraint(ComponentId id) const {
    return store.constraint(id);
}

DeformerData* ComponentManager::deformer(ComponentId id) {
    return store.deformer(id);
}

const DeformerData* ComponentManager::deformer(ComponentId id) const {
    return store.deformer(id);
}

std::size_t ComponentManager::size(ComponentKind kind) const {
    return store.size(kind);
}

bool ComponentManager::contains(std::string_view name) const {
    return store.contains(name);
}

std::vector<orlviewer::Joint> ComponentManager::packed_joints() const {
    return store.packed_joints();
}

std::vector<ComponentId> ComponentManager::packed_joint_ids() const {
    return store.packed_joint_ids();
}

std::int64_t ComponentManager::joint_index(ComponentId id) const {
    return store.joint_index(id);
}

} // namespace ORL
