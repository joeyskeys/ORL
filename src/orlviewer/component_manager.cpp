#include "component_manager.hpp"

namespace ORL
{

ComponentId ComponentManager::create(std::string name, ComponentKind kind, Payload payload) {
    if (name.empty() || names.contains(name)) {
        return {};
    }

    const ComponentId id{next_id++};
    Record rec;
    rec.meta.id = id;
    rec.meta.name = name;
    rec.meta.kind = kind;
    rec.payload = std::move(payload);
    names.emplace(std::move(name), id.value);
    records.emplace(id.value, std::move(rec));
    if (kind == ComponentKind::Joint) {
        joint_order.push_back(id);
    }
    return id;
}

ComponentId ComponentManager::create_joint(std::string name, orlviewer::Joint joint) {
    return create(std::move(name), ComponentKind::Joint, std::move(joint));
}

ComponentId ComponentManager::create_curve(std::string name, CurveLink curve) {
    return create(std::move(name), ComponentKind::Curve, std::move(curve));
}

ComponentId ComponentManager::create_weight(std::string name, WeightData weight) {
    return create(std::move(name), ComponentKind::Weight, std::move(weight));
}

ComponentId ComponentManager::create_constraint(std::string name, ConstraintData constraint) {
    return create(std::move(name), ComponentKind::Constraint, std::move(constraint));
}

ComponentId ComponentManager::create_deformer(std::string name, DeformerData deformer) {
    return create(std::move(name), ComponentKind::Deformer, std::move(deformer));
}

bool ComponentManager::destroy(ComponentId id) {
    const auto found = records.find(id.value);
    if (found == records.end()) {
        return false;
    }
    names.erase(found->second.meta.name);
    if (found->second.meta.kind == ComponentKind::Joint) {
        std::erase(joint_order, id);
    }
    records.erase(found);
    return true;
}

bool ComponentManager::destroy(std::string_view name) {
    const auto* comp = find(name);
    return comp != nullptr && destroy(comp->id);
}

bool ComponentManager::rename(ComponentId id, std::string new_name) {
    auto* rec = record(id);
    if (rec == nullptr || new_name.empty()) {
        return false;
    }
    if (rec->meta.name == new_name) {
        return true;
    }
    if (names.contains(new_name)) {
        return false;
    }
    names.erase(rec->meta.name);
    rec->meta.name = new_name;
    names.emplace(std::move(new_name), id.value);
    return true;
}

bool ComponentManager::bind_display(ComponentId id, DisplayLink display) {
    auto* rec = record(id);
    if (rec == nullptr) {
        return false;
    }
    rec->meta.display = std::move(display);
    return true;
}

bool ComponentManager::unbind_display(ComponentId id) {
    return bind_display(id, {});
}

const Component* ComponentManager::find(ComponentId id) const {
    const auto* rec = record(id);
    return rec == nullptr ? nullptr : &rec->meta;
}

Component* ComponentManager::find(ComponentId id) {
    auto* rec = record(id);
    return rec == nullptr ? nullptr : &rec->meta;
}

const Component* ComponentManager::find(std::string_view name) const {
    const auto found = names.find(std::string{name});
    if (found == names.end()) {
        return nullptr;
    }
    return find(ComponentId{found->second});
}

Component* ComponentManager::find(std::string_view name) {
    const auto found = names.find(std::string{name});
    if (found == names.end()) {
        return nullptr;
    }
    return find(ComponentId{found->second});
}

orlviewer::Joint* ComponentManager::joint(ComponentId id) {
    return payload_as<orlviewer::Joint>(id);
}

const orlviewer::Joint* ComponentManager::joint(ComponentId id) const {
    return payload_as<orlviewer::Joint>(id);
}

CurveLink* ComponentManager::curve(ComponentId id) {
    return payload_as<CurveLink>(id);
}

const CurveLink* ComponentManager::curve(ComponentId id) const {
    return payload_as<CurveLink>(id);
}

WeightData* ComponentManager::weight(ComponentId id) {
    return payload_as<WeightData>(id);
}

const WeightData* ComponentManager::weight(ComponentId id) const {
    return payload_as<WeightData>(id);
}

ConstraintData* ComponentManager::constraint(ComponentId id) {
    return payload_as<ConstraintData>(id);
}

const ConstraintData* ComponentManager::constraint(ComponentId id) const {
    return payload_as<ConstraintData>(id);
}

DeformerData* ComponentManager::deformer(ComponentId id) {
    return payload_as<DeformerData>(id);
}

const DeformerData* ComponentManager::deformer(ComponentId id) const {
    return payload_as<DeformerData>(id);
}

std::size_t ComponentManager::size(ComponentKind kind) const {
    std::size_t count = 0;
    for (const auto& [_, rec] : records) {
        if (rec.meta.kind == kind) {
            ++count;
        }
    }
    return count;
}

bool ComponentManager::contains(std::string_view name) const {
    return names.contains(std::string{name});
}

std::vector<orlviewer::Joint> ComponentManager::packed_joints() const {
    std::vector<orlviewer::Joint> joints;
    joints.reserve(joint_order.size());
    for (const ComponentId id : joint_order) {
        if (const auto* value = joint(id)) {
            joints.push_back(*value);
        }
    }
    return joints;
}

std::int64_t ComponentManager::joint_index(ComponentId id) const {
    for (std::size_t i = 0; i < joint_order.size(); ++i) {
        if (joint_order[i] == id) {
            return static_cast<std::int64_t>(i);
        }
    }
    return -1;
}

ComponentManager::Record* ComponentManager::record(ComponentId id) {
    const auto found = records.find(id.value);
    return found == records.end() ? nullptr : &found->second;
}

const ComponentManager::Record* ComponentManager::record(ComponentId id) const {
    const auto found = records.find(id.value);
    return found == records.end() ? nullptr : &found->second;
}

} // namespace ORL
