#include "component_store.hpp"

namespace orlrig
{

ComponentId ComponentStore::create(std::string name, ComponentKind kind, Payload payload) {
    if (name.empty() || names.contains(name)) {
        return {};
    }

    const ComponentId id{next_id++};
    Record rec;
    rec.meta.id = id;
    rec.meta.name = name;
    rec.meta.kind = kind;
    rec.payload = std::move(payload);
    names.emplace(name, id.value);
    records.emplace(id.value, std::move(rec));
    if (kind == ComponentKind::Joint) {
        joint_order.push_back(id);
    }
    return id;
}

ComponentId ComponentStore::create_joint(std::string name, Joint joint) {
    return create(std::move(name), ComponentKind::Joint, std::move(joint));
}

ComponentId ComponentStore::create_controller(std::string name, Controller controller) {
    return create(std::move(name), ComponentKind::Controller, std::move(controller));
}

ComponentId ComponentStore::create_curve(std::string name) {
    return create(std::move(name), ComponentKind::Curve, std::monostate{});
}

ComponentId ComponentStore::create_weight(std::string name, WeightData weight) {
    return create(std::move(name), ComponentKind::Weight, std::move(weight));
}

ComponentId ComponentStore::create_constraint(std::string name, ConstraintData constraint) {
    return create(std::move(name), ComponentKind::Constraint, std::move(constraint));
}

ComponentId ComponentStore::create_deformer(std::string name, DeformerData deformer) {
    return create(std::move(name), ComponentKind::Deformer, std::move(deformer));
}

bool ComponentStore::destroy(ComponentId id) {
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

bool ComponentStore::destroy(std::string_view name) {
    const auto* comp = find(name);
    return comp != nullptr && destroy(comp->id);
}

void ComponentStore::destroy_kind(ComponentKind kind) {
    std::vector<ComponentId> ids;
    for (const auto& [_, rec] : records) {
        if (rec.meta.kind == kind) {
            ids.push_back(rec.meta.id);
        }
    }
    for (const ComponentId id : ids) {
        destroy(id);
    }
}

bool ComponentStore::rename(ComponentId id, std::string new_name) {
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
    names.emplace(new_name, id.value);
    return true;
}

const Component* ComponentStore::find(ComponentId id) const {
    const auto* rec = record(id);
    return rec == nullptr ? nullptr : &rec->meta;
}

Component* ComponentStore::find(ComponentId id) {
    auto* rec = record(id);
    return rec == nullptr ? nullptr : &rec->meta;
}

const Component* ComponentStore::find(std::string_view name) const {
    const auto found = names.find(std::string{name});
    return found == names.end() ? nullptr : find(ComponentId{found->second});
}

Component* ComponentStore::find(std::string_view name) {
    const auto found = names.find(std::string{name});
    return found == names.end() ? nullptr : find(ComponentId{found->second});
}

Joint* ComponentStore::joint(ComponentId id) {
    return payload_as<Joint>(id);
}

const Joint* ComponentStore::joint(ComponentId id) const {
    return payload_as<Joint>(id);
}

Controller* ComponentStore::controller(ComponentId id) {
    return payload_as<Controller>(id);
}

const Controller* ComponentStore::controller(ComponentId id) const {
    return payload_as<Controller>(id);
}

WeightData* ComponentStore::weight(ComponentId id) {
    return payload_as<WeightData>(id);
}

const WeightData* ComponentStore::weight(ComponentId id) const {
    return payload_as<WeightData>(id);
}

ConstraintData* ComponentStore::constraint(ComponentId id) {
    return payload_as<ConstraintData>(id);
}

const ConstraintData* ComponentStore::constraint(ComponentId id) const {
    return payload_as<ConstraintData>(id);
}

DeformerData* ComponentStore::deformer(ComponentId id) {
    return payload_as<DeformerData>(id);
}

const DeformerData* ComponentStore::deformer(ComponentId id) const {
    return payload_as<DeformerData>(id);
}

std::size_t ComponentStore::size(ComponentKind kind) const {
    std::size_t count = 0;
    for (const auto& [_, rec] : records) {
        if (rec.meta.kind == kind) {
            ++count;
        }
    }
    return count;
}

bool ComponentStore::contains(std::string_view name) const {
    return names.contains(std::string{name});
}

std::vector<Joint> ComponentStore::packed_joints() const {
    std::vector<Joint> joints;
    joints.reserve(joint_order.size());
    for (const ComponentId id : joint_order) {
        if (const auto* value = joint(id)) {
            joints.push_back(*value);
        }
    }
    return joints;
}

std::vector<ComponentId> ComponentStore::packed_joint_ids() const {
    std::vector<ComponentId> ids;
    ids.reserve(joint_order.size());
    for (const ComponentId id : joint_order) {
        if (joint(id) != nullptr) {
            ids.push_back(id);
        }
    }
    return ids;
}

std::int64_t ComponentStore::joint_index(ComponentId id) const {
    for (std::size_t i = 0; i < joint_order.size(); ++i) {
        if (joint_order[i] == id) {
            return static_cast<std::int64_t>(i);
        }
    }
    return -1;
}

ComponentStore::Record* ComponentStore::record(ComponentId id) {
    const auto found = records.find(id.value);
    return found == records.end() ? nullptr : &found->second;
}

const ComponentStore::Record* ComponentStore::record(ComponentId id) const {
    const auto found = records.find(id.value);
    return found == records.end() ? nullptr : &found->second;
}

} // namespace orlrig
