#pragma once

#include <cstdint>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

#include "controller.hpp"
#include "deformer.hpp"
#include "joint.hpp"
#include "weight.hpp"

namespace orlrig
{

struct ComponentId {
    std::uint64_t value = 0;

    explicit operator bool() const { return value != 0; }
    friend bool operator==(ComponentId, ComponentId) = default;
};

enum class ComponentKind {
    Joint,
    Weight,
    Curve,
    Constraint,
    Deformer,
    Controller,
};

struct ConstraintData {
    std::string type = "ik_two_bone";
    ComponentId root;
    ComponentId mid;
    ComponentId end;
    ComponentId target;
    ComponentId pole;
    bool bound = false;
};

struct Component {
    ComponentId id;
    std::string name;
    ComponentKind kind = ComponentKind::Joint;
};

// Standalone data registry for rigging and deformation. It has no window,
// scene, renderer, or vkkk ownership.
class ComponentStore {
public:
    ComponentId create_joint(std::string name,
        Joint joint = make_identity_joint());
    ComponentId create_controller(std::string name,
        Controller controller = Controller{});
    ComponentId create_curve(std::string name);
    ComponentId create_weight(std::string name, WeightData weight = {});
    ComponentId create_constraint(std::string name,
        ConstraintData constraint = {});
    ComponentId create_deformer(std::string name,
        DeformerData deformer = {});

    bool destroy(ComponentId id);
    bool destroy(std::string_view name);
    void destroy_kind(ComponentKind kind);
    bool rename(ComponentId id, std::string new_name);

    const Component* find(ComponentId id) const;
    Component* find(ComponentId id);
    const Component* find(std::string_view name) const;
    Component* find(std::string_view name);

    Joint* joint(ComponentId id);
    const Joint* joint(ComponentId id) const;
    Controller* controller(ComponentId id);
    const Controller* controller(ComponentId id) const;
    WeightData* weight(ComponentId id);
    const WeightData* weight(ComponentId id) const;
    ConstraintData* constraint(ComponentId id);
    const ConstraintData* constraint(ComponentId id) const;
    DeformerData* deformer(ComponentId id);
    const DeformerData* deformer(ComponentId id) const;

    std::size_t size() const { return records.size(); }
    std::size_t size(ComponentKind kind) const;
    bool contains(std::string_view name) const;

    // Parent indices refer to this deterministic creation-order packing.
    std::vector<Joint> packed_joints() const;
    std::vector<ComponentId> packed_joint_ids() const;
    std::int64_t joint_index(ComponentId id) const;

    template <typename Fn>
    void for_each(Fn&& fn) const {
        for (const auto& [_, record] : records) {
            fn(record.meta);
        }
    }

private:
    using Payload = std::variant<
        std::monostate,
        Joint,
        Controller,
        WeightData,
        ConstraintData,
        DeformerData>;

    struct Record {
        Component meta;
        Payload payload;
    };

    ComponentId create(std::string name, ComponentKind kind, Payload payload);
    Record* record(ComponentId id);
    const Record* record(ComponentId id) const;

    template <typename T>
    T* payload_as(ComponentId id);
    template <typename T>
    const T* payload_as(ComponentId id) const;

    std::uint64_t next_id = 1;
    std::unordered_map<std::uint64_t, Record> records;
    std::unordered_map<std::string, std::uint64_t> names;
    std::vector<ComponentId> joint_order;
};

template <typename T>
T* ComponentStore::payload_as(ComponentId id) {
    auto* rec = record(id);
    return rec == nullptr ? nullptr : std::get_if<T>(&rec->payload);
}

template <typename T>
const T* ComponentStore::payload_as(ComponentId id) const {
    const auto* rec = record(id);
    return rec == nullptr ? nullptr : std::get_if<T>(&rec->payload);
}

} // namespace orlrig
