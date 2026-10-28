#pragma once

#include <cstdint>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

#include <glm/mat4x4.hpp>

#include "controller.hpp"
#include "deformer.hpp"
#include "joint.hpp"
#include "locator.hpp"
#include "weight.hpp"

namespace orlrig
{

struct ComponentId {
    std::uint64_t value = 0;

    explicit operator bool() const { return value != 0; }
    friend bool operator==(ComponentId, ComponentId) = default;
};

enum class TransformParentKind : std::uint8_t {
    World,
    Joint,
    Locator,
};

struct TransformParentRelation {
    ComponentId child;
    ComponentId parent;
    TransformParentKind parent_kind = TransformParentKind::World;
    glm::mat4 local_offset{1.0f};
    bool inherit_translation = true;
    bool inherit_rotation = true;
    bool inherit_scale = true;
    bool enabled = true;
};

enum class ComponentKind {
    Joint,
    Weight,
    Curve,
    Constraint,
    Deformer,
    Controller,
    Locator,
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
    // Controller setup and animation input transforms are authoring data.
    // Viewer handle shapes and display resources belong to the viewer
    // adapter, not this store.
    ComponentId create_controller(std::string name,
        Controller controller = Controller{});
    ComponentId create_locator(std::string name,
        Locator locator = Locator{});
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
    Locator* locator(ComponentId id);
    const Locator* locator(ComponentId id) const;
    WeightData* weight(ComponentId id);
    const WeightData* weight(ComponentId id) const;
    ConstraintData* constraint(ComponentId id);
    const ConstraintData* constraint(ComponentId id) const;
    DeformerData* deformer(ComponentId id);
    const DeformerData* deformer(ComponentId id) const;

    std::size_t size() const { return records.size(); }
    std::size_t size(ComponentKind kind) const;
    bool contains(std::string_view name) const;
    std::uint64_t topology_revision() const { return revision; }

    // Parent indices refer to this deterministic creation-order packing.
    std::vector<Joint> packed_joints() const;
    std::vector<ComponentId> packed_joint_ids() const;
    std::int64_t joint_index(ComponentId id) const;
    std::vector<Locator> packed_locators() const;
    std::vector<ComponentId> packed_locator_ids() const;
    std::int64_t locator_index(ComponentId id) const;

    bool set_transform_parent(TransformParentRelation relation,
        std::string* error = nullptr);
    bool clear_transform_parent(ComponentId child);
    const TransformParentRelation* transform_parent(
        ComponentId child) const;
    std::vector<TransformParentRelation> transform_parents() const;
    std::uint64_t relation_revision() const { return relation_revision_; }

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
        DeformerData,
        Locator>;

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
    std::uint64_t revision = 1;
    std::unordered_map<std::uint64_t, Record> records;
    std::unordered_map<std::string, std::uint64_t> names;
    std::vector<ComponentId> joint_order;
    std::vector<ComponentId> locator_order;
    std::unordered_map<std::uint64_t, TransformParentRelation>
        transform_parents_;
    std::uint64_t relation_revision_ = 1;
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
