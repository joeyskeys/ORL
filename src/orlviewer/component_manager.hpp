#pragma once

#include <cstdint>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

#include "comps/joint.hpp"
#include "concepts/curve.hpp"

namespace ORL
{

// Viewport-owned CPU identity for one user-created rig resource.
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
};

// Named handle into vkkk draw storage. ComponentManager never owns GPU
// resources; it only records which draw object (if any) represents this
// CPU component.
//
//   SceneObject -> vkkk::Scene objects
//   Mesh        -> DrawableMgr / Context meshes
//   Lines       -> DrawableMgr / Context lines
//   Points      -> Context points
enum class DisplayResourceKind {
    None,
    SceneObject,
    Mesh,
    Lines,
    Points,
};

struct DisplayLink {
    DisplayResourceKind kind = DisplayResourceKind::None;
    std::string name;

    bool bound() const {
        return kind != DisplayResourceKind::None && !name.empty();
    }
};

// Non-owning pointer to a vkkk curve. Control points and evaluation stay in
// vkkk; the viewport component only keeps this link. Tessellated draw data
// (if any) is a separate DisplayLink, typically DisplayResourceKind::Lines.
struct CurveLink {
    using Handle = std::variant<
        std::monostate,
        const vkkk::BezierCurve*,
        const vkkk::BSplineCurve*,
        const vkkk::NurbsCurve*,
        const vkkk::CatmullRomCurve*>;

    Handle handle;

    CurveLink() = default;
    CurveLink(const vkkk::BezierCurve& curve) : handle(&curve) {}
    CurveLink(const vkkk::BSplineCurve& curve) : handle(&curve) {}
    CurveLink(const vkkk::NurbsCurve& curve) : handle(&curve) {}
    CurveLink(const vkkk::CatmullRomCurve& curve) : handle(&curve) {}

    bool empty() const { return std::holds_alternative<std::monostate>(handle); }
};

struct WeightData {};
struct ConstraintData {};
struct DeformerData {};

struct Component {
    ComponentId id;
    std::string name;
    ComponentKind kind = ComponentKind::Joint;
    DisplayLink display;
};

// CPU-side registry of rig components the user can create and edit.
// Drawing stays in vkkk Scene / DrawableMgr / Context; bind_display() only
// stores the name of the matching draw object.
class ComponentManager {
public:
    ComponentId create_joint(std::string name, orlviewer::Joint joint = orlviewer::make_identity_joint());
    ComponentId create_curve(std::string name, CurveLink curve = {});
    ComponentId create_weight(std::string name, WeightData weight = {});
    ComponentId create_constraint(std::string name, ConstraintData constraint = {});
    ComponentId create_deformer(std::string name, DeformerData deformer = {});

    bool destroy(ComponentId id);
    bool destroy(std::string_view name);
    bool rename(ComponentId id, std::string new_name);

    bool bind_display(ComponentId id, DisplayLink display);
    bool unbind_display(ComponentId id);

    const Component* find(ComponentId id) const;
    Component* find(ComponentId id);
    const Component* find(std::string_view name) const;
    Component* find(std::string_view name);

    orlviewer::Joint* joint(ComponentId id);
    const orlviewer::Joint* joint(ComponentId id) const;
    CurveLink* curve(ComponentId id);
    const CurveLink* curve(ComponentId id) const;
    WeightData* weight(ComponentId id);
    const WeightData* weight(ComponentId id) const;
    ConstraintData* constraint(ComponentId id);
    const ConstraintData* constraint(ComponentId id) const;
    DeformerData* deformer(ComponentId id);
    const DeformerData* deformer(ComponentId id) const;

    std::size_t size() const { return records.size(); }
    std::size_t size(ComponentKind kind) const;
    bool contains(std::string_view name) const;

    // Joints in creation order, suitable for packing into Joint joints[].
    std::vector<orlviewer::Joint> packed_joints() const;
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
        orlviewer::Joint,
        CurveLink,
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
T* ComponentManager::payload_as(ComponentId id) {
    auto* rec = record(id);
    if (rec == nullptr) {
        return nullptr;
    }
    return std::get_if<T>(&rec->payload);
}

template <typename T>
const T* ComponentManager::payload_as(ComponentId id) const {
    const auto* rec = record(id);
    if (rec == nullptr) {
        return nullptr;
    }
    return std::get_if<T>(&rec->payload);
}

} // namespace ORL
