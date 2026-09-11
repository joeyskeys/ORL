#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <unordered_map>
#include <variant>
#include <vector>

#include "../orlexec/orlrig/component_store.hpp"
#include "comps/controller.hpp"
#include "comps/joint.hpp"
#include "comps/weight.hpp"
#include "comps/deformer.hpp"
#include "concepts/curve.hpp"

namespace ORL
{

using ComponentId = orlrig::ComponentId;
using ComponentKind = orlrig::ComponentKind;
using ConstraintData = orlrig::ConstraintData;

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

struct Component {
    ComponentId id;
    std::string name;
    ComponentKind kind = ComponentKind::Joint;
    DisplayLink display;
};

// Viewer adapter around the standalone rigging store. Display and curve
// handles stay here; pure rigging data lives in orlrig::ComponentStore.
class ComponentManager {
public:
    ComponentId create_joint(std::string name, orlviewer::Joint joint = orlviewer::make_identity_joint());
    ComponentId create_controller(std::string name,
        orlviewer::Controller controller = orlviewer::Controller{});
    ComponentId create_curve(std::string name, CurveLink curve = {});
    ComponentId create_weight(std::string name, WeightData weight = {});
    ComponentId create_constraint(std::string name, ConstraintData constraint = {});
    ComponentId create_deformer(std::string name, DeformerData deformer = {});

    bool destroy(ComponentId id);
    bool destroy(std::string_view name);
    void destroy_kind(ComponentKind kind);
    bool rename(ComponentId id, std::string new_name);

    bool bind_display(ComponentId id, DisplayLink display);
    bool unbind_display(ComponentId id);

    const Component* find(ComponentId id) const;
    Component* find(ComponentId id);
    const Component* find(std::string_view name) const;
    Component* find(std::string_view name);

    orlviewer::Joint* joint(ComponentId id);
    const orlviewer::Joint* joint(ComponentId id) const;
    orlviewer::Controller* controller(ComponentId id);
    const orlviewer::Controller* controller(ComponentId id) const;
    CurveLink* curve(ComponentId id);
    const CurveLink* curve(ComponentId id) const;
    WeightData* weight(ComponentId id);
    const WeightData* weight(ComponentId id) const;
    ConstraintData* constraint(ComponentId id);
    const ConstraintData* constraint(ComponentId id) const;
    DeformerData* deformer(ComponentId id);
    const DeformerData* deformer(ComponentId id) const;

    std::size_t size() const { return store.size(); }
    std::size_t size(ComponentKind kind) const;
    bool contains(std::string_view name) const;

    // Joints in creation order, suitable for packing into Joint joints[].
    std::vector<orlviewer::Joint> packed_joints() const;
    std::vector<ComponentId> packed_joint_ids() const;
    std::int64_t joint_index(ComponentId id) const;

    orlrig::ComponentStore& rigging() { return store; }
    const orlrig::ComponentStore& rigging() const { return store; }

    template <typename Fn>
    void for_each(Fn&& fn) const {
        for (const auto& [_, meta] : metadata) {
            fn(meta);
        }
    }

private:
    orlrig::ComponentStore store;
    std::unordered_map<std::uint64_t, Component> metadata;
    std::unordered_map<std::uint64_t, orlviewer::Controller> controllers;
    std::unordered_map<std::uint64_t, CurveLink> curves;
};

} // namespace ORL
